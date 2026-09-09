# Copyright 2026 sky
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Gate and supervise explore_lite without owning frontier selection."""

from datetime import datetime
import math
import os
import time
from typing import Dict, Optional

from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from .home_return import HomeReturn
from .map_home import save_home
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .health import evaluate_freshness, timestamped_prefix
from .health import validate_laser_scan
from .health import validate_occupancy_grid
from .health import validate_odometry


class ExplorationSupervisor(Node):
    """Start exploration only after inputs are fresh and stop on faults."""

    WAITING = "waiting_for_health"
    RUNNING = "running"
    PAUSED_FAULT = "paused_fault"
    PAUSED_OPERATOR = "paused_operator"
    STARTUP_TIMEOUT = "startup_timeout"
    TIMED_OUT = "exploration_timeout"
    COMPLETE = "complete"

    def __init__(self) -> None:
        super().__init__("exploration_supervisor")

        self._auto_start = self.declare_parameter("auto_start", True).value
        self._startup_timeout = float(
            self.declare_parameter("startup_timeout_s", 60.0).value)
        self._maximum_duration = float(
            self.declare_parameter("max_exploration_duration_s", 1800.0).value)
        self._startup_hold = float(
            self.declare_parameter("minimum_startup_hold_s", 3.0).value)
        self._healthy_cycles_required = int(
            self.declare_parameter("healthy_cycles_required", 5).value)
        self._timer_period = float(
            self.declare_parameter("supervision_period_s", 0.2).value)
        self._pause_republish = float(
            self.declare_parameter("pause_republish_s", 1.0).value)
        self._save_on_complete = self.declare_parameter(
            "save_on_complete", True).value
        self._save_on_timeout = self.declare_parameter(
            "save_on_timeout", True).value
        self._serialize_pose_graph = self.declare_parameter(
            "serialize_pose_graph", True).value
        self._map_prefix_pattern = self.declare_parameter(
            "map_output_prefix",
            "/home/sky/rk3576_footbath_ws/maps/footbath_auto_%Y%m%d_%H%M%S",
        ).value

        self._scan_topic = self.declare_parameter(
            "topics.high_scan", "/scan_high").value
        self._map_topic = self.declare_parameter("topics.map", "/map").value
        self._odom_topic = self.declare_parameter("topics.odom", "/odom").value
        self._status_topic = self.declare_parameter(
            "topics.explore_status", "/explore/status").value
        self._resume_topic = self.declare_parameter(
            "topics.explore_resume", "/explore/resume").value
        self._auto_cmd_topic = self.declare_parameter(
            "topics.auto_stop_cmd", "/cmd_vel").value
        self._lease_topic = self.declare_parameter(
            "topics.auto_motion_lease", "/safety/auto_motion_lease").value
        self._diagnostics_topic = self.declare_parameter(
            "topics.diagnostics", "/diagnostics").value

        self._cancel_service_name = self.declare_parameter(
            "services.cancel_navigation",
            "/navigate_to_pose/_action/cancel_goal",
        ).value
        self._save_service_name = self.declare_parameter(
            "services.slam_save_map", "/slam_toolbox/save_map").value
        self._serialize_service_name = self.declare_parameter(
            "services.slam_serialize_map",
            "/slam_toolbox/serialize_map",
        ).value

        self._maximum_age: Dict[str, float] = {
            "scan": float(self.declare_parameter("freshness.scan_s", 1.0).value),
            "map": float(self.declare_parameter("freshness.map_s", 5.0).value),
            "odom": float(self.declare_parameter("freshness.odom_s", 1.0).value),
        }
        self._last_seen: Dict[str, Optional[float]] = {
            "scan": None,
            "map": None,
            "odom": None,
        }
        self._ages: Dict[str, float] = {
            "scan": math.inf,
            "map": math.inf,
            "odom": math.inf,
        }
        self._invalid_counts: Dict[str, int] = {
            "scan": 0,
            "map": 0,
            "odom": 0,
        }
        self._last_invalid_reason: Dict[str, str] = {
            "scan": "none",
            "map": "none",
            "odom": "none",
        }

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_callback, transient_qos)
        self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 20)
        self.create_subscription(
            ExploreStatus, self._status_topic, self._status_callback, status_qos)

        self._resume_publisher = self.create_publisher(
            Bool, self._resume_topic, command_qos)
        self._lease_publisher = self.create_publisher(
            Bool, self._lease_topic, command_qos)
        self._stop_publisher = self.create_publisher(
            Twist, self._auto_cmd_topic, command_qos)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, self._diagnostics_topic, 10)

        self._cancel_client = self.create_client(
            CancelGoal, self._cancel_service_name)
        self._save_client = self.create_client(
            SaveMap, self._save_service_name)
        self._serialize_client = self.create_client(
            SerializePoseGraph, self._serialize_service_name)

        self.create_service(Trigger, "/exploration/start", self._start_service)
        self.create_service(Trigger, "/exploration/stop", self._stop_service)
        self.create_service(
            Trigger, "/exploration/save_map", self._save_map_service)

        now = time.monotonic()
        self._created_at = now
        self._exploration_started_at: Optional[float] = None
        self._last_pause_publish = 0.0
        self._last_diagnostic_publish = 0.0
        self._healthy_cycles = 0
        self._state = self.WAITING
        self._reason = "waiting for high scan, map and odometry"
        self._last_explore_status = "not_received"
        self._fault_latched = False
        self._save_in_progress = False
        self._save_state = "idle"
        self._last_map_prefix = ""
        self._nav_states = {}
        self._nav_pending = {}
        self._nav_clients = {
            name: self.create_client(GetState, '/' + name + '/get_state')
            for name in ('planner_server', 'controller_server', 'bt_navigator')
        }
        self._nav_timer = self.create_timer(1.0, self._poll_navigation)
        self._home_behavior_tree = os.path.join(
            get_package_share_directory('rk3576_footbath_exploration'),
            'behavior_trees', 'exploration_limited_recovery.xml')
        self.home = HomeReturn(self)
        if self.home.localization:
            self._auto_start = False
            for name in ('amcl', 'map_server'):
                self._nav_clients[name] = self.create_client(GetState, '/' + name + '/get_state')
        self._timer = self.create_timer(self._timer_period, self._supervise)

        self._publish_pause(force=True)
        self._publish_lease(False)
        self._publish_zero()
        self.get_logger().info(
            "Exploration supervisor waiting for "
            f"{self._scan_topic}, {self._map_topic} and {self._odom_topic}"
        )

    def _mark_seen(self, name: str) -> None:
        self._last_seen[name] = time.monotonic()

    def _record_invalid(self, name: str, reason: str) -> None:
        self._invalid_counts[name] += 1
        self._last_invalid_reason[name] = reason

    def _scan_callback(self, message: LaserScan) -> None:
        valid, reason = validate_laser_scan(
            message.angle_min,
            message.angle_max,
            message.angle_increment,
            message.range_min,
            message.range_max,
            message.ranges,
        )
        if valid:
            self._mark_seen("scan")
        else:
            self._record_invalid("scan", reason)

    def _map_callback(self, message: OccupancyGrid) -> None:
        valid, reason = validate_occupancy_grid(
            message.info.width,
            message.info.height,
            message.info.resolution,
            len(message.data),
        )
        if valid:
            self._mark_seen("map")
        else:
            self._record_invalid("map", reason)

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        valid, reason = validate_odometry(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
        )
        if valid:
            self.home.observe_odom(message)
            self._mark_seen("odom")
        else:
            self._record_invalid("odom", reason)

    def _status_callback(self, message: ExploreStatus) -> None:
        self._last_explore_status = message.status
        if message.status == ExploreStatus.EXPLORATION_COMPLETE:
            if self._state == self.COMPLETE:
                return
            if self._state != self.RUNNING:
                self.get_logger().warning(
                    "Ignoring exploration_complete while supervisor state is "
                    f"{self._state}"
                )
                return
            returning = False
            if self.home.auto_return:
                returning, reason = self.home.request()
                if not returning:
                    self.home.phase = 'failed'
                    self.home.message = '自动返航未启动: ' + reason
                    self.get_logger().warning('Auto-return not started: ' + reason)
            if not returning:
                self._pause(self.COMPLETE, "explore_lite reported no frontiers")
            if self._save_on_complete and not returning:
                self._request_map_save("exploration complete")

    def _publish_pause(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_pause_publish < self._pause_republish:
            return
        message = Bool()
        message.data = False
        self._resume_publisher.publish(message)
        self._last_pause_publish = now

    def _publish_resume(self) -> None:
        message = Bool()
        message.data = True
        self._resume_publisher.publish(message)

    def _publish_lease(self, allowed: bool) -> None:
        message = Bool()
        message.data = allowed
        self._lease_publisher.publish(message)

    def _publish_zero(self) -> None:
        self._stop_publisher.publish(Twist())

    def _cancel_navigation(self) -> None:
        if not self._cancel_client.service_is_ready():
            self.get_logger().warning(
                "NavigateToPose cancel service is not ready: "
                f"{self._cancel_service_name}"
            )
            return
        self._cancel_client.call_async(CancelGoal.Request())

    def _pause(self, state: str, reason: str) -> None:
        if state not in ('return_preparing', 'returning_home'):
            self.home.interrupt(reason)
        was_running = self._state == self.RUNNING
        self._state = state
        self._reason = reason
        self._publish_pause(force=True)
        self._publish_lease(False)
        self._publish_zero()
        if state != 'return_preparing' and (
                was_running or state in (self.COMPLETE, self.TIMED_OUT)):
            self._cancel_navigation()
        self.get_logger().warning(f"Exploration paused: {reason}")

    def _poll_navigation(self) -> None:
        now = time.monotonic()
        for name, client in self._nav_clients.items():
            pending = self._nav_pending.get(name)
            if pending is not None:
                future, started = pending
                if not future.done() and now - started < 2.0:
                    continue
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()
                self._nav_pending.pop(name, None)
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                self._nav_pending[name] = (future, now)
                future.add_done_callback(
                    lambda result, node=name: self._navigation_state(node, result))

    def _navigation_state(self, name, future) -> None:
        if future.cancelled():
            return
        try:
            self._nav_states[name] = (
                future.result().current_state.id, time.monotonic())
        except Exception:
            self._nav_states.pop(name, None)

    def _navigation_ready(self) -> bool:
        now = time.monotonic()
        return all(
            self._nav_states.get(name, (0, 0))[0] == 3
            and now - self._nav_states.get(name, (0, 0))[1] < 4.0
            for name in self._nav_clients)

    def _begin_exploration(self) -> None:
        dock = self.home.dock
        if self.home.navigation_session and dock.exit_complete:
            self._state = 'navigation_ready'
            self.home.phase = 'navigation_ready'
            self.home.message = '已完成50cm出站，可以规划普通导航目标'
            self._publish_lease(True)
            return True
        if dock.enabled and dock.auto_exit and not dock.exit_complete:
            try:
                dock.begin('exit')
                return True
            except ValueError as error:
                self.home._fail(str(error))
                return False
        self._state = self.RUNNING
        self._reason = "health gate passed"
        self._fault_latched = False
        self._exploration_started_at = time.monotonic()
        self._publish_resume()
        self._publish_lease(True)
        self.get_logger().info("Exploration resumed after health gate passed")
        return True

    def _start_service(self, _request: Trigger.Request, response: Trigger.Response):
        if self.home.localization:
            response.success = False
            response.message = '当前是已保存地图导航模式，不能继续探索'
            return response
        if self.home.pose is None or self.home.phase in self.home.BUSY:
            response.success = False
            response.message = '起点未记录或正在返航，不能启动探索'
            return response
        if not self._navigation_ready():
            response.success = False
            response.message = 'Nav2 not active; wait for navigation startup'
            return response
        healthy, self._ages = evaluate_freshness(
            time.monotonic(), self._last_seen, self._maximum_age,
            ("scan", "map", "odom"))
        if not healthy:
            response.success = False
            response.message = "inputs are not fresh: " + self._format_ages()
            return response
        self._healthy_cycles = self._healthy_cycles_required
        response.success = self._begin_exploration()
        response.message = self.home.message if self.home.dock.enabled and not self.home.dock.exit_complete else 'exploration resumed'
        return response

    def _stop_service(self, _request: Trigger.Request, response: Trigger.Response):
        self._pause(self.PAUSED_OPERATOR, "operator stop request")
        response.success = True
        response.message = "exploration paused and navigation cancel requested"
        return response

    def _save_map_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        accepted, message = self._request_map_save("operator request")
        response.success = accepted
        response.message = message
        return response

    def _home_inputs_ready(self) -> bool:
        self._refresh_static_map_health()
        healthy, _ = evaluate_freshness(
            time.monotonic(), self._last_seen, self._maximum_age,
            ('scan', 'map', 'odom'))
        return healthy and self._navigation_ready()

    def _motion_allowed(self) -> bool:
        return self._state in (self.RUNNING, 'returning_home', 'dock_motion', 'navigation_ready')

    def _refresh_static_map_health(self):
        # Static map_server is transient-local: no periodic SLAM map expected.
        # Only renew a previously validated map while map_server remains active.
        if (self.home.localization and self._last_seen.get('map') is not None
                and self._nav_states.get('map_server', (0, 0))[0] == 3
                and time.monotonic()-self._nav_states['map_server'][1] < 4):
            self._last_seen['map'] = time.monotonic()

    def _supervise(self) -> None:
        self._refresh_static_map_health()
        now = time.monotonic()
        healthy, self._ages = evaluate_freshness(
            now, self._last_seen, self._maximum_age,
            ("scan", "map", "odom"))
        healthy = healthy and self._navigation_ready()
        self.home.tick(healthy)
        self._healthy_cycles = self._healthy_cycles + 1 if healthy else 0

        if self._state == self.WAITING:
            self._publish_pause()
            self._publish_zero()
            ready_long_enough = now - self._created_at >= self._startup_hold
            enough_samples = self._healthy_cycles >= self._healthy_cycles_required
            if (self._auto_start and ready_long_enough and enough_samples
                    and self.home.pose is not None):
                self._begin_exploration()
            elif self._auto_start and now - self._created_at > self._startup_timeout:
                self._fault_latched = True
                self._pause(
                    self.STARTUP_TIMEOUT,
                    "startup health timeout; explicit /exploration/start required",
                )
        elif self._state == self.RUNNING:
            if not healthy:
                self._fault_latched = True
                self._pause(
                    self.PAUSED_FAULT,
                    "required input stale; explicit /exploration/start required",
                )
            elif (
                self._exploration_started_at is not None
                and now - self._exploration_started_at > self._maximum_duration
            ):
                self._pause(self.TIMED_OUT, "maximum exploration duration reached")
                if self._save_on_timeout:
                    self._request_map_save("exploration timeout")
        elif self._state == 'navigation_ready':
            if not healthy or self.home.current_pose() is None:
                self._pause(self.PAUSED_FAULT, '导航输入或定位失效，停车；恢复后请重新提交目标')
            else:
                self._publish_pause()
        elif self._state in ('returning_home', 'dock_motion'):
            self._publish_pause()
        else:
            self._publish_pause()
            self._publish_zero()

        # Short-lived heartbeat; /explore/resume remains exploration control only.
        self._publish_lease(
            self._motion_allowed())

        if now - self._last_diagnostic_publish >= 1.0:
            self._publish_diagnostics(healthy)
            self._last_diagnostic_publish = now

    def _request_map_save(self, reason: str):
        if self._save_in_progress:
            return False, "map save already in progress"
        if not os.path.isabs(self._map_prefix_pattern):
            self._save_state = "rejected_non_absolute_prefix"
            return False, "map_output_prefix must be absolute"
        if not self._save_client.service_is_ready():
            self._save_state = "save_service_unavailable"
            return False, f"service unavailable: {self._save_service_name}"

        prefix = timestamped_prefix(self._map_prefix_pattern, datetime.now())
        try:
            os.makedirs(os.path.dirname(prefix), exist_ok=True)
        except OSError as error:
            self._save_state = "directory_error"
            return False, f"cannot create map directory: {error}"

        request = SaveMap.Request()
        request.name.data = prefix
        self._save_in_progress = True
        self._save_state = "saving_occupancy_map"
        self._last_map_prefix = prefix
        future = self._save_client.call_async(request)
        future.add_done_callback(self._save_map_done)
        self.get_logger().info(f"Map save requested ({reason}): {prefix}")
        return True, f"map save accepted: {prefix}"

    def _save_map_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: B902 - ROS future propagates service errors.
            self._save_in_progress = False
            self._save_state = f"save_call_failed: {error}"
            self.get_logger().error(f"Map save service failed: {error}")
            return
        if response is None or response.result != SaveMap.Response.RESULT_SUCCESS:
            self._save_in_progress = False
            result = "none" if response is None else str(response.result)
            self._save_state = "save_map_failed_result_" + result
            self.get_logger().error(
                f"slam_toolbox SaveMap returned {result}")
            return
        if not self._serialize_pose_graph:
            self._save_in_progress = False
            self._save_state = "complete"
            self._persist_home()
            self.get_logger().info(
                f"Occupancy map saved: {self._last_map_prefix}")
            return
        if not self._serialize_client.service_is_ready():
            self._save_in_progress = False
            self._save_state = "serialize_service_unavailable"
            self.get_logger().error(
                "Pose-graph service unavailable: "
                f"{self._serialize_service_name}")
            return
        request = SerializePoseGraph.Request()
        request.filename = self._last_map_prefix + ".posegraph"
        future = self._serialize_client.call_async(request)
        self._save_state = "serializing_pose_graph"
        future.add_done_callback(self._serialize_done)

    def _serialize_done(self, future) -> None:
        self._save_in_progress = False
        try:
            response = future.result()
        except Exception as error:  # noqa: B902 - ROS future propagates service errors.
            self._save_state = f"serialize_call_failed: {error}"
            self.get_logger().error(
                f"Pose-graph serialization failed: {error}")
            return
        if (
            response is None
            or response.result != SerializePoseGraph.Response.RESULT_SUCCESS
        ):
            result = "none" if response is None else str(response.result)
            self._save_state = "serialize_failed_result_" + result
            self.get_logger().error(
                f"SerializePoseGraph returned {result}")
            return
        self._save_state = "complete"
        self._persist_home()
        self.get_logger().info(
            "Map and pose graph saved with prefix: "
            f"{self._last_map_prefix}")

    def _persist_home(self):
        try:
            save_home(self._last_map_prefix, self.home.pose)
        except Exception as error:
            self.get_logger().error('地图已保存但起点保存失败，必须手动初始化: '+str(error))

    def _format_ages(self) -> str:
        parts = []
        for name in ("scan", "map", "odom"):
            age = self._ages[name]
            parts.append(f"{name}={'never' if math.isinf(age) else f'{age:.2f}s'}")
        return ", ".join(parts)

    @staticmethod
    def _key_value(key: str, value: object) -> KeyValue:
        message = KeyValue()
        message.key = key
        message.value = str(value)
        return message

    def _publish_diagnostics(self, healthy: bool) -> None:
        status = DiagnosticStatus()
        status.name = "rk3576_footbath/exploration_supervisor"
        status.hardware_id = "rk3576"
        if self._motion_allowed() and healthy:
            status.level = DiagnosticStatus.OK
            status.message = "automatic exploration running"
        elif self._state in (self.WAITING, self.PAUSED_OPERATOR, self.COMPLETE):
            status.level = DiagnosticStatus.WARN
            status.message = self._reason
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = self._reason
        status.values = [
            self._key_value("state", self._state),
            self._key_value("reason", self._reason),
            self._key_value("inputs_healthy", healthy),
            self._key_value("input_ages", self._format_ages()),
            self._key_value("explore_status", self._last_explore_status),
            self._key_value("fault_latched", self._fault_latched),
            self._key_value(
                "auto_motion_lease",
                self._motion_allowed(),
            ),
            self._key_value("save_state", self._save_state),
            self._key_value("last_map_prefix", self._last_map_prefix),
        ]
        for name in ("scan", "map", "odom"):
            status.values.extend([
                self._key_value(
                    f"invalid_{name}_count", self._invalid_counts[name]),
                self._key_value(
                    f"last_invalid_{name}_reason",
                    self._last_invalid_reason[name],
                ),
            ])
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diagnostic_publisher.publish(array)

    def emergency_stop(self, reason: str) -> None:
        """Best-effort stop used during orderly node shutdown."""
        self._pause(self.PAUSED_OPERATOR, reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.emergency_stop("supervisor shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
