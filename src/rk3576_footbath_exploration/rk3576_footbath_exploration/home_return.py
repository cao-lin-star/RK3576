"""Session-scoped home capture and fail-closed return through Nav2."""

import copy
import json
import math
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from .dock_motion import DockMotion, forward_point, pose_tuple


def home_pose_allowed(first, current, stationary_seconds):
    """Do not silently record a later location as the session start."""
    return (
        first is not None and current is not None
        and all(math.isfinite(v) for v in (*first, *current))
        and math.hypot(current[0] - first[0], current[1] - first[1]) <= 0.04
        and stationary_seconds >= 1.0)


class HomeReturn:
    BUSY = ('preparing', 'sending', 'returning', 'waiting_health', 'saving_map', 'handoff_ready',
            'undocking', 'docking', 'dock_waiting', 'dock_preparing', 'aligning')

    def __init__(self, node):
        self.node = node
        self.pose = None
        self.localization = bool(node.declare_parameter('return_home.localization_mode', False).value)
        self.navigation_session = bool(node.declare_parameter('dock.navigation_session', False).value)
        self.departure_required = bool(node.declare_parameter('dock.departure_required', True).value)
        restored = node.declare_parameter('return_home.pose_json', '').value
        self.handoff = None
        self.save_started = False
        self.last_motion = time.monotonic()
        self.phase = 'recording'
        self.message = '等待定位与静止，记录本次建图起点'
        self.first_odom = None
        self.current_odom = None
        self.stationary_since = None
        self.moved_before_capture = False
        self.generation = 0
        self.handle = None
        self.cancel_future = None
        self.active_goals = False
        self.last_status = 0.0
        self.distance = None
        self.started = 0.0
        self.stage_started = 0.0
        self.waiting_since = None
        self.healthy_since = None
        self.recovery_count = 0
        self.last_health_error = ''
        self.pose_error = ''
        self.cancel_done_at = None
        self.last_publish = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.nav = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.timeout = float(node.declare_parameter('return_home.timeout_s', 300.0).value)
        self.auto_return = bool(node.declare_parameter('return_home.on_complete', True).value)
        self.recovery_timeout = float(node.declare_parameter(
            'return_home.health_recovery_timeout_s', 8.0).value)
        self.stable_time = float(node.declare_parameter(
            'return_home.health_stable_s', 1.0).value)
        self.max_recoveries = int(node.declare_parameter(
            'return_home.health_max_recoveries', 5).value)
        if (not math.isfinite(self.recovery_timeout)
                or not math.isfinite(self.stable_time)
                or not 0 < self.stable_time < self.recovery_timeout
                or self.max_recoveries < 1):
            raise ValueError('invalid return-home recovery limits')
        if not math.isfinite(self.timeout) or self.timeout < 10:
            raise ValueError('return_home.timeout_s must be finite and >= 10')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, '/mapping/home_status', qos)
        node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._action_status, qos)
        node.create_service(Trigger, '/exploration/return_home', self._service)
        node.create_service(Trigger, '/exploration/depart', self._depart)
        self.dock = DockMotion(self)
        if self.localization and not self.navigation_session:
            data = json.loads(restored)
            self.pose = PoseStamped()
            self.pose.header.frame_id = 'map'
            self.pose.pose.position.x = float(data['x'])
            self.pose.pose.position.y = float(data['y'])
            yaw = float(data['yaw'])
            if not all(math.isfinite(v) for v in (data['x'], data['y'], yaw)):
                raise ValueError('invalid restored home')
            self.pose.pose.orientation.z = math.sin(yaw/2)
            self.pose.pose.orientation.w = math.cos(yaw/2)
            self.phase, self.message = 'ready', '已保留建图原起点，等待AMCL定位确认'

    def _depart(self, request, response):
        response.success = False
        if not self.navigation_session or self.phase in self.BUSY:
            response.message = '当前会话不允许普通导航出站，或已有任务运行'
            return response
        if self.dock.kind is not None or self.phase == 'arrived':
            response.message = '基站直线段已中断或已入站，请重新开始会话'
            return response
        current = self.current_pose()
        if not self.node._home_inputs_ready() or current is None or self.active_goals:
            response.message = '定位/传感器未就绪或旧导航未退出，保持停车'
            return response
        if not self.departure_required:
            self.dock.exit_complete = True
        if not self.dock.exit_complete:
            # Capture the confirmed AMCL position, not any pre-initialization TF.
            self.pose = current
            try:
                self.dock.begin('exit')
            except ValueError as error:
                response.message = str(error)
                return response
        else:
            self.node._begin_exploration()
        response.success = True
        response.message = '先沿车头方向直行50cm；完成前不会向Nav2发送目标'
        return response

    def _action_status(self, msg):
        self.active_goals = any(s.status in (1, 2, 3) for s in msg.status_list)
        self.last_status = time.monotonic()

    def observe_odom(self, msg):
        self.dock.observe_odom(msg)
        if abs(msg.twist.twist.linear.x) > .01 or abs(msg.twist.twist.angular.z) > .02:
            self.last_motion = time.monotonic()
        if self.pose is not None:
            return
        p = msg.pose.pose.position
        self.current_odom = (p.x, p.y)
        if self.first_odom is None:
            self.first_odom = self.current_odom
        twist = msg.twist.twist
        moving = abs(twist.linear.x) > 0.03 or abs(twist.angular.z) > 0.05
        displaced = math.hypot(p.x - self.first_odom[0], p.y - self.first_odom[1]) > 0.04
        if moving or displaced:
            self.moved_before_capture = True
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = time.monotonic()

    def current_pose(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
            stamp = Time.from_msg(tf.header.stamp).nanoseconds
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= age <= 1.0:
                self.pose_error = f'map->base_footprint TF age={age:.3f}s (limit 1.0s)'
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            if not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)):
                self.pose_error = 'map->base_footprint contains non-finite values'
                return None
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y = t.x, t.y
            p.pose.orientation = copy.deepcopy(q)
            self.pose_error = ''
            return p
        except TransformException as error:
            self.pose_error = str(error)
            return None

    def interrupt(self, reason):
        self.dock.pause()
        self.generation += 1
        if self.handle is not None:
            self.handle.cancel_goal_async()
            self.handle = None
        if self.phase in self.BUSY or self.phase == 'navigation_ready':
            self.phase = 'canceled'
            self.message = reason

    def request(self):
        n = self.node
        if self.phase in self.BUSY:
            return True, '返航已在执行，请勿重复提交'
        if self.dock.enabled and self.phase == 'arrived':
            return True, '已返回基站并停车'
        if self.dock.enabled and self.dock.kind is not None:
            return False, '基站直线段曾中断，请人工恢复位置后重新开始会话，禁止在基站内重新规划'
        if self.pose is None:
            return False, '本次建图尚未记录有效起点，不能返航'
        if not n._home_inputs_ready() or self.current_pose() is None:
            return False, '地图、定位、雷达或Nav2未就绪；保持停车，禁止盲目返航'
        if not n._cancel_client.service_is_ready() or not self.nav.server_is_ready():
            return False, '导航取消/目标服务尚未就绪'
        if not self.localization:
            n._pause('return_preparing', '停止探索，准备保存地图并切换定位导航')
            self.started = time.monotonic()
            self.phase = 'saving_map'
            self.message = '已停车，正在保存地图；返航不再继续建图'
            self.save_started = False
            self.cancel_future = n._cancel_client.call_async(CancelGoal.Request())
            return True, self.message
        self.generation += 1
        self.phase = 'preparing'
        self.message = '正在暂停探索并等待旧导航取消'
        self.started = time.monotonic()
        self.stage_started = self.started
        self.waiting_since = None
        self.healthy_since = None
        self.recovery_count = 0
        self.last_health_error = ''
        self.cancel_done_at = None
        self.distance = None
        n._pause('return_preparing', self.message)
        self.cancel_future = n._cancel_client.call_async(CancelGoal.Request())
        return True, '返航请求已接受；将保留当前地图并通过Nav2避障返回'

    def _service(self, request, response):
        response.success, response.message = self.request()
        return response

    def _fail(self, reason):
        self.phase, self.message = 'failed', reason
        self.node._pause(self.node.PAUSED_FAULT, reason)

    def _health_issue(self, healthy):
        if not healthy:
            return '输入/Nav2未就绪: ' + self.node._format_ages()
        if self.current_pose() is None:
            return '定位TF失效: ' + self.pose_error
        return ''

    def _wait_for_health(self, issue, now):
        # Revoke permission before canceling the action; never coast through
        # an invalid pose. Retain only the intent to return, not a stale goal.
        self.node._publish_lease(False)
        self.node._publish_zero()
        self.interrupt(issue)
        self.recovery_count += 1
        self.waiting_since = now
        self.healthy_since = None
        self.last_health_error = issue
        self.phase = 'waiting_health'
        self.message = '已停车，等待短暂恢复: ' + issue
        self.node._state = 'return_waiting'
        self.node._reason = self.message
        self.node._publish_pause(force=True)
        self.node.get_logger().warning(self.message)
        if self.recovery_count > self.max_recoveries:
            self._fail('定位/输入反复抖动，超过恢复次数限制，已取消返航: ' + issue)

    def _recover(self, now):
        if not self.node._cancel_client.service_is_ready():
            return
        self.phase = 'preparing'
        self.stage_started = now
        self.cancel_done_at = None
        self.message = '数据已持续恢复，确认旧目标退出后重新规划返航'
        self.node._state = 'return_preparing'
        self.node._reason = self.message
        self.cancel_future = self.node._cancel_client.call_async(CancelGoal.Request())
        self.node.get_logger().info(self.message)

    def _send(self):
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(self.pose)
        if self.dock.enabled:
            x, y, _ = forward_point(pose_tuple(self.pose), self.dock.distance)
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        # Bounded recovery, without repeatedly spinning in front of glass.
        goal.behavior_tree = self.node._home_behavior_tree
        self.phase = 'sending'
        self.message = '等待Nav2接受返航目标'
        generation = self.generation
        future = self.nav.send_goal_async(
            goal, feedback_callback=lambda m: self._feedback(m, generation))
        future.add_done_callback(lambda f: self._accepted(f, generation))

    def _accepted(self, future, generation):
        try:
            handle = future.result()
            if generation != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._fail('Nav2拒绝返航目标，已停车')
                return
            self.handle = handle
            self.progress_pose = self.current_pose()
            self.progress_at = time.monotonic()
            self.phase = 'returning'
            self.message = ('正在导航到基站正前方50cm并对齐车头'
                            if self.dock.enabled else '正在避障返回本次建图起点')
            self.node._state = 'returning_home'
            self.node._reason = self.message
            handle.get_result_async().add_done_callback(
                lambda f: self._result(f, generation))
        except Exception as error:
            if generation == self.generation:
                self._fail('返航目标发送失败: ' + str(error))

    def _feedback(self, msg, generation):
        if generation == self.generation:
            self.distance = float(msg.feedback.distance_remaining)

    def _result(self, future, generation):
        if generation != self.generation:
            return
        self.handle = None
        try:
            status = future.result().status
            if status == 4:
                if self.dock.enabled:
                    self.node._pause('return_preparing', '已到基站前方，等待旧导航退出后倒车')
                    self.phase = 'dock_preparing'
                    self.message = '等待Nav2退出，随后独立对齐朝向'
                    self.stage_started = time.monotonic()
                    return
                self.phase, self.message = 'arrived', '已返回起点并停车'
                self.node._pause(self.node.COMPLETE, self.message)
            else:
                self._fail('返航未完成，Nav2状态码 ' + str(status) + '；已停车，可检查后重试')
        except Exception as error:
            self._fail('返航结果读取失败: ' + str(error))

    def tick(self, healthy):
        now = time.monotonic()
        if self.dock.active:
            self.dock.tick(healthy)
            if now-self.last_publish>=.5:
                self.publish(); self.last_publish=now
            return
        if self.phase == 'dock_preparing':
            self.node._publish_zero()
            self.node._publish_lease(False)
            if now-self.stage_started>8:
                self._fail('入站准备超时，已停车')
            elif healthy and not self.active_goals and now-self.stage_started>=1.0:
                try:
                    self.dock.begin('align')
                except ValueError as error:
                    self._fail(str(error))
            self.publish()
            return
        if self.phase in ('saving_map', 'handoff_ready'):
            self.node._publish_lease(False)
            self.node._publish_zero()
            if now-self.started > 90:
                self._fail('保存地图/切换导航超时，保持停车')
            elif self.phase == 'saving_map':
                self._save_for_handoff(now)
            self.publish()
            return
        if self.pose is None:
            if self.moved_before_capture:
                self.phase, self.message = 'unavailable', '记录起点前已移动；本次禁止自动返航'
            elif healthy and home_pose_allowed(
                    self.first_odom, self.current_odom,
                    0 if self.stationary_since is None else now - self.stationary_since):
                self.pose = self.current_pose()
                if self.pose is not None:
                    self.phase, self.message = 'ready', '本次建图起点已记录'
                    self.node.get_logger().info(self.message)
        if self.phase in self.BUSY:
            issue = self._health_issue(healthy)
            if self.phase == 'returning':
                current = self.current_pose()
                if current is not None:
                    old = getattr(self, 'progress_pose', None)
                    if old is None or math.hypot(
                            current.pose.position.x-old.pose.position.x,
                            current.pose.position.y-old.pose.position.y) >= .03:
                        self.progress_pose, self.progress_at = current, now
                    elif now-getattr(self, 'progress_at', now) > 20:
                        self._fail('返航20秒无平移进展，已停车并取消目标；请检查局部障碍、控制器和底盘安全状态')
                        self.publish()
                        return
            if now - self.started > self.timeout:
                self._fail('返航超时，已停车；请检查是否存在玻璃或堵塞')
            elif self.phase == 'waiting_health':
                if now - self.waiting_since >= self.recovery_timeout:
                    self._fail('等待恢复超时，已取消返航并停车: ' + self.last_health_error)
                elif issue:
                    self.healthy_since = None
                    self.last_health_error = issue
                elif self.healthy_since is None:
                    self.healthy_since = now
                elif now - self.healthy_since >= self.stable_time:
                    self._recover(now)
            elif issue:
                self._wait_for_health(issue, now)
            elif self.phase == 'preparing':
                if not self.cancel_future.done():
                    if now - self.stage_started > 5:
                        self._fail('旧导航取消请求超时，未发出返航目标')
                else:
                    try:
                        result = self.cancel_future.result()
                        if result.return_code != 0:
                            self._fail('旧导航取消被拒绝，未发出返航目标')
                        elif self.cancel_done_at is None:
                            self.cancel_done_at = now
                        elif (now - self.cancel_done_at >= 1.0
                              and not self.active_goals
                              and (not result.goals_canceling
                                   or self.last_status >= self.cancel_done_at)):
                            self._send()
                        elif now - self.stage_started > 8:
                            self._fail('旧导航尚未退出，未发出返航目标')
                    except Exception as error:
                        self._fail('取消旧导航失败: ' + str(error))
            elif self.phase == 'sending' and now - self.stage_started > 15:
                self._fail('返航目标应答超时，已撤销运动许可')
        if now - self.last_publish >= 1.0:
            self.publish()
            self.last_publish = now

    def publish(self):
        pose = None
        if self.pose is not None:
            p, q = self.pose.pose.position, self.pose.pose.orientation
            pose = dict(x=p.x, y=p.y,
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        msg = String()
        msg.data = json.dumps(dict(
            session_started_at=self.node._created_at, updated_at=time.monotonic(),
            phase=self.phase, message=self.message, pose=pose,
            available=self.pose is not None, distance_remaining_m=self.distance,
            recovery_count=self.recovery_count, last_health_error=self.last_health_error,
            supervisor_state=self.node._state, auto_return=self.auto_return,
            supervisor_reason=self.node._reason,
            dock_enabled=self.dock.enabled, dock_distance_m=self.dock.distance,
            dock_progress_m=self.dock.progress,
            dock_exit_complete=self.dock.exit_complete,
            **(self.handoff or {})))
        self.publisher.publish(msg)

    def _save_for_handoff(self, now):
        n = self.node
        if now-self.last_motion < 1.0 or not n._home_inputs_ready():
            return
        if not self.cancel_future.done() or self.active_goals or now-self.started < 1.0:
            return
        try:
            if self.cancel_future.result().return_code != 0:
                self._fail('旧导航取消失败，禁止切换')
                return
            if not self.save_started:
                if n._save_in_progress:
                    return
                accepted, message = n._request_map_save('switch to localization return')
                if not accepted:
                    self._fail(message)
                    return
                self.save_started = True
                return
            if n._save_in_progress:
                return
            if n._save_state != 'complete':
                self._fail('地图保存未完成: ' + n._save_state)
                return
            current = self.current_pose()
            if current is None:
                return
            # Both poses retain the saved occupancy map's coordinate system.
            p, q = current.pose.position, current.pose.orientation
            self.handoff = dict(map_path=n._last_map_prefix+'.yaml', current_pose=dict(
                x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
            self.phase = 'handoff_ready'
            self.message = '地图保存成功，等待关闭SLAM并启动AMCL导航'
        except Exception as error:
            self._fail('保存/切换失败: ' + str(error))
