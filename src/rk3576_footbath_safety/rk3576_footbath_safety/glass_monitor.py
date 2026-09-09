"""Publish a diagnostic-only suspected-glass flag from sensor disagreement."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool

from .logic import (
    front_sector_minimum,
    held_detection_active,
    suspected_glass,
)


class GlassSuspectMonitor(Node):
    """Compare one ultrasonic return with both lidar forward sectors."""

    def __init__(self) -> None:
        super().__init__("glass_suspect_monitor")
        self._high_topic = self.declare_parameter(
            "topics.high_scan", "/scan_high").value
        self._low_topic = self.declare_parameter(
            "topics.low_scan", "/scan_low_front").value
        self._ultrasonic_topic = self.declare_parameter(
            "topics.ultrasonic", "/range/ultrasonic").value
        self._output_topic = self.declare_parameter(
            "topics.output", "/safety/suspected_glass").value
        self._valid_topic = self.declare_parameter(
            "topics.valid", "/safety/suspected_glass_valid").value
        self._virtual_topic = self.declare_parameter(
            "topics.virtual_obstacle", "/range/suspected_glass").value
        self._half_angle = float(self.declare_parameter(
            "front_half_angle_rad", 0.261799388).value)
        self._trigger_max = float(self.declare_parameter(
            "ultrasonic_trigger_max_m", 1.50).value)
        self._margin = float(self.declare_parameter(
            "lidar_correspondence_margin_m", 0.15).value)
        self._freshness = float(self.declare_parameter(
            "sensor_freshness_timeout_s", 0.50).value)
        self._assert_samples = int(self.declare_parameter(
            "assert_consecutive_samples", 3).value)
        self._clear_samples = int(self.declare_parameter(
            "clear_consecutive_samples", 3).value)
        self._virtual_hold = float(self.declare_parameter(
            "virtual_obstacle_hold_s", 5.0).value)
        if min(
            self._half_angle,
            self._trigger_max,
            self._margin,
            self._freshness,
            self._virtual_hold,
        ) <= 0.0:
            raise ValueError("glass monitor limits must be positive")
        if self._assert_samples < 1 or self._clear_samples < 1:
            raise ValueError("debounce sample counts must be >= 1")

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            Bool, self._output_topic, latched_qos)
        self._valid_publisher = self.create_publisher(
            Bool, self._valid_topic, latched_qos)
        self._virtual_publisher = self.create_publisher(
            Range, self._virtual_topic, qos_profile_sensor_data)
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(
            LaserScan,
            self._high_topic,
            self._on_high,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self._low_topic,
            self._on_low,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            self._ultrasonic_topic,
            self._on_ultrasonic,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._evaluate)
        self.create_timer(1.0, self._publish_diagnostics)

        self._high_min = math.inf
        self._low_min = math.inf
        self._ultrasonic = math.nan
        self._last_ultrasonic_message = None
        self._last_candidate_time = None
        self._virtual_active = False
        self._last_seen = {"high": None, "low": None, "ultrasonic": None}
        self._assert_count = 0
        self._clear_count = 0
        self._suspected = False
        self._inputs_fresh = False
        self._publish_flag()
        self._publish_valid()
        self.get_logger().info(
            "Suspected-glass monitor publishes a held virtual range obstacle; "
            "it never commands chassis motion directly"
        )

    def _scan_minimum(self, message: LaserScan) -> float:
        return front_sector_minimum(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            self._half_angle,
        )

    def _on_high(self, message: LaserScan) -> None:
        self._high_min = self._scan_minimum(message)
        self._last_seen["high"] = time.monotonic()

    def _on_low(self, message: LaserScan) -> None:
        self._low_min = self._scan_minimum(message)
        self._last_seen["low"] = time.monotonic()

    def _on_ultrasonic(self, message: Range) -> None:
        value = float(message.range)
        if (
            math.isfinite(value)
            and message.min_range <= value <= message.max_range
        ):
            self._ultrasonic = value
            self._last_ultrasonic_message = message
        else:
            self._ultrasonic = math.nan
        self._last_seen["ultrasonic"] = time.monotonic()

    def _all_fresh(self, now: float) -> bool:
        return all(
            stamp is not None and now - stamp <= self._freshness
            for stamp in self._last_seen.values()
        )

    def _publish_flag(self) -> None:
        message = Bool()
        message.data = self._suspected
        self._publisher.publish(message)

    def _publish_valid(self) -> None:
        message = Bool()
        message.data = self._inputs_fresh
        self._valid_publisher.publish(message)

    def _publish_virtual_obstacle(self) -> None:
        source = self._last_ultrasonic_message
        if source is None:
            return
        message = Range()
        message.header.frame_id = source.header.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.radiation_type = source.radiation_type
        message.field_of_view = source.field_of_view
        message.min_range = source.min_range
        message.max_range = source.max_range
        message.range = (
            float(source.range) if self._virtual_active
            else float(source.max_range)
        )
        self._virtual_publisher.publish(message)

    def _evaluate(self) -> None:
        now = time.monotonic()
        previous_fresh = self._inputs_fresh
        self._inputs_fresh = self._all_fresh(now)
        if self._inputs_fresh != previous_fresh:
            self._publish_valid()
        candidate = self._inputs_fresh and suspected_glass(
            self._ultrasonic,
            self._high_min,
            self._low_min,
            self._trigger_max,
            self._margin,
        )
        if candidate:
            self._last_candidate_time = now
            self._assert_count += 1
            self._clear_count = 0
            if (
                not self._suspected
                and self._assert_count >= self._assert_samples
            ):
                self._suspected = True
                self._publish_flag()
        else:
            self._assert_count = 0
            self._clear_count += 1
            if self._suspected and self._clear_count >= self._clear_samples:
                self._suspected = False
                self._publish_flag()

        self._virtual_active = (
            self._last_ultrasonic_message is not None
            and (
                self._suspected
                or held_detection_active(
                    now, self._last_candidate_time, self._virtual_hold)
            )
        )
        self._publish_virtual_obstacle()

    @staticmethod
    def _kv(key: str, value: object) -> KeyValue:
        result = KeyValue()
        result.key = key
        result.value = str(value)
        return result

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "rk3576_footbath/glass_suspect_monitor"
        status.hardware_id = "high_c1+low_c1+front_ultrasonic"
        if not self._inputs_fresh:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "sensor input stale; suspected-glass flag forced false"
            )
        elif self._suspected:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "ultrasonic obstacle has no corresponding lidar return"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = "no ultrasonic/lidar disagreement"
        status.values = [
            self._kv("suspected_glass", self._suspected),
            self._kv("result_valid", self._inputs_fresh),
            self._kv("ultrasonic_m", self._ultrasonic),
            self._kv("high_front_min_m", self._high_min),
            self._kv("low_front_min_m", self._low_min),
            self._kv("diagnostic_only", False),
            self._kv("virtual_obstacle_active", self._virtual_active),
            self._kv("virtual_obstacle_hold_s", self._virtual_hold),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diagnostics.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlassSuspectMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
