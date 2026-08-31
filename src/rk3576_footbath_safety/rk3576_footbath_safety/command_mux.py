"""Select manual commands before limited automatic commands, fail-safe to zero."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .logic import clamp_diff_drive, select_fresh_command


class CommandMux(Node):
    """Continuously publish manual, then auto, or zero on a selected channel."""

    def __init__(self) -> None:
        super().__init__("footbath_command_mux")
        self._manual_topic = self.declare_parameter(
            "topics.manual", "/cmd_vel_manual").value
        self._auto_topic = self.declare_parameter(
            "topics.auto", "/cmd_vel_auto_limited").value
        self._output_topic = self.declare_parameter(
            "topics.output", "/cmd_vel_selected").value
        self._manual_timeout = float(self.declare_parameter(
            "manual_timeout_s", 0.30).value)
        self._auto_timeout = float(self.declare_parameter(
            "auto_timeout_s", 0.30).value)
        self._manual_max_linear = float(self.declare_parameter(
            "manual_max_linear_mps", 1.00).value)
        self._manual_max_angular = float(self.declare_parameter(
            "manual_max_angular_rps", 1.50).value)
        self._auto_max_linear = float(self.declare_parameter(
            "auto_max_linear_mps", 0.20).value)
        self._auto_max_angular = float(self.declare_parameter(
            "auto_max_angular_rps", 0.80).value)
        self._publish_period = float(self.declare_parameter(
            "publish_period_s", 0.05).value)
        if min(
            self._manual_timeout,
            self._auto_timeout,
            self._manual_max_linear,
            self._manual_max_angular,
            self._auto_max_linear,
            self._auto_max_angular,
            self._publish_period,
        ) <= 0.0:
            raise ValueError("mux limits, timeouts and period must be positive")
        if self._manual_max_linear > 1.0:
            raise ValueError("manual_max_linear_mps must not exceed 1.0")
        if self._auto_max_linear > 0.2:
            raise ValueError("auto_max_linear_mps must not exceed 0.2")

        self._publisher = self.create_publisher(Twist, self._output_topic, 10)
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Twist, self._manual_topic, self._on_manual, 10)
        self.create_subscription(Twist, self._auto_topic, self._on_auto, 10)
        self.create_timer(self._publish_period, self._publish_selected)
        self.create_timer(1.0, self._publish_diagnostics)

        self._manual_command = (0.0, 0.0)
        self._auto_command = (0.0, 0.0)
        self._manual_stamp = None
        self._auto_stamp = None
        self._selected_source = "none"
        self._manual_publisher_count = 0
        self._last_publisher_check = 0.0
        self._manual_conflict = False
        self._manual_rejected = 0
        self._auto_rejected = 0
        self._manual_clamped = 0
        self._auto_clamped = 0
        self._publisher.publish(Twist())
        self.get_logger().info(
            f"Command mux manual={self._manual_topic} "
            f"auto={self._auto_topic} output={self._output_topic}"
        )

    @staticmethod
    def _twist(command) -> Twist:
        message = Twist()
        message.linear.x = command[0]
        message.angular.z = command[1]
        return message

    @staticmethod
    def _was_clamped(message: Twist, command) -> bool:
        return (
            not math.isclose(message.linear.x, command[0])
            or not math.isclose(message.angular.z, command[1])
        )

    def _on_manual(self, message: Twist) -> None:
        now = time.monotonic()
        try:
            command = clamp_diff_drive(
                message.linear.x,
                message.angular.z,
                self._manual_max_linear,
                self._manual_max_angular,
            )
        except ValueError:
            self._manual_rejected += 1
            self._manual_command = (0.0, 0.0)
            self._manual_stamp = now
            return
        if self._was_clamped(message, command):
            self._manual_clamped += 1
        self._manual_command = command
        self._manual_stamp = now

    def _on_auto(self, message: Twist) -> None:
        now = time.monotonic()
        try:
            command = clamp_diff_drive(
                message.linear.x,
                message.angular.z,
                self._auto_max_linear,
                self._auto_max_angular,
            )
        except ValueError:
            self._auto_rejected += 1
            self._auto_command = (0.0, 0.0)
            self._auto_stamp = now
            return
        if self._was_clamped(message, command):
            self._auto_clamped += 1
        self._auto_command = command
        self._auto_stamp = now

    def _publish_selected(self) -> None:
        now = time.monotonic()
        # Keep the 20 Hz output path, but refresh the expensive DDS graph
        # query at 2 Hz. A conflict still fails closed within 0.5 s, matching
        # the STM32 final hard-stop envelope.
        if now - self._last_publisher_check >= 0.5:
            self._manual_publisher_count = self.count_publishers(
                self._manual_topic
            )
            self._last_publisher_check = now
        source, command = select_fresh_command(
            now,
            self._manual_command,
            self._manual_stamp,
            self._manual_timeout,
            self._auto_command,
            self._auto_stamp,
            self._auto_timeout,
            self._manual_publisher_count,
        )
        self._manual_conflict = source == "manual_conflict"
        if self._manual_conflict:
            self._manual_stamp = None
        self._selected_source = source
        self._publisher.publish(self._twist(command))

    @staticmethod
    def _kv(key: str, value: object) -> KeyValue:
        result = KeyValue()
        result.key = key
        result.value = str(value)
        return result

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "rk3576_footbath/command_mux"
        status.hardware_id = "rk3576"
        if self._manual_conflict:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                "multiple ROS manual publishers; fail-closed zero"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                f"selected command source: {self._selected_source}"
            )
        status.values = [
            self._kv("selected_source", self._selected_source),
            self._kv(
                "manual_publisher_count", self._manual_publisher_count
            ),
            self._kv("manual_conflict", self._manual_conflict),
            self._kv("manual_timeout_s", self._manual_timeout),
            self._kv("auto_timeout_s", self._auto_timeout),
            self._kv("manual_max_linear_mps", self._manual_max_linear),
            self._kv("auto_max_linear_mps", self._auto_max_linear),
            self._kv("manual_rejected", self._manual_rejected),
            self._kv("auto_rejected", self._auto_rejected),
            self._kv("manual_clamped", self._manual_clamped),
            self._kv("auto_clamped", self._auto_clamped),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diagnostics.publish(array)

    def stop(self) -> None:
        self._manual_stamp = None
        self._auto_stamp = None
        self._publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
