"""Hard-limit the leased automatic velocity channel before command muxing."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .logic import clamp_diff_drive, motion_lease_allows


class AutoCmdVelLimiter(Node):
    """Fail closed unless a fresh supervisor lease authorizes motion."""

    def __init__(self) -> None:
        super().__init__("auto_cmd_vel_limiter")
        self._input_topic = self.declare_parameter(
            "input_topic", "/cmd_vel").value
        self._output_topic = self.declare_parameter(
            "output_topic", "/cmd_vel_auto_limited").value
        self._lease_topic = self.declare_parameter(
            "lease_topic", "/safety/auto_motion_lease").value
        self._require_lease = bool(self.declare_parameter(
            "require_lease", True).value)
        self._max_linear = float(self.declare_parameter(
            "max_linear_mps", 0.20).value)
        self._max_angular = float(self.declare_parameter(
            "max_angular_rps", 0.80).value)
        self._command_timeout = float(self.declare_parameter(
            "command_timeout_s", 0.30).value)
        self._lease_timeout = float(self.declare_parameter(
            "lease_timeout_s", 0.50).value)
        if self._max_linear <= 0.0 or self._max_linear > 0.20:
            raise ValueError("automatic max_linear_mps must be in (0, 0.20]")
        if min(
            self._max_angular,
            self._command_timeout,
            self._lease_timeout,
        ) <= 0.0:
            raise ValueError("angular, command and lease limits must be positive")

        self._publisher = self.create_publisher(Twist, self._output_topic, 10)
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Twist, self._input_topic, self._on_command, 10)
        self.create_subscription(Bool, self._lease_topic, self._on_lease, 10)
        self.create_timer(
            min(0.05, self._command_timeout / 2.0, self._lease_timeout / 2.0),
            self._watchdog,
        )
        self.create_timer(1.0, self._publish_diagnostics)

        self._lease_level = False
        self._last_lease = None
        self._last_command = None
        self._lease_timed_out = False
        self._command_timed_out = False
        self._clamped_count = 0
        self._rejected_count = 0
        self._publish_zero()
        self.get_logger().info(
            f"Leased automatic limiter: {self._input_topic} -> "
            f"{self._output_topic}, |v|<={self._max_linear:.3f} m/s"
        )

    def _publish_zero(self) -> None:
        self._publisher.publish(Twist())

    def _lease_active(self, now: float) -> bool:
        return motion_lease_allows(
            now,
            self._require_lease,
            self._lease_level,
            self._last_lease,
            self._lease_timeout,
        )

    def _on_lease(self, message: Bool) -> None:
        now = time.monotonic()
        was_active = self._lease_active(now)
        self._lease_level = bool(message.data)
        self._last_lease = now
        self._lease_timed_out = False
        if self._require_lease and not self._lease_level:
            self._last_command = None
            self._command_timed_out = False
            self._publish_zero()
        elif not was_active:
            self.get_logger().info("Fresh automatic motion lease received")

    def _on_command(self, message: Twist) -> None:
        now = time.monotonic()
        if not self._lease_active(now):
            self._rejected_count += 1
            self._publish_zero()
            return
        try:
            linear, angular = clamp_diff_drive(
                message.linear.x,
                message.angular.z,
                self._max_linear,
                self._max_angular,
            )
        except ValueError:
            self._rejected_count += 1
            self._last_command = None
            self._publish_zero()
            return
        output = Twist()
        output.linear.x = linear
        output.angular.z = angular
        if (
            not math.isclose(linear, message.linear.x)
            or not math.isclose(angular, message.angular.z)
        ):
            self._clamped_count += 1
        self._last_command = now
        self._command_timed_out = False
        self._publisher.publish(output)

    def _watchdog(self) -> None:
        now = time.monotonic()
        if not self._lease_active(now):
            self._lease_timed_out = (
                self._require_lease
                and self._lease_level
                and self._last_lease is not None
                and now - self._last_lease > self._lease_timeout
            )
            self._last_command = None
            self._command_timed_out = False
            self._publish_zero()
            return
        self._lease_timed_out = False
        if (
            self._last_command is None
            or now - self._last_command > self._command_timeout
        ):
            self._command_timed_out = True
            self._publish_zero()
        else:
            self._command_timed_out = False

    @staticmethod
    def _kv(key: str, value: object) -> KeyValue:
        result = KeyValue()
        result.key = key
        result.value = str(value)
        return result

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        active = self._lease_active(now)
        lease_age = (
            math.inf if self._last_lease is None else now - self._last_lease
        )
        status = DiagnosticStatus()
        status.name = "rk3576_footbath/auto_cmd_vel_limiter"
        status.hardware_id = "rk3576"
        if self._lease_timed_out:
            status.level = DiagnosticStatus.ERROR
            status.message = "automatic lease timeout; fail-closed zero"
        elif not active:
            status.level = DiagnosticStatus.WARN
            status.message = "automatic motion lease inactive"
        elif self._command_timed_out:
            status.level = DiagnosticStatus.ERROR
            status.message = "automatic command timeout; publishing zero"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "automatic command leased, limited and healthy"
        status.values = [
            self._kv("require_lease", self._require_lease),
            self._kv("lease_active", active),
            self._kv("lease_level", self._lease_level),
            self._kv("lease_age_s", lease_age),
            self._kv("lease_timeout_s", self._lease_timeout),
            self._kv("max_linear_mps", self._max_linear),
            self._kv("max_angular_rps", self._max_angular),
            self._kv("command_timeout_s", self._command_timeout),
            self._kv("clamped_commands", self._clamped_count),
            self._kv("rejected_commands", self._rejected_count),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diagnostics.publish(array)

    def stop(self) -> None:
        self._lease_level = False
        self._last_lease = None
        self._publish_zero()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoCmdVelLimiter()
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
