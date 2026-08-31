#!/usr/bin/env python3
"""Gate Foxglove Twist commands with the root-owned onsite arm lease."""

from __future__ import annotations

import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


ARM_FILE = Path("/run/footbath-motion-arm")
ENV_FILE = Path(
    os.environ.get("FOOTBATH_REMOTE_ENV", "/etc/footbath/remote-debug.env")
)


def read_key_value(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        for raw in path.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    except OSError:
        pass
    return values


class FoxgloveDriveGate(Node):
    """Forward only fresh, finite, armed commands to the manual mux input."""

    def __init__(self) -> None:
        super().__init__("footbath_foxglove_drive_gate")
        self._input_topic = self.declare_parameter(
            "input_topic", "/cmd_vel_foxglove"
        ).value
        self._output_topic = self.declare_parameter(
            "output_topic", "/cmd_vel_manual"
        ).value
        self._max_linear = float(
            self.declare_parameter("max_linear_mps", 1.00).value
        )
        self._max_angular = float(
            self.declare_parameter("max_angular_rps", 1.50).value
        )
        self._timeout = float(
            self.declare_parameter("command_timeout_s", 0.30).value
        )
        self._period = float(
            self.declare_parameter("publish_period_s", 0.05).value
        )
        if (
            self._max_linear <= 0.0
            or self._max_linear > 1.0
            or self._max_angular <= 0.0
            or self._timeout <= 0.0
            or self._period <= 0.0
        ):
            raise ValueError("invalid Foxglove gate limits")

        policy = read_key_value(ENV_FILE)
        self._policy_enabled = (
            policy.get("FOOTBATH_ALLOW_REMOTE_MOTION") == "1"
        )
        # Own /cmd_vel_manual only while actively forwarding.
        self._publisher = None
        self.create_subscription(
            Twist, self._input_topic, self._on_command, 10
        )
        self.create_timer(self._period, self._tick)
        self._command = Twist()
        self._last_command: float | None = None
        self._forwarding = False
        self._manual_conflict_active = False
        self._last_reject_log = 0.0
        self._last_conflict_log = 0.0
        self.get_logger().info(
            f"Foxglove gate {self._input_topic} -> {self._output_topic}; "
            f"policy_enabled={self._policy_enabled}"
        )

    def _lease_active(self) -> bool:
        if not self._policy_enabled:
            return False
        lease = read_key_value(ARM_FILE)
        try:
            expiry = int(lease["expires_epoch"])
        except (KeyError, ValueError):
            return False
        return expiry > int(time.time())

    def _release_output(self, send_zero: bool = False) -> None:
        if self._publisher is not None:
            if send_zero:
                self._publisher.publish(Twist())
            self.destroy_publisher(self._publisher)
            self._publisher = None
        self._forwarding = False

    def _warn_manual_conflict(self, now: float) -> None:
        if now - self._last_conflict_log >= 2.0:
            self.get_logger().warning(
                "Foxglove forwarding stopped: another ROS manual "
                "publisher owns the mux input"
            )
            self._last_conflict_log = now

    def _on_command(self, message: Twist) -> None:
        values = (message.linear.x, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self._last_command = None
            self._release_output(send_zero=True)
            return
        self._command = Twist()
        self._command.linear.x = max(
            -self._max_linear, min(self._max_linear, message.linear.x)
        )
        self._command.angular.z = max(
            -self._max_angular, min(self._max_angular, message.angular.z)
        )
        self._last_command = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        fresh = (
            self._last_command is not None
            and 0.0 <= now - self._last_command <= self._timeout
        )
        if not fresh or not self._lease_active():
            self._release_output(send_zero=self._forwarding)
            self._manual_conflict_active = False
            if fresh and now - self._last_reject_log >= 2.0:
                self.get_logger().warning(
                    "Foxglove command rejected: remote-motion policy or "
                    "root-owned arm lease is inactive"
                )
                self._last_reject_log = now
            return

        if self._publisher is None:
            if self.count_publishers(self._output_topic) > 0:
                self._manual_conflict_active = True
                self._warn_manual_conflict(now)
                return
            self._publisher = self.create_publisher(
                Twist, self._output_topic, 10
            )
            self._manual_conflict_active = False
        elif self.count_publishers(self._output_topic) > 1:
            self._release_output(send_zero=True)
            self._manual_conflict_active = True
            self._warn_manual_conflict(now)
            return

        self._publisher.publish(self._command)
        self._forwarding = True

    def stop(self) -> None:
        self._release_output(send_zero=self._forwarding)
        self._manual_conflict_active = False
        self._last_command = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FoxgloveDriveGate()
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


if __name__ == "__main__":
    main()
