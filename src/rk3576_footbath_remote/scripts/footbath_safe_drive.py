#!/usr/bin/env python3
"""Bounded /cmd_vel_manual publisher protected by a root-owned short arm lease."""

from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist

ARM_FILE = Path("/run/footbath-motion-arm")
ENV_FILE = Path(os.environ.get("FOOTBATH_REMOTE_ENV", "/etc/footbath/remote-debug.env"))
MAX_LINEAR_MPS = 0.20
MAX_ANGULAR_RPS = 0.80
MAX_DURATION_S = 30.0


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


def lease_expiry() -> int:
    policy = read_key_value(ENV_FILE)
    if policy.get("FOOTBATH_ALLOW_REMOTE_MOTION") != "1":
        raise RuntimeError(f"remote motion is disabled in {ENV_FILE}")
    lease = read_key_value(ARM_FILE)
    try:
        expiry = int(lease["expires_epoch"])
    except (KeyError, ValueError) as exc:
        raise RuntimeError(
            "motion lease is missing; run sudo footbath_motion_arm arm ..."
        ) from exc
    if expiry <= int(time.time()):
        raise RuntimeError("motion lease expired; arm again with an onsite safety check")
    return expiry


def main() -> int:
    parser = argparse.ArgumentParser(description="Safely publish a bounded chassis velocity")
    parser.add_argument("--linear", type=float, required=True, help="m/s, absolute limit 0.20")
    parser.add_argument("--angular", type=float, default=0.0, help="rad/s, absolute limit 0.80")
    parser.add_argument("--duration", type=float, default=2.0, help="seconds, maximum 30")
    parser.add_argument("--rate", type=float, default=20.0, help="publish rate, 10..50 Hz")
    args = parser.parse_args()

    if abs(args.linear) > MAX_LINEAR_MPS:
        parser.error(f"abs(linear) must be <= {MAX_LINEAR_MPS}")
    if abs(args.angular) > MAX_ANGULAR_RPS:
        parser.error(f"abs(angular) must be <= {MAX_ANGULAR_RPS}")
    if not 0.1 <= args.duration <= MAX_DURATION_S:
        parser.error(f"duration must be 0.1..{MAX_DURATION_S} seconds")
    if not 10.0 <= args.rate <= 50.0:
        parser.error("rate must be 10..50 Hz")

    expiry = lease_expiry()
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    rclpy.init()
    node = rclpy.create_node("footbath_safe_drive")
    try:
        rclpy.spin_once(node, timeout_sec=0.75)
        existing_publishers = node.count_publishers("/cmd_vel_manual")
        subscribers = node.count_subscribers("/cmd_vel_manual")
        if existing_publishers > 0:
            raise RuntimeError(
                f"/cmd_vel_manual already has {existing_publishers} "
                "publisher(s); exactly one ROS manual source is allowed"
            )
        if subscribers < 1:
            raise RuntimeError(
                "/cmd_vel_manual has no subscriber; start hardware.launch.py "
                "and the command mux first"
            )

        publisher = node.create_publisher(Twist, "/cmd_vel_manual", 10)
        command = Twist()
        command.linear.x = float(args.linear)
        command.angular.z = float(args.angular)
        zero = Twist()
        period = 1.0 / args.rate
        deadline = time.monotonic() + args.duration
        print(
            f"DRIVE linear={args.linear:.3f}m/s angular={args.angular:.3f}rad/s "
            f"duration={args.duration:.1f}s",
            flush=True,
        )
        while not stop_requested and time.monotonic() < deadline:
            total_publishers = node.count_publishers("/cmd_vel_manual")
            if total_publishers > 1:
                publisher.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.0)
                raise RuntimeError(
                    "/cmd_vel_manual gained another publisher during motion; "
                    "STOP sent and test aborted"
                )
            if int(time.time()) >= expiry:
                print("Motion lease expired; stopping.", flush=True)
                break
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        for _ in range(6):
            publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        print("STOP sent; STM32 500 ms watchdog remains the final fallback.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
