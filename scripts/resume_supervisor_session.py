#!/usr/bin/env python3
"""One-shot supervisor replacement, bound to an existing live launch session.

Does not launch SLAM, alter the map or request motion. A separate explicit
return-home service request is required after replacement verification.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def identity(pid):
    raw = Path(f'/proc/{pid}/stat').read_text()
    return raw[raw.rfind(')') + 2:].split()[19]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', required=True)
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--owner', type=int)
    parser.add_argument('--old-supervisor', type=int)
    args = parser.parse_args()
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if args.capture:
        owner_start = identity(args.owner)
        old_start = identity(args.old_supervisor)
        assert 'auto_mapping.launch.py' in Path(f'/proc/{args.owner}/cmdline').read_text()
        assert 'exploration_supervisor' in Path(f'/proc/{args.old_supervisor}/cmdline').read_text()
        rclpy.init()
        n = rclpy.create_node('capture_home_for_supervisor_update')
        captured = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(String, '/mapping/home_status',
                              lambda m: captured.append(json.loads(m.data)), qos)
        deadline = time.monotonic() + 5
        while not captured and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=0.1)
        assert captured, 'no home state received'
        status = captured[-1]
        assert status['available'] and status['pose']
        assert status['phase'] not in ('preparing', 'sending', 'returning', 'waiting_health')
        assert time.monotonic() - status['updated_at'] < 3
        status.update(owner=args.owner, owner_start=owner_start,
                      old_supervisor=args.old_supervisor, old_start=old_start, boot=boot)
        Path(args.state).write_text(json.dumps(status, ensure_ascii=False), encoding='utf-8')
        print(json.dumps(status, ensure_ascii=False))
        n.destroy_node()
        rclpy.shutdown()
        return

    saved = json.loads(Path(args.state).read_text(encoding='utf-8'))
    assert saved['boot'] == boot and identity(saved['owner']) == saved['owner_start']
    try:
        assert identity(saved['old_supervisor']) != saved['old_start'], 'old supervisor still alive'
    except FileNotFoundError:
        pass
    pose = saved['pose']
    assert all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw'))
    from ament_index_python.packages import get_package_share_directory
    from rk3576_footbath_exploration.supervisor import ExplorationSupervisor
    config = str(Path(get_package_share_directory('rk3576_footbath_exploration')) /
                 'config/supervisor.yaml')
    rclpy.init(args=['--ros-args', '--params-file', config, '-p', 'auto_start:=false'])
    node = ExplorationSupervisor()
    node._created_at = saved['session_started_at']
    node.home.pose = PoseStamped()
    node.home.pose.header.frame_id = 'map'
    node.home.pose.pose.position.x = pose['x']
    node.home.pose.pose.position.y = pose['y']
    node.home.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
    node.home.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
    node.home.phase = 'ready'
    node.home.message = '原建图起点已恢复，等待明确返航指令'
    node._state = node.PAUSED_OPERATOR
    node._reason = node.home.message

    def owner_watchdog():
        try:
            alive = identity(saved['owner']) == saved['owner_start']
        except (FileNotFoundError, ProcessLookupError):
            alive = False
        if not alive and rclpy.ok():
            node.emergency_stop('owning mapping session ended')
            rclpy.shutdown()

    node.create_timer(0.2, owner_watchdog)
    node.get_logger().info('Original home restored; owner-bound supervisor, no automatic motion')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.emergency_stop('replacement supervisor shutdown')
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
