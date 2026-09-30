#!/usr/bin/env python3
"""Bounded, explicitly authorized field trial. No velocity publishers.

Uses the existing supervisor start/stop services. Not a safety-rated watchdog;
onsite emergency stop and all existing robot protections remain required.
"""
import argparse
import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatusArray
from trial_progress import TrialProgress
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-authorized', action='store_true')
    parser.add_argument('--stop-only', action='store_true')
    parser.add_argument('--monitor-running', action='store_true')
    parser.add_argument('--seconds', type=float, default=120)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 120:
        parser.error('Field trials must be between 1 and 120 seconds')
    if not (args.start_authorized or args.stop_only or args.monitor_running):
        parser.error('Explicit --start-authorized or --stop-only required')
    rclpy.init()
    node = rclpy.create_node('supervised_mapping_trial')

    def call(name):
        client = node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=4):
            raise RuntimeError('Unavailable service: ' + name)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Failed service: ' + name)
        return future.result().message

    progress = TrialProgress()
    seen, diag, state, poses = {}, {}, {}, progress.poses
    latest_command = [0., 0.]
    known_cells = [0]

    def odom(msg):
        q = msg.pose.pose.orientation
        now = time.monotonic()
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        progress.observe(now, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        seen['odom'] = now

    def grid(msg):
        seen['map'] = time.monotonic()
        known_cells[0] = sum(x >= 0 for x in msg.data)

    def diagnostic(msg):
        for item in msg.status:
            diag[item.name] = dict(message=item.message, values={v.key:v.value for v in item.values})

    def home(msg):
        state.update(json.loads(msg.data))

    def command(msg):
        latest_command[:] = [msg.linear.x, msg.angular.z]

    def navigation_status(msg):
        active=[s for s in msg.status_list if s.status in (1,2)]
        if len(active)>1:
            progress.set_goal(None)  # Ambiguous overlap cannot establish one-goal looping.
        else:
            progress.set_goal(tuple(active[0].goal_info.goal_id.uuid) if active else None)

    transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan_high', lambda m:seen.update(scan=time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', grid, transient)
    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10)
    node.create_subscription(String, '/mapping/home_status', home, transient)
    node.create_subscription(Twist, '/cmd_vel_selected', command, 10)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', navigation_status, transient)
    stop_attempted = False
    try:
        if args.stop_only:
            print(call('/exploration/stop'), flush=True)
            return
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        expected = ('running',) if args.monitor_running else ('paused_fault', 'paused_operator')
        assert state.get('supervisor_state') in expected, state
        assert state.get('dock_exit_complete'), 'Do not repeat dock departure in this trial'
        assert all(time.monotonic()-seen.get(k, 0)<limit for k,limit in [('scan',1),('odom',1),('map',5)]), seen
        if args.monitor_running:
            print('MONITOR existing running session; no start request', flush=True)
        else:
            assert max(abs(x) for x in latest_command)<.001, latest_command
            print('START '+call('/exploration/start'), flush=True)
        start = last_print = time.monotonic()
        poses.clear()
        reason = 'trial time limit reached'
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            now=time.monotonic()
            if any(now-seen.get(k,0)>limit for k,limit in [('scan',1),('odom',1),('map',5)]):
                reason='critical input stale'; break
            if now-start>3 and state.get('supervisor_state')!='running':
                reason='supervisor no longer running'; break
            displacement, rotation, looping = progress.summary()
            if looping:
                reason='repeated turning without net progress within one navigation goal'; break
            if now-last_print>=10:
                print(json.dumps(dict(elapsed=round(now-start,1),position=list(poses[-1])[1:] if poses else None,
                    displacement_30s=displacement,turn_30s_rad=rotation,cmd=latest_command,
                    known_cells=known_cells[0],state=state.get('supervisor_state'),diagnostics=diag)),flush=True)
                last_print=now
        print('STOP_REASON '+reason, flush=True)
    finally:
        if not args.stop_only:
            try:
                print('STOP '+call('/exploration/stop'), flush=True)
                stop_attempted=True
                end=time.monotonic()+3
                while time.monotonic()<end:
                    rclpy.spin_once(node, timeout_sec=.1)
                print(json.dumps(dict(stop_confirmed=stop_attempted and max(abs(x) for x in latest_command)<.001,
                                      cmd=latest_command, state=state.get('supervisor_state'))),flush=True)
            except Exception as error:
                print('STOP_UNCONFIRMED: onsite operator must stop robot: '+str(error),flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
