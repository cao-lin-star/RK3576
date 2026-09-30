"""Offline controller comparison. NEVER runs in the robot DDS domain.

Reads a CLOSED bag, starts only a local isolated controller with snapshot inputs,
and remaps all command output to /offline_cmd. No hardware or bridge is launched.
"""
import argparse
import copy
import glob
import importlib.util
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time

import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowPath
from dwb_msgs.msg import LocalPlanEvaluation
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from launch import LaunchContext


def snapshot(folder, stamp):
    wanted = ['/odom', '/plan', '/local_costmap/costmap', '/tf']
    latest = {}
    for filename in sorted(glob.glob(str(Path(folder) / '*.db3'))):
        with sqlite3.connect('file:' + filename + '?mode=ro', uri=True) as db:
            for tid, name, typ in db.execute('select id,name,type from topics'):
                if name not in wanted:
                    continue
                rows = db.execute(
                    'select timestamp,data from messages where topic_id=? '
                    'and timestamp<=? order by timestamp desc limit 30',
                    (tid, int(stamp * 1e9)))
                for t, data in rows:
                    msg = deserialize_message(data, get_message(typ))
                    if name == '/tf':
                        found = [x for x in msg.transforms
                                 if x.header.frame_id == 'map' and x.child_frame_id == 'odom']
                        if not found:
                            continue
                        msg = found[0]
                    if name not in latest or t > latest[name][0]:
                        latest[name] = (t, msg)
                    break
    assert all(n in latest for n in wanted), latest.keys()
    return {n: m for n, (_, m) in latest.items()}


def run_case(config, data, mode):
    config = copy.deepcopy(config)
    follow = config['controller_server']['ros__parameters']['FollowPath']
    follow['short_circuit_trajectory_evaluation'] = False
    if mode == 'baseline':
        follow['plugin'] = 'dwb_core::DWBLocalPlanner'
        follow['sim_time'] = 1.5
    grid = copy.deepcopy(data['/local_costmap/costmap'])
    odom = copy.deepcopy(data['/odom'])
    path = copy.deepcopy(data['/plan'])
    tf = data['/tf'].transform
    angle = 2 * math.atan2(tf.rotation.z, tf.rotation.w)
    for p in path.poses:
        xx = p.pose.position.x - tf.translation.x
        yy = p.pose.position.y - tf.translation.y
        p.pose.position.x = math.cos(angle) * xx + math.sin(angle) * yy
        p.pose.position.y = -math.sin(angle) * xx + math.cos(angle) * yy
        p.header.frame_id = 'odom'
        q = p.pose.orientation
        path_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) - angle
        p.pose.orientation.z = math.sin(path_yaw/2)
        p.pose.orientation.w = math.cos(path_yaw/2)
    path.header.frame_id = 'odom'
    start = path.poses[0].pose.position
    sampled = next(p.pose.position for p in path.poses
                   if math.hypot(p.pose.position.x-start.x, p.pose.position.y-start.y) >= .5)
    desired = math.atan2(sampled.y-odom.pose.pose.position.y,
                         sampled.x-odom.pose.pose.position.x)
    yaw = 2 * math.atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))
    if mode == 'aligned':
        odom.pose.pose.orientation.z = math.sin(desired/2)
        odom.pose.pose.orientation.w = math.cos(desired/2)
    if mode == 'blocked':
        grid.data = [100] * len(grid.data)
    odom.twist.twist = Twist()
    config['local_costmap']['local_costmap']['ros__parameters'] = {
        'global_frame': 'odom', 'robot_base_frame': 'base_footprint',
        'rolling_window': False, 'resolution': .05, 'robot_radius': .22,
        'footprint_padding': .02, 'update_frequency': 10.0,
        'publish_frequency': 1.0, 'transform_tolerance': .5,
        'plugins': ['snapshot'], 'snapshot': {
            'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/offline_grid',
            'map_subscribe_transient_local': True, 'trinary_costmap': False,
            'lethal_cost_threshold': 99, 'footprint_clearing_enabled': False}}
    node = rclpy.create_node('offline_snapshot_' + mode)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_grid = node.create_publisher(OccupancyGrid, '/offline_grid', qos)
    pub_odom = node.create_publisher(Odometry, '/odom', 10)
    broadcaster = TransformBroadcaster(node)
    commands = []
    evaluations = []
    node.create_subscription(LocalPlanEvaluation, '/evaluation',
                             lambda m: evaluations.append(m), 10)
    sub = node.create_subscription(Twist, '/offline_cmd', lambda m: commands.append(
        (m.linear.x, m.angular.z)), 100)

    def feed():
        stamp = node.get_clock().now().to_msg()
        grid.header.stamp = stamp
        grid.header.frame_id = 'odom'
        pub_grid.publish(grid)
        odom.header.stamp = stamp
        pub_odom.publish(odom)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.rotation = odom.pose.pose.orientation
        broadcaster.sendTransform(t)

    timer = node.create_timer(.05, feed)

    def wait(future, seconds=12):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert future.done(), 'isolated service/action timed out'
        return future.result()

    with tempfile.TemporaryDirectory(prefix='offline-controller-') as temp:
        filename = Path(temp) / 'params.yaml'
        filename.write_text(yaml.safe_dump(config))
        with open(Path(temp)/'controller.log', 'w+') as log:
            process = subprocess.Popen([
                '/opt/ros/humble/lib/nav2_controller/controller_server',
                '--ros-args', '--params-file', str(filename),
                '-r', 'cmd_vel:=/offline_cmd'], stdout=log, stderr=log,
                start_new_session=True)
            try:
                client = node.create_client(ChangeState, '/controller_server/change_state')
                assert client.wait_for_service(timeout_sec=10)
                for transition in [1, 3]:
                    req = ChangeState.Request()
                    req.transition.id = transition
                    assert wait(client.call_async(req)).success
                action = ActionClient(node, FollowPath, '/follow_path')
                assert action.wait_for_server(timeout_sec=5)
                path.header.stamp = node.get_clock().now().to_msg()
                goal = FollowPath.Goal()
                goal.path = path
                goal.controller_id = 'FollowPath'
                goal.goal_checker_id = 'general_goal_checker'
                handle = wait(action.send_goal_async(goal))
                assert handle.accepted
                end = time.monotonic()+3
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
                wait(handle.cancel_goal_async())
                assert commands, 'No command output received'
                if evaluations:
                    ev = evaluations[-1]
                    for score in sorted(ev.twists, key=lambda s: s.total)[:5]:
                        print('SCORE', score.traj.velocity, score.total,
                              [(c.name, c.scale, c.raw_score) for c in score.scores])
                    positive = [s for s in ev.twists if s.traj.velocity.x > .05]
                    for score in sorted(positive, key=lambda s: s.total)[:2]:
                        print('FORWARD', score.traj.velocity, score.total,
                              [(c.name, c.scale, c.raw_score) for c in score.scores])
                print(mode, 'heading_error_deg', round(math.degrees(error), 1),
                      'commands', commands[:6], 'max_v', max(v for v,w in commands),
                      'max_abs_w', max(abs(w) for v,w in commands), flush=True)
                assert max(abs(v) for v,w in commands) <= follow['max_vel_x'] + .001
                assert max(abs(w) for v,w in commands) <= follow['max_vel_theta'] + .001
                if mode == 'shim':
                    assert any(w*error > 0 and abs(w) >= .075 for v,w in commands)
                    assert all(abs(v) < 1e-6 for v,w in commands)
                elif mode == 'blocked':
                    assert all(abs(v)+abs(w) < 1e-6 for v,w in commands)
                elif mode == 'aligned':
                    assert any(v > .01 for v,w in commands)
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                log.seek(0)
                print(log.read()[-1500:], flush=True)
                node.destroy_node()


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '177' or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Offline test requires domain 177 and localhost-only')
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='CLOSED bag only')
    parser.add_argument('--stamp', type=float, default=1789962580)
    parser.add_argument('--profile', choices=['mapping', 'navigation'], default='mapping')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    if args.profile == 'navigation':
        config = yaml.safe_load((workspace /
            'src/rk3576_footbath_navigation/config/nav2_params.yaml').read_text())
        data = snapshot(args.bag, args.stamp)
        rclpy.init()
        try:
            for case in ['nav_baseline', 'nav_long', 'nav_shim']:
                trial = copy.deepcopy(config)
                follow = trial['controller_server']['ros__parameters']['FollowPath']
                if case != 'nav_baseline':
                    follow.update(sim_time=3.0, max_vel_theta=.45,
                                  acc_lim_x=.4, decel_lim_x=-.4,
                                  acc_lim_theta=.8, decel_lim_theta=-.8)
                if case == 'nav_shim':
                    follow.update(plugin='nav2_rotation_shim_controller::RotationShimController',
                        primary_controller='dwb_core::DWBLocalPlanner',
                        angular_dist_threshold=.785, forward_sampling_distance=.5,
                        rotate_to_heading_angular_vel=.3, max_angular_accel=.8,
                        simulate_ahead_time=1.5, rotate_to_goal_heading=False)
                run_case(trial, data, case)
        finally:
            rclpy.shutdown()
        print('PASS: isolated saved-map navigation snapshots (not a motion validation)')
        return
    spec = importlib.util.spec_from_file_location('auto_launch', workspace /
        'src/rk3576_footbath_bringup/launch/auto_mapping.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    generated = Path(module.exploration_nav_params(str(workspace /
        'src/rk3576_footbath_navigation/config/nav2_params.yaml')).perform(LaunchContext()))
    try:
        config = yaml.safe_load(generated.read_text())
    finally:
        generated.unlink()
    data = snapshot(args.bag, args.stamp)
    rclpy.init()
    try:
        for mode in ['baseline', 'shim', 'aligned', 'blocked']:
            run_case(config, data, mode)
    finally:
        rclpy.shutdown()
    print('PASS: isolated snapshot alignment, DWB handoff and blocked-map stop')


if __name__ == '__main__':
    main()
