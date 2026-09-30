"""Isolated Nav2 costmaps -> HazardRecovery ACK handshake; never send a goal.

Source Humble and the isolated costmap install, add the source exploration
package to PYTHONPATH, and set ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1.
"""
import argparse
import copy
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from rk3576_footbath_exploration.hazard_recovery import HazardRecovery


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '177' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Refusing to run outside isolated localhost DDS domain 177')
    parser = argparse.ArgumentParser()
    parser.add_argument('--params', required=True)
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.params).read_text())
    layer_config = dict(global_frame='map', robot_base_frame='base_footprint',
                        rolling_window=False, width=5, height=5, origin_x=-2.5,
                        origin_y=-2.5, resolution=.05, update_frequency=10.,
                        publish_frequency=5., always_send_full_costmap=True, robot_radius=.22,
                        plugins=['hazard_layer', 'inflation_layer'],
                        hazard_layer={'plugin': 'rk3576_footbath_costmap::HazardLayer'},
                        inflation_layer={'plugin': 'nav2_costmap_2d::InflationLayer',
                                         'inflation_radius': .4})
    for name in ('local_costmap', 'global_costmap'):
        config[name][name]['ros__parameters'] = copy.deepcopy(layer_config)
    rclpy.init()
    node = rclpy.create_node('hazard_ack_handshake_test')
    recovery = HazardRecovery.__new__(HazardRecovery)
    recovery.applied = {}
    recovery.marked_stamp = node.get_clock().now().nanoseconds
    recovery.marked_at = time.monotonic()
    grids = {}
    nonzero_commands = []
    for name in ('local_costmap', 'global_costmap'):
        node.create_subscription(Header, '/' + name + '/hazard_zones_applied',
                                 lambda msg, key=name: recovery._applied(key, msg), 10)
        node.create_subscription(OccupancyGrid, '/' + name + '/costmap',
                                 lambda msg, key=name: grids.__setitem__(key, msg), 10)
    node.create_subscription(Twist, '/offline_cmd', lambda msg: nonzero_commands.append(msg)
                             if msg.linear.x or msg.angular.z else None, 10)
    publisher = node.create_publisher(PoseArray, '/safety/hazard_zones',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster = TransformBroadcaster(node)

    def feed_tf():
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)

    node.create_timer(.02, feed_tf)

    def until(predicate, seconds=15.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        raise AssertionError('Isolated ACK test timed out')

    with tempfile.TemporaryDirectory(prefix='hazard-handshake-') as temp:
        parameters = Path(temp) / 'params.yaml'
        parameters.write_text(yaml.safe_dump(config))
        processes, logs = [], []
        try:
            for name, package in [('controller_server', 'nav2_controller'),
                                  ('planner_server', 'nav2_planner')]:
                log = open(Path(temp) / (name + '.log'), 'w+')
                logs.append(log)
                processes.append(subprocess.Popen([
                    '/opt/ros/humble/lib/' + package + '/' + name,
                    '--ros-args', '--params-file', str(parameters),
                    '-r', 'cmd_vel:=/offline_cmd'], stdout=log, stderr=log,
                    start_new_session=True))
                client = node.create_client(ChangeState, '/' + name + '/change_state')
                assert client.wait_for_service(timeout_sec=12.), name + ' unavailable'
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = client.call_async(request)
                    until(future.done)
                    assert future.result().success, name + ' transition failed'
            names = node.get_node_names_and_namespaces()
            for name in ('local_costmap', 'global_costmap'):
                assert (name, '/' + name) in names, names
            assert not recovery.zones_applied(), 'No hazard revision has been sent yet'
            zones = PoseArray()
            zones.header.frame_id = 'map'
            zones.header.stamp = node.get_clock().now().to_msg()
            recovery.marked_stamp = (zones.header.stamp.sec * 1000000000
                                     + zones.header.stamp.nanosec)
            recovery.marked_at = time.monotonic()
            point = Pose()
            point.position.x = .4
            point.position.y = .1
            point.position.z = .15
            zones.poses.append(point)
            publisher.publish(zones)
            until(recovery.zones_applied)

            def both_marked():
                for name in ('local_costmap', 'global_costmap'):
                    if name not in grids:
                        return False
                    grid = grids[name]
                    x = int((.4 - grid.info.origin.position.x) / grid.info.resolution)
                    y = int((.1 - grid.info.origin.position.y) / grid.info.resolution)
                    if grid.data[y * grid.info.width + x] != 100:
                        return False
                return True

            until(both_marked)
            assert not nonzero_commands, 'Unexpected nonzero isolated command'
            print('PASS: actual controller/planner costmap namespaces have no duplicated suffix')
            print('PASS: both Header ACKs satisfy current HazardRecovery.zones_applied()')
            print('PASS: both costmaps contain lethal hazard geometry; no goal or nonzero command')
            print('ACK_TOPICS=/local_costmap/hazard_zones_applied /global_costmap/hazard_zones_applied')
        except BaseException:
            for log in logs:
                log.flush(); log.seek(0)
                print(log.read()[-6000:])
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.)
            for log in logs:
                log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
