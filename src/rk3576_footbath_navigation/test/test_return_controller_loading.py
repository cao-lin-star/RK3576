"""Load both real Nav2 profiles without activation, hardware or navigation goals."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import tempfile

import pytest
import yaml


def test_precise_return_controller_loads_without_activation():
    if os.environ.get('ROS_DOMAIN_ID') != '177' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        pytest.skip('Requires isolated localhost-only DDS domain 177')

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist
    from launch import LaunchContext
    from lifecycle_msgs.srv import ChangeState, GetState
    from rcl_interfaces.srv import GetParameters

    source = Path(__file__).resolve().parents[2]
    path = source / 'rk3576_footbath_navigation/launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('return_loading_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    generated = Path(module.NavigationReturnParams(
        source_file=str(source / 'rk3576_footbath_navigation/config/nav2_params.yaml'),
        param_rewrites={}, convert_types=True).perform(LaunchContext()))
    config = yaml.safe_load(generated.read_text())
    executable = Path(get_package_prefix('nav2_controller')) / 'lib/nav2_controller/controller_server'
    rclpy.init()
    node = rclpy.create_node('precise_return_loading_test')
    nonzero = []
    node.create_subscription(Twist, '/offline_return_cmd',
        lambda msg: nonzero.append(msg) if msg.linear.x or msg.angular.z else None, 10)

    def request(service_type, name, request):
        client = node.create_client(service_type, name)
        try:
            assert client.wait_for_service(timeout_sec=12), name
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            assert future.done() and future.exception() is None, name
            return future.result()
        finally:
            node.destroy_client(client)

    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '--params-file', str(generated),
            '-r', 'cmd_vel:=/offline_return_cmd'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            change = ChangeState.Request()
            change.transition.id = 1  # Configure only. NEVER activate or send an action.
            assert request(ChangeState, '/controller_server/change_state', change).success
            state = request(GetState, '/controller_server/get_state', GetState.Request())
            assert state.current_state.id == 2, state.current_state.label  # inactive
            query = GetParameters.Request()
            query.names = ['controller_plugins', 'goal_checker_plugins',
                           'general_goal_checker.xy_goal_tolerance',
                           'general_goal_checker.yaw_goal_tolerance',
                           'FollowPath.xy_goal_tolerance', 'ReturnPath.xy_goal_tolerance',
                           'return_goal_checker.xy_goal_tolerance',
                           'return_goal_checker.yaw_goal_tolerance',
                           'ReturnPath.max_vel_x', 'ReturnPath.max_vel_theta']
            values = request(GetParameters, '/controller_server/get_parameters', query).values
            assert list(values[0].string_array_value) == ['FollowPath', 'ExplorePath', 'ReturnPath']
            assert list(values[1].string_array_value) == ['general_goal_checker', 'exploration_goal_checker', 'return_goal_checker']
            assert [p.double_value for p in values[2:]] == [.15, .20, .15, .03, .03, 3.142, .20, .80]
            assert not nonzero
            assert process.poll() is None
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.seek(0)
            print(log.read())
            node.destroy_node()
            rclpy.shutdown()
            navigator = config['bt_navigator']['ros__parameters']
            for key in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
                Path(navigator[key]).unlink()
            generated.unlink()
