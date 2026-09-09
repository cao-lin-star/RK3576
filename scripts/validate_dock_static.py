#!/usr/bin/env python3
"""Isolated ROS startup + launch-parameter validation; never starts hardware."""
import importlib.util
import os
from pathlib import Path
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import rclpy
from std_msgs.msg import Bool
from rk3576_footbath_exploration.supervisor import ExplorationSupervisor


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '91'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    share=Path(get_package_share_directory('rk3576_footbath_navigation'))
    spec=importlib.util.spec_from_file_location('dock_nav_launch',share/'launch/navigation.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    source=share/'config/nav2_params.yaml'
    for dock,expected in [('false',.15),('true',.03)]:
        ctx=LaunchContext(); ctx.launch_configurations.update(
            params_file=str(source),dock_return=dock)
        include=module.navigation_stack(ctx)[0]
        value=dict(include.launch_arguments)['params_file']
        resolved=value.perform(ctx)
        config=yaml.safe_load(Path(resolved).read_text())
        actual=config['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']
        assert actual==expected,(dock,actual)
        yaw=config['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance']
        assert yaw==(3.142 if dock=='true' else .20)
        print('navigation dock_return='+dock+' tolerance='+str(actual))
    rclpy.init(args=['--ros-args','-p','auto_start:=true','-p','dock.auto_exit:=true'])
    node=ExplorationSupervisor()
    leases=[]
    node.create_subscription(Bool,'/safety/auto_motion_lease',lambda m:leases.append(m.data),10)
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    assert node.home.dock.distance==.5
    assert leases and not any(leases)
    assert not node.home.dock.active and node.home.pose is None
    print('missing inputs: no departure, lease false; distance=0.5m')
    node.emergency_stop('isolated dock static check done')
    node.destroy_node(); rclpy.shutdown()
    for required in ('true','false'):
        rclpy.init(args=['--ros-args','-p','return_home.localization_mode:=true',
            '-p','dock.navigation_session:=true','-p','dock.departure_required:='+required])
        node=ExplorationSupervisor()
        for _ in range(10):
            rclpy.spin_once(node,timeout_sec=.1)
        assert node.home.pose is None and not node._motion_allowed()
        assert not node.home.dock.exit_complete
        print('navigation startup without input: lease false, depart_required='+required)
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
