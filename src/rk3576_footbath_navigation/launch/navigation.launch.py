# Copyright 2026 sky
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy
import os
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def explicit_navigation_tree(source, controller_ids, checker_ids):
    """Keep the installed/custom BT policy, but remove ambiguous empty IDs.

    Humble only accepts an empty FollowPath controller/checker ID when exactly
    one plugin exists. A second return-only plugin must not break ordinary
    NavigateToPose or NavigateThroughPoses. Explicit IDs in custom BTs are kept;
    unprovable dynamic selection is rejected instead of silently choosing one.
    """
    tree = ET.parse(source)
    followers = list(tree.iter('FollowPath'))
    if not followers:
        raise ValueError('navigation BT must contain an explicit FollowPath: ' + str(source))
    for follower in followers:
        for attribute, default, available in (
                ('controller_id', 'FollowPath', controller_ids),
                ('goal_checker_id', 'general_goal_checker', checker_ids)):
            selected = follower.get(attribute) or default
            if selected not in available:
                raise ValueError(f'unsupported {attribute}={selected} in navigation BT {source}; '
                                 'use an explicitly configured plugin ID')
            follower.set(attribute, selected)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False,
                                     encoding='utf-8') as stream:
        tree.write(stream, encoding='unicode')
        return stream.name


class NavigationReturnParams(RewrittenYaml):
    """Separate precise staging from ordinary navigation in every nav session."""

    def perform(self, context):
        filename = super().perform(context)
        generated = [filename]
        try:
            return self._configure(filename, generated)
        except Exception:
            # Invalid custom BTs fail before launching Nav2. Do not accumulate
            # half-generated YAML/XML files over repeated failed launches.
            for path in generated:
                try:
                    os.unlink(path)
                except OSError:
                    pass
            raise

    def _configure(self, filename, generated):
        with open(filename, encoding='utf-8') as stream:
            params = yaml.safe_load(stream)
        control = params['controller_server']['ros__parameters']
        if ('ReturnPath' in control or 'return_goal_checker' in control
                or 'ReturnPath' in control['controller_plugins']
                or 'return_goal_checker' in control['goal_checker_plugins']):
            raise ValueError('ReturnPath/return_goal_checker are reserved for precise dock staging')
        # The DWB RotateToGoal critic has its own xy tolerance. Tightening only
        # the checker would stop translation at the old 15 cm threshold forever.
        control['ReturnPath'] = copy.deepcopy(control['FollowPath'])
        control['ReturnPath']['xy_goal_tolerance'] = 0.03
        control['controller_plugins'].append('ReturnPath')
        control['return_goal_checker'] = copy.deepcopy(control['general_goal_checker'])
        control['return_goal_checker'].update(xy_goal_tolerance=0.03, yaw_goal_tolerance=3.142)
        control['goal_checker_plugins'].append('return_goal_checker')
        navigator = params['bt_navigator']['ros__parameters']
        stock = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
        for parameter, basename in (
                ('default_nav_to_pose_bt_xml', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                ('default_nav_through_poses_bt_xml', 'navigate_through_poses_w_replanning_and_recovery.xml')):
            # Honor a caller-supplied tree rather than replacing its recovery policy.
            source = navigator.get(parameter) or os.path.join(stock, basename)
            navigator[parameter] = explicit_navigation_tree(
                source, control['controller_plugins'], control['goal_checker_plugins'])
            generated.append(navigator[parameter])
        with open(filename, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(params, stream)
        return filename


def navigation_stack(context):
    # Ordinary navigation can later receive return_home without a relaunch.
    # Both entry paths therefore load the same two isolated plugin profiles.
    # dock_return remains a compatible launch argument, not a global override.
    params = NavigationReturnParams(source_file=LaunchConfiguration('params_file'),
                                    param_rewrites={}, convert_types=True)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py']),
        launch_arguments={
            'map': LaunchConfiguration('map'), 'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params, 'autostart': LaunchConfiguration('autostart'),
            'slam': 'False', 'use_composition': 'False',
        }.items())]


def generate_launch_description():
    nav2_dir = FindPackageShare("nav2_bringup")
    package_dir = FindPackageShare("rk3576_footbath_navigation")
    default_map = [package_dir, "/maps/legacy_fishbot_map.yaml"]
    default_params = [package_dir, "/config/nav2_params.yaml"]
    rviz_config = [package_dir, "/rviz/navigation.rviz"]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("dock_return", default_value="false"),
        OpaqueFunction(function=navigation_stack),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
