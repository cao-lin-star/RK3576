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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def navigation_stack(context):
    params = LaunchConfiguration('params_file')
    if LaunchConfiguration('dock_return').perform(context).lower() == 'true':
        params = RewrittenYaml(source_file=params, param_rewrites={
            'xy_goal_tolerance': '0.03', 'yaw_goal_tolerance': '3.142'}, convert_types=True)
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
