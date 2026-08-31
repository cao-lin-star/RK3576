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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("rk3576_footbath_exploration")
    default_explore_params = os.path.join(
        package_dir, "config", "explore_lite.yaml")
    default_supervisor_params = os.path.join(
        package_dir, "config", "supervisor.yaml")

    supervisor = Node(
        package="rk3576_footbath_exploration",
        executable="exploration_supervisor",
        name="exploration_supervisor",
        output="screen",
        parameters=[
            LaunchConfiguration("supervisor_params"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "map_output_prefix": LaunchConfiguration("map_output_prefix"),
            },
        ],
    )
    explorer = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[
            LaunchConfiguration("explore_params"),
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_explorer")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_explorer", default_value="true"),
        DeclareLaunchArgument(
            "explore_params", default_value=default_explore_params),
        DeclareLaunchArgument(
            "supervisor_params", default_value=default_supervisor_params),
        DeclareLaunchArgument(
            "map_output_prefix",
            default_value=(
                "/home/sky/rk3576_footbath_ws/maps/"
                "footbath_auto_%Y%m%d_%H%M%S"
            ),
        ),
        supervisor,
        # explore_lite starts in its constructor. Starting it after the
        # supervisor gives the pause publisher time to establish discovery;
        # the independent auto-speed limiter remains the hard software bound.
        TimerAction(period=1.0, actions=[explorer]),
    ])
