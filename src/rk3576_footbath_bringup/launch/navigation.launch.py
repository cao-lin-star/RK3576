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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("rk3576_footbath_bringup")
    navigation_dir = get_package_share_directory("rk3576_footbath_navigation")
    safety_dir = get_package_share_directory("rk3576_footbath_safety")
    default_map = os.path.join(navigation_dir, "maps", "legacy_fishbot_map.yaml")
    show_rviz = PythonExpression([
        "'", LaunchConfiguration("headless"), "'.lower() == 'false'"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("return_session", default_value="false"),
        DeclareLaunchArgument("depart_from_dock", default_value="true"),
        DeclareLaunchArgument("home_pose_json", default_value="{}"),
        Node(
            package="rk3576_footbath_exploration", executable="exploration_supervisor",
            name="exploration_supervisor", output="screen",
            parameters=[os.path.join(get_package_share_directory("rk3576_footbath_exploration"),
                                     "config", "supervisor.yaml"),
                        {"auto_start": False, "return_home.localization_mode": True,
                         "hazard_recovery.map_file": LaunchConfiguration("map"),
                         "dock.departure_required": ParameterValue(
                             LaunchConfiguration("depart_from_dock"), value_type=bool),
                         "dock.navigation_session": ParameterValue(PythonExpression([
                             "'", LaunchConfiguration("return_session"), "'.lower() != 'true'"
                         ]), value_type=bool),
                         "return_home.pose_json": ParameterValue(
                             LaunchConfiguration("home_pose_json"), value_type=str)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "hardware.launch.py")),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, "launch", "navigation.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map": LaunchConfiguration("map"),
                "use_rviz": show_rviz,
                "dock_return": LaunchConfiguration("return_session"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_dir, "launch", "safety.launch.py")),
            launch_arguments={
                "start_glass_monitor": "false",
                "start_auto_limiter": "true",
                "auto_limiter_require_lease": "true",
                "start_command_mux": "false",
            }.items(),
        ),
    ])
