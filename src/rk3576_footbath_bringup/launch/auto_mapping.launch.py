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


def generate_launch_description():
    bringup_dir = get_package_share_directory("rk3576_footbath_bringup")
    slam_dir = get_package_share_directory("rk3576_footbath_slam")
    navigation_dir = get_package_share_directory(
        "rk3576_footbath_navigation")
    safety_dir = get_package_share_directory("rk3576_footbath_safety")
    exploration_dir = get_package_share_directory(
        "rk3576_footbath_exploration")
    nav2_dir = get_package_share_directory("nav2_bringup")

    nav2_params = os.path.join(
        navigation_dir, "config", "nav2_params.yaml")
    show_rviz = PythonExpression([
        "'", LaunchConfiguration("headless"), "'.lower() == 'false'"
    ])
    mapping_scan_topic = PythonExpression([
        "'/scan_mapping_fused' if '",
        LaunchConfiguration("mapping_scan_source"),
        "' == 'fused' else '/scan_high'"
    ])
    fusion_enabled = PythonExpression([
        "'", LaunchConfiguration("mapping_scan_source"), "' == 'fused'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("start_explorer", default_value="true"),
        DeclareLaunchArgument("auto_start_exploration", default_value="true"),
        DeclareLaunchArgument(
            "mapping_scan_source",
            default_value="fused",
            choices=["high", "fused"],
        ),
        DeclareLaunchArgument(
            "map_output_prefix",
            default_value=(
                "/home/sky/rk3576_footbath_ws/maps/"
                "footbath_auto_%Y%m%d_%H%M%S"
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_dir, "launch", "hardware.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_mapping_fusion": fusion_enabled,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, "launch", "slam.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam_backend": "slam_toolbox",
                "scan_topic": mapping_scan_topic,
                "use_rviz": show_rviz,
            }.items(),
        ),
        # Use Nav2's navigation-only launch: slam_toolbox already owns
        # map->odom and /map, so AMCL and map_server must not be started.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_dir, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": nav2_params,
                "autostart": "true",
                # Humble expects Python boolean literals for these values.
                "use_composition": "False",
                "use_respawn": "True",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_dir, "launch", "safety.launch.py")),
            launch_arguments={
                "start_auto_limiter": "true",
                "auto_limiter_require_lease": "true",
                "start_command_mux": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    exploration_dir, "launch", "exploration.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_explorer": LaunchConfiguration("start_explorer"),
                "auto_start_exploration": LaunchConfiguration("auto_start_exploration"),
                "map_output_prefix": LaunchConfiguration(
                    "map_output_prefix"),
            }.items(),
        ),
    ])
