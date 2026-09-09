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
        DeclareLaunchArgument("slam_backend", default_value="slam_toolbox"),
        DeclareLaunchArgument(
            "mapping_scan_source",
            default_value="fused",
            choices=["high", "fused"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "hardware.launch.py")),
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
                "slam_backend": LaunchConfiguration("slam_backend"),
                "scan_topic": mapping_scan_topic,
                "use_rviz": show_rviz,
            }.items(),
        ),
    ])
