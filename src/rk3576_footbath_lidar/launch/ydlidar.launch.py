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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = [FindPackageShare("rk3576_footbath_lidar"),
                      "/config/default_ydlidar_model12.yaml"]
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("driver_package", default_value="ydlidar"),
        DeclareLaunchArgument("driver_executable", default_value="ydlidar_node"),
        Node(
            package=LaunchConfiguration("driver_package"),
            executable=LaunchConfiguration("driver_executable"),
            name="ydlidar_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
