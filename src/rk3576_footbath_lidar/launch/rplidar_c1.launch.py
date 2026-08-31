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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("node_name", default_value="rplidar_c1_high"),
        DeclareLaunchArgument("serial_port", default_value="/dev/footbath_lidar_high"),
        DeclareLaunchArgument("serial_baudrate", default_value="460800"),
        DeclareLaunchArgument("frame_id", default_value="laser_high_frame"),
        DeclareLaunchArgument("scan_topic", default_value="/scan_high"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("angle_compensate", default_value="true"),
        DeclareLaunchArgument("scan_mode", default_value="Standard"),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name=LaunchConfiguration("node_name"),
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": ParameterValue(
                    LaunchConfiguration("serial_baudrate"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
                "inverted": ParameterValue(LaunchConfiguration("inverted"), value_type=bool),
                "angle_compensate": ParameterValue(
                    LaunchConfiguration("angle_compensate"), value_type=bool),
                "scan_mode": LaunchConfiguration("scan_mode"),
            }],
            remappings=[("scan", LaunchConfiguration("scan_topic"))],
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
