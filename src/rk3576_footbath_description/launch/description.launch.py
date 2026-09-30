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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = [FindPackageShare("rk3576_footbath_description"),
                  "/urdf/rk3576_footbath.urdf.xacro"]
    defaults = {
        "base_radius": "0.22", "base_height": "0.50", "base_link_z": "0.25",
        "laser_high_x": "0.10", "laser_high_y": "0.0", "laser_high_z": "0.50",
        "laser_high_roll": "0.0", "laser_high_pitch": "0.0", "laser_high_yaw": "0.0",
        "laser_low_x": "0.10", "laser_low_y": "0.0", "laser_low_z": "0.15",
        "laser_low_roll": "0.0", "laser_low_pitch": "0.0", "laser_low_yaw": "0.0",
        "imu_x": "0.0", "imu_y": "0.0", "imu_z": "0.195",
        "imu_roll": "0.0", "imu_pitch": "0.0", "imu_yaw": "0.0",
        "tof_left_x": "0.155563", "tof_left_y": "0.155563", "tof_left_z": "0.15",
        "tof_left_pitch": "0.0", "tof_left_yaw": "0.7853981634",
        "tof_right_x": "0.155563", "tof_right_y": "-0.155563",
        "tof_right_z": "0.15", "tof_right_pitch": "0.0",
        "tof_right_yaw": "-0.7853981634",
        "ultrasonic_x": "0.135", "ultrasonic_y": "0.0", "ultrasonic_z": "0.24",
        "ultrasonic_yaw": "0.0",
        "ultrasonic_side_x": "0.084853", "ultrasonic_side_y": "0.084853",
        "ultrasonic_side_z": "0.24", "ultrasonic_side_yaw": "0.7853981634",
    }
    command = ["xacro ", *xacro_file]
    for name in defaults:
        command += [f" {name}:=", LaunchConfiguration(name)]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": ParameterValue(Command(command), value_type=str),
            }],
        ),
    ])
