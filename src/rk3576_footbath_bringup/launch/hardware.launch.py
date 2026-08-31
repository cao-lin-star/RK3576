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
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def both_enabled(global_name, unit_name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration(global_name), "'.lower() == 'true' and '",
        LaunchConfiguration(unit_name), "'.lower() == 'true'",
    ]))


def generate_launch_description():
    bringup_dir = get_package_share_directory("rk3576_footbath_bringup")
    description_dir = get_package_share_directory("rk3576_footbath_description")
    base_dir = get_package_share_directory("rk3576_footbath_base")
    lidar_dir = get_package_share_directory("rk3576_footbath_lidar")
    safety_dir = get_package_share_directory("rk3576_footbath_safety")
    with open(os.path.join(bringup_dir, "config", "hardware.yaml"), encoding="utf-8") as stream:
        hardware = yaml.safe_load(stream)
    geometry = hardware["robot_geometry"]
    stm32 = hardware["stm32"]
    lidar_high = hardware["rplidar_high"]
    lidar_low = hardware["rplidar_low"]

    geometry_args = {
        "base_radius": str(geometry["base_radius_m"]),
        "base_height": str(geometry["base_height_m"]),
        "base_link_z": str(geometry["base_link_z_m"]),
        "laser_high_x": str(geometry["laser_high_x_m"]),
        "laser_high_y": str(geometry["laser_high_y_m"]),
        "laser_high_z": str(geometry["laser_high_z_m"]),
        "laser_high_roll": str(geometry["laser_high_roll_rad"]),
        "laser_high_pitch": str(geometry["laser_high_pitch_rad"]),
        "laser_high_yaw": str(geometry["laser_high_yaw_rad"]),
        "laser_low_x": str(geometry["laser_low_x_m"]),
        "laser_low_y": str(geometry["laser_low_y_m"]),
        "laser_low_z": str(geometry["laser_low_z_m"]),
        "laser_low_roll": str(geometry["laser_low_roll_rad"]),
        "laser_low_pitch": str(geometry["laser_low_pitch_rad"]),
        "laser_low_yaw": str(geometry["laser_low_yaw_rad"]),
        "imu_x": str(geometry["imu_x_m"]),
        "imu_y": str(geometry["imu_y_m"]),
        "imu_z": str(geometry["imu_z_m"]),
        "imu_roll": str(geometry["imu_roll_rad"]),
        "imu_pitch": str(geometry["imu_pitch_rad"]),
        "imu_yaw": str(geometry["imu_yaw_rad"]),
        "tof_left_x": str(geometry["tof_left_x_m"]),
        "tof_left_y": str(geometry["tof_left_y_m"]),
        "tof_left_z": str(geometry["tof_left_z_m"]),
        "tof_left_pitch": str(geometry["tof_left_pitch_rad"]),
        "tof_left_yaw": str(geometry["tof_left_yaw_rad"]),
        "tof_right_x": str(geometry["tof_right_x_m"]),
        "tof_right_y": str(geometry["tof_right_y_m"]),
        "tof_right_z": str(geometry["tof_right_z_m"]),
        "tof_right_pitch": str(geometry["tof_right_pitch_rad"]),
        "tof_right_yaw": str(geometry["tof_right_yaw_rad"]),
        "ultrasonic_x": str(geometry["ultrasonic_x_m"]),
        "ultrasonic_y": str(geometry["ultrasonic_y_m"]),
        "ultrasonic_z": str(geometry["ultrasonic_z_m"]),
        "ultrasonic_yaw": str(geometry["ultrasonic_yaw_rad"]),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    rplidar_launch = os.path.join(lidar_dir, "launch", "rplidar_c1.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_lidar_high", default_value="true"),
        DeclareLaunchArgument("start_lidar_low", default_value="true"),
        DeclareLaunchArgument("stm32_device", default_value=stm32["device"]),
        DeclareLaunchArgument("stm32_baudrate", default_value=str(stm32["baudrate"])),
        DeclareLaunchArgument("lidar_high_device", default_value=lidar_high["device"]),
        DeclareLaunchArgument("lidar_high_baudrate", default_value=str(lidar_high["baudrate"])),
        DeclareLaunchArgument("lidar_low_device", default_value=lidar_low["device"]),
        DeclareLaunchArgument("lidar_low_baudrate", default_value=str(lidar_low["baudrate"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_dir, "launch", "description.launch.py")),
            launch_arguments=geometry_args.items(),
        ),
        Node(
            package="rk3576_footbath_base",
            executable="stm32_serial_bridge",
            name="stm32_serial_bridge",
            output="screen",
            parameters=[
                os.path.join(base_dir, "config", "serial_bridge.yaml"),
                {
                    "serial.port": LaunchConfiguration("stm32_device"),
                    "serial.baudrate": ParameterValue(
                        LaunchConfiguration("stm32_baudrate"), value_type=int),
                },
            ],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration("start_base")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                "node_name": "rplidar_c1_high",
                "serial_port": LaunchConfiguration("lidar_high_device"),
                "serial_baudrate": LaunchConfiguration("lidar_high_baudrate"),
                "frame_id": lidar_high["frame_id"],
                "scan_topic": lidar_high["scan_topic"],
                "scan_mode": lidar_high["scan_mode"],
            }.items(),
            condition=both_enabled("start_lidar", "start_lidar_high"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                "node_name": "rplidar_c1_low",
                "serial_port": LaunchConfiguration("lidar_low_device"),
                "serial_baudrate": LaunchConfiguration("lidar_low_baudrate"),
                "frame_id": lidar_low["frame_id"],
                "scan_topic": lidar_low["raw_scan_topic"],
                "scan_mode": lidar_low["scan_mode"],
            }.items(),
            condition=both_enabled("start_lidar", "start_lidar_low"),
        ),
        Node(
            package="rk3576_footbath_lidar",
            executable="low_lidar_angular_filter",
            name="low_lidar_angular_filter",
            output="screen",
            parameters=[{
                "input_topic": lidar_low["raw_scan_topic"],
                "output_topic": lidar_low["filtered_scan_topic"],
                "valid_angle_min": lidar_low["valid_angle_min_rad"],
                "valid_angle_max": lidar_low["valid_angle_max_rad"],
                "sensor_x_m": geometry["laser_low_x_m"],
                "sensor_y_m": geometry["laser_low_y_m"],
                "sensor_yaw_rad": geometry["laser_low_yaw_rad"],
                "self_filter_radius_m": (
                    geometry["base_radius_m"]
                    + lidar_low.get("self_filter_margin_m", 0.005)
                ),
            }],
            respawn=True,
            respawn_delay=2.0,
            condition=both_enabled("start_lidar", "start_lidar_low"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(safety_dir, "launch", "safety.launch.py")),
            launch_arguments={
                "start_glass_monitor": "true",
                "start_auto_limiter": "false",
                "start_command_mux": "true",
            }.items(),
        ),
    ])
