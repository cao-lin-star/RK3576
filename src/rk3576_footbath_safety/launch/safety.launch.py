import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("rk3576_footbath_safety")
    params = os.path.join(package_dir, "config", "safety.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("start_auto_limiter", default_value="false"),
        DeclareLaunchArgument(
            "auto_limiter_require_lease", default_value="true"),
        DeclareLaunchArgument("start_command_mux", default_value="false"),
        Node(
            package="rk3576_footbath_safety",
            executable="auto_cmd_vel_limiter",
            name="auto_cmd_vel_limiter",
            output="screen",
            parameters=[
                params,
                {
                    "require_lease": ParameterValue(
                        LaunchConfiguration("auto_limiter_require_lease"),
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("start_auto_limiter")),
        ),
        Node(
            package="rk3576_footbath_safety",
            executable="footbath_command_mux",
            name="footbath_command_mux",
            output="screen",
            parameters=[params],
            condition=IfCondition(
                LaunchConfiguration("start_command_mux")),
        ),
    ])
