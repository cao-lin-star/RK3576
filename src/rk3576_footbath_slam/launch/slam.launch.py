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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def backend_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration("slam_backend"), "' == '", name, "'"
    ]))


def generate_launch_description():
    slam_params = [FindPackageShare("rk3576_footbath_slam"),
                   "/config/slam_toolbox_mapping.yaml"]
    cartographer_config = [FindPackageShare("rk3576_footbath_slam"), "/config"]
    rviz_config = [FindPackageShare("rk3576_footbath_slam"), "/rviz/mapping.rviz"]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_backend", default_value="slam_toolbox",
                              description="slam_toolbox 或 cartographer"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[LaunchConfiguration("slam_params_file"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=backend_is("slam_toolbox"),
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=["-configuration_directory", cartographer_config,
                       "-configuration_basename", "footbath_cartographer_2d.lua"],
            remappings=[("scan", "/scan_high")],
            condition=backend_is("cartographer"),
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
            condition=backend_is("cartographer"),
        ),
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
