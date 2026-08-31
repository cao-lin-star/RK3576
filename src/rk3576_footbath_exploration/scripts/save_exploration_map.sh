#!/usr/bin/env bash
set -euo pipefail

# std_srvs is provided by the base ROS installation. The running supervisor
# owns the configured destination and the asynchronous save sequence.
# ROS setup scripts may reference variables that are not initialized yet.
set +u
source /opt/ros/humble/setup.bash
set -u
ros2 service call /exploration/save_map std_srvs/srv/Trigger '{}'
