#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS setup scripts may reference variables that are not initialized yet.
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$workspace_dir"
# 厂商 YDLIDAR 驱动并非 Ubuntu/ROS 仓库中的统一 rosdep key；它在板端单独安装，
# 这里明确跳过该 key，而不是为了构建通过删除 rk3576_footbath_lidar 核心功能。
rosdep install --from-paths src --ignore-src -r -y --skip-keys ydlidar
colcon build --symlink-install --event-handlers console_cohesion+
