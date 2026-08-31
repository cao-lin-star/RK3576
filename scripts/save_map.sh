#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "用法: $0 /absolute/path/map_prefix（不要带扩展名）" >&2
  exit 2
fi

map_prefix="$1"
if [[ "$map_prefix" != /* ]]; then
  echo "必须使用绝对路径，避免 systemd/终端工作目录不同导致地图丢失" >&2
  exit 2
fi

mkdir -p "$(dirname "$map_prefix")"
# ROS setup scripts may reference variables that are not initialized yet.
set +u
source /opt/ros/humble/setup.bash
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/install/setup.bash"
set -u
ros2 run nav2_map_server map_saver_cli -f "$map_prefix" --ros-args -p save_map_timeout:=10.0
echo "地图已保存为 ${map_prefix}.yaml/.pgm"
