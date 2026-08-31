#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts may reference variables that are not initialized yet.
set +u
source /opt/ros/humble/setup.bash
source /home/sky/rk3576_footbath_ws/install/setup.bash
set -u

mode="${FOOTBATH_MODE:-hardware}"
case "$mode" in
  hardware)
    exec ros2 launch rk3576_footbath_bringup hardware.launch.py \
      use_sim_time:=false
    ;;
  mapping)
    exec ros2 launch rk3576_footbath_bringup mapping.launch.py \
      headless:=true slam_backend:="${FOOTBATH_SLAM_BACKEND:-slam_toolbox}"
    ;;
  auto_mapping)
    exec ros2 launch rk3576_footbath_bringup auto_mapping.launch.py \
      headless:=true start_explorer:=true \
      map_output_prefix:=/home/sky/rk3576_footbath_ws/maps/footbath_auto_%Y%m%d_%H%M%S
    ;;
  navigation)
    : "${FOOTBATH_MAP:?FOOTBATH_MAP 必须指向绝对地图 yaml}"
    if [[ ! -f "$FOOTBATH_MAP" ]]; then
      echo "地图不存在: $FOOTBATH_MAP" >&2
      exit 3
    fi
    exec ros2 launch rk3576_footbath_bringup navigation.launch.py \
      headless:=true map:="$FOOTBATH_MAP"
    ;;
  *)
    echo "不支持的 FOOTBATH_MODE=$mode；可选 hardware、mapping、auto_mapping、navigation" >&2
    exit 2
    ;;
esac
