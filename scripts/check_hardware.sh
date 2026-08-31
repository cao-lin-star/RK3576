#!/usr/bin/env bash
set -euo pipefail

echo "== 串口稳定名 =="
for device in /dev/footbath_lidar_high /dev/footbath_lidar_low /dev/footbath_stm32; do
  if [[ -e "$device" || -L "$device" ]]; then
    printf '%s -> %s\n' "$device" "$(readlink -f "$device")"
  else
    printf '%s -> MISSING\n' "$device"
  fi
done

echo "== RK3576 UART3_M0 =="
if [[ -c /dev/ttyS3 ]]; then
  ls -l /dev/ttyS3
  stty -F /dev/ttyS3 2>/dev/null || true
else
  echo "/dev/ttyS3 -> MISSING（检查 uart3-m0 设备树插件）"
fi
if grep -Eq '(^|[[:space:]])console=ttyS3([,[:space:]]|$)' /proc/cmdline; then
  echo "ERROR: ttyS3 is configured as a kernel console"
else
  echo "OK: ttyS3 is not a kernel console"
fi
if systemctl is-active --quiet serial-getty@ttyS3.service; then
  echo "ERROR: serial-getty@ttyS3.service is active"
else
  echo "OK: no active serial getty on ttyS3"
fi

echo "== 当前用户组 =="
id
echo "== ROS 发行版 =="
# ROS setup scripts may reference variables that are not initialized yet.
set +u
source /opt/ros/humble/setup.bash
echo "${ROS_DISTRO:-未设置}"
if [[ -r /home/sky/rk3576_footbath_ws/install/setup.bash ]]; then
  source /home/sky/rk3576_footbath_ws/install/setup.bash
fi
set -u

echo "== 关键包 =="
for package_name in xacro robot_state_publisher slam_toolbox nav2_bringup sllidar_ros2 explore_lite rk3576_footbath_base rk3576_footbath_lidar rk3576_footbath_safety rk3576_footbath_exploration; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    echo "OK   $package_name"
  else
    echo "MISS $package_name"
  fi
done

echo "== 双雷达节点/话题（运行硬件launch后） =="
ros2 pkg executables rk3576_footbath_lidar 2>/dev/null || true
for topic in /scan_high /scan_low_raw /scan_low_front; do
  ros2 topic info "$topic" 2>/dev/null || true
done
