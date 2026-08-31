#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "请使用 sudo $0" >&2
  exit 2
fi

apt-get -o Acquire::ForceIPv4=true update
apt-get -o Acquire::ForceIPv4=true install -y \
  ros-humble-ros-base \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-diagnostic-msgs \
  ros-humble-slam-toolbox \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-foxglove-bridge \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-serial \
  openocd \
  tmux \
  build-essential

rosdep init 2>/dev/null || true
rosdep update
