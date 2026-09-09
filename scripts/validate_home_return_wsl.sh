#!/usr/bin/env bash
# Isolated host-side checks; never launches hardware or connects to the robot.
set -eo pipefail
task_source=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
task_build=$(mktemp -d /tmp/footbath-home-check-XXXXXX)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
colcon --log-base "$task_build/log" build \
  --base-paths "$task_source/src/m_explore_ros2" \
  "$task_source/src/rk3576_footbath_exploration" \
  "$task_source/src/rk3576_footbath_mobile" \
  "$task_source/src/rk3576_footbath_bringup" \
  "$task_source/src/rk3576_footbath_navigation" \
  "$task_source/src/rk3576_footbath_costmap" \
  --build-base "$task_build/build" --install-base "$task_build/install" \
  --packages-select explore_lite_msgs explore_lite rk3576_footbath_exploration \
  rk3576_footbath_mobile rk3576_footbath_bringup rk3576_footbath_navigation rk3576_footbath_costmap --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
source "$task_build/install/setup.bash"
export PYTHONPATH="$task_source/src/rk3576_footbath_exploration:$task_source/src/rk3576_footbath_mobile:$PYTHONPATH"
/usr/bin/python3 -m pytest -q \
  "$task_source/src/rk3576_footbath_exploration/test/test_home_return.py" \
  "$task_source/src/rk3576_footbath_exploration/test/test_dock_motion.py" \
  "$task_source/src/rk3576_footbath_exploration/test/test_navigation_readiness.py" \
  "$task_source/src/rk3576_footbath_exploration/test/test_health.py" \
  "$task_source/src/rk3576_footbath_mobile/test"
/usr/bin/python3 - <<'PY'
import rclpy
from rk3576_footbath_exploration.supervisor import ExplorationSupervisor
rclpy.init(args=['--ros-args', '-p', 'auto_start:=false'])
node = ExplorationSupervisor()
for _ in range(15):
    rclpy.spin_once(node, timeout_sec=0.1)
assert node.home.pose is None
assert not node._motion_allowed()
node.emergency_stop('isolated startup smoke test complete')
node.destroy_node()
rclpy.shutdown()
print('Isolated supervisor startup: OK; no motion lease')
PY
