#!/usr/bin/env bash
# Passive recording only: no commands, service calls, or driver launches.
# Run under a time-limited service; stopping this recorder does NOT stop the robot.
# Navigation/return diagnosis needs every velocity stage and per-goal context:
# controller -> smoother -> safety limiter -> mux -> measured odometry.
# Unknown/unpublished topics are discovered if their publishers appear later.
# Wait until rosbag exits before inspecting any SQLite file or replaying a bag.
set -eo pipefail
workspace="${FOOTBATH_TRIAL_WS:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
output="${1:?Usage: record_mapping_trial.sh /absolute/new/bag/path}"
[[ "$output" = /* && ! -e "$output" ]] || { echo 'Output must be a new absolute path' >&2; exit 2; }
parent="$(dirname -- "$output")"
[[ -d "$parent" ]] || { echo 'Output parent must already exist' >&2; exit 2; }
available_kb="$(df -Pk -- "$parent" | awk 'NR==2 {print $4}')"
(( available_kb >= 5242880 )) || { echo 'At least 5 GiB free space required' >&2; exit 2; }
exec ros2 bag record --include-hidden-topics --max-cache-size 16777216 \
  --max-bag-size 104857600 -o "$output" \
  /scan_high /scan_low_front /scan_mapping_fused /odom /imu/data_raw \
  /tf /tf_static /map /amcl_pose /initialpose /plan /local_plan \
  /scan_low_raw /plan_smoothed /received_global_plan /transformed_global_plan \
  /particle_cloud /goal_pose /speed_limit \
  /local_costmap/costmap /global_costmap/costmap \
  /local_costmap/costmap_raw /global_costmap/costmap_raw \
  /local_costmap/costmap_updates /global_costmap/costmap_updates \
  /local_costmap/published_footprint /global_costmap/published_footprint \
  /cmd_vel_nav /cmd_vel /cmd_vel_auto_limited /cmd_vel_selected \
  /cmd_vel_manual /cmd_vel_recovery \
  /chassis/control_source /chassis/side_ultrasonic_enabled /ultrasonic/status /diagnostics /rosout \
  /range/ultrasonic /range/ultrasonic_front_observe \
  /range/ultrasonic_left /range/ultrasonic_right /range/tof_left /range/tof_right \
  /safety/suspected_glass /safety/suspected_glass_valid /range/suspected_glass \
  /safety/suspected_glass_left /safety/suspected_glass_right \
  /range/suspected_glass_left /range/suspected_glass_right \
  /safety/auto_motion_lease /safety/recovery_active /safety/hazard_zones \
  /local_costmap/hazard_zones_applied /global_costmap/hazard_zones_applied \
  /explore/status /explore/frontiers /mapping/home_status \
  /mobile/navigation_context /mobile/navigation_recovery /behavior_tree_log \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status \
  /follow_path/_action/feedback /follow_path/_action/status \
  /compute_path_to_pose/_action/status /spin/_action/status \
  /backup/_action/status /wait/_action/status /parameter_events
