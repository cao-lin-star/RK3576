// Copyright 2026 sky
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef RK3576_FOOTBATH_LIDAR__LOW_LIDAR_ANGULAR_FILTER_HPP_
#define RK3576_FOOTBATH_LIDAR__LOW_LIDAR_ANGULAR_FILTER_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"

namespace rk3576_footbath_lidar
{

sensor_msgs::msg::LaserScan filter_scan_by_angle(
  const sensor_msgs::msg::LaserScan & input,
  double valid_angle_min,
  double valid_angle_max,
  double sensor_x_m = 0.0,
  double sensor_y_m = 0.0,
  double sensor_yaw_rad = 0.0,
  double self_filter_radius_m = 0.0);

}  // namespace rk3576_footbath_lidar

#endif  // RK3576_FOOTBATH_LIDAR__LOW_LIDAR_ANGULAR_FILTER_HPP_
