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
#ifndef RK3576_FOOTBATH_LIDAR__DUAL_LIDAR_FUSION_HPP_
#define RK3576_FOOTBATH_LIDAR__DUAL_LIDAR_FUSION_HPP_

#include <cstddef>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace rk3576_footbath_lidar
{

struct PlanarTransform
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct FusionStatistics
{
  std::size_t low_finite_points{0U};
  std::size_t projected_points{0U};
  std::size_t bins_updated{0U};
};

sensor_msgs::msg::LaserScan fuse_laser_scans(
  const sensor_msgs::msg::LaserScan & high_scan,
  const sensor_msgs::msg::LaserScan & filtered_low_scan,
  const PlanarTransform & low_to_high,
  FusionStatistics * statistics = nullptr);

}  // namespace rk3576_footbath_lidar

#endif  // RK3576_FOOTBATH_LIDAR__DUAL_LIDAR_FUSION_HPP_
