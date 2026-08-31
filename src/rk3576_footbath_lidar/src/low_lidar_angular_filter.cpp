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
#include "rk3576_footbath_lidar/low_lidar_angular_filter.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace rk3576_footbath_lidar
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

sensor_msgs::msg::LaserScan filter_scan_by_angle(
  const sensor_msgs::msg::LaserScan & input,
  const double valid_angle_min,
  const double valid_angle_max,
  const double sensor_x_m,
  const double sensor_y_m,
  const double sensor_yaw_rad,
  const double self_filter_radius_m)
{
  if (!std::isfinite(valid_angle_min) || !std::isfinite(valid_angle_max) ||
    valid_angle_min < -kPi || valid_angle_min > kPi ||
    valid_angle_max < -kPi || valid_angle_max > kPi)
  {
    throw std::invalid_argument(
            "low lidar angular bounds must be finite and within [-pi, pi]");
  }
  if (!std::isfinite(sensor_x_m) || !std::isfinite(sensor_y_m) ||
    !std::isfinite(sensor_yaw_rad) || !std::isfinite(self_filter_radius_m) ||
    self_filter_radius_m < 0.0)
  {
    throw std::invalid_argument("low lidar self-filter geometry is invalid");
  }
  if (!std::isfinite(input.angle_min) || !std::isfinite(input.angle_increment) ||
    input.angle_increment <= 0.0F)
  {
    throw std::invalid_argument("LaserScan angle_min/increment are invalid");
  }

  auto output = input;
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  const bool intensities_match = output.intensities.size() == output.ranges.size();
  const double radius_squared = self_filter_radius_m * self_filter_radius_m;
  const bool sector_wraps_at_pi = valid_angle_min > valid_angle_max;
  for (std::size_t index = 0; index < output.ranges.size(); ++index) {
    const double raw_angle = static_cast<double>(input.angle_min) +
      static_cast<double>(index) * static_cast<double>(input.angle_increment);
    const double angle = std::atan2(std::sin(raw_angle), std::cos(raw_angle));
    const bool inside_angular_sector = sector_wraps_at_pi ?
      (angle >= valid_angle_min || angle <= valid_angle_max) :
      (angle >= valid_angle_min && angle <= valid_angle_max);
    bool reject = !inside_angular_sector;
    const double range = static_cast<double>(input.ranges[index]);
    if (!reject && self_filter_radius_m > 0.0 && std::isfinite(range) && range >= 0.0) {
      const double base_angle = angle + sensor_yaw_rad;
      const double base_x = sensor_x_m + range * std::cos(base_angle);
      const double base_y = sensor_y_m + range * std::sin(base_angle);
      reject = base_x * base_x + base_y * base_y <= radius_squared;
    }
    if (reject) {
      output.ranges[index] = invalid;
      if (intensities_match) {
        output.intensities[index] = invalid;
      }
    }
  }
  return output;
}

}  // namespace rk3576_footbath_lidar
