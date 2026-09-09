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
#include "rk3576_footbath_lidar/dual_lidar_fusion.hpp"

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace rk3576_footbath_lidar
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finite_range(const float range, const sensor_msgs::msg::LaserScan & scan)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

std::optional<std::size_t> angle_to_index(
  const double wrapped_angle,
  const sensor_msgs::msg::LaserScan & scan)
{
  const auto count = scan.ranges.size();
  const double increment = static_cast<double>(scan.angle_increment);
  std::optional<std::size_t> best_index;
  double best_error = std::numeric_limits<double>::infinity();

  for (int turn = -2; turn <= 2; ++turn) {
    const double candidate = wrapped_angle + static_cast<double>(turn) * kTwoPi;
    const double raw_index =
      (candidate - static_cast<double>(scan.angle_min)) / increment;
    const auto rounded = static_cast<std::int64_t>(std::llround(raw_index));
    if (rounded < 0 || static_cast<std::size_t>(rounded) >= count) {
      continue;
    }
    const double bin_angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(rounded) * increment;
    const double error = std::abs(
      std::atan2(
        std::sin(bin_angle - wrapped_angle), std::cos(bin_angle - wrapped_angle)));
    if (error < best_error) {
      best_error = error;
      best_index = static_cast<std::size_t>(rounded);
    }
  }

  if (best_index && best_error <= std::abs(increment) * 0.51 + 1.0e-9) {
    return best_index;
  }
  return std::nullopt;
}

void validate_scan(const sensor_msgs::msg::LaserScan & scan, const char * name)
{
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_increment) || std::abs(scan.angle_increment) < 1.0e-12F ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_min < 0.0F || scan.range_max <= scan.range_min)
  {
    throw std::invalid_argument(std::string(name) + " LaserScan metadata is invalid");
  }
}
}  // namespace

sensor_msgs::msg::LaserScan fuse_laser_scans(
  const sensor_msgs::msg::LaserScan & high_scan,
  const sensor_msgs::msg::LaserScan & filtered_low_scan,
  const PlanarTransform & low_to_high,
  FusionStatistics * statistics)
{
  validate_scan(high_scan, "high");
  validate_scan(filtered_low_scan, "low");
  if (!std::isfinite(low_to_high.x_m) || !std::isfinite(low_to_high.y_m) ||
    !std::isfinite(low_to_high.yaw_rad))
  {
    throw std::invalid_argument("low-to-high transform is invalid");
  }

  FusionStatistics local_statistics;
  auto output = high_scan;
  const bool output_has_intensity = output.intensities.size() == output.ranges.size();
  const bool low_has_intensity =
    filtered_low_scan.intensities.size() == filtered_low_scan.ranges.size();
  const double cosine = std::cos(low_to_high.yaw_rad);
  const double sine = std::sin(low_to_high.yaw_rad);

  for (std::size_t index = 0; index < filtered_low_scan.ranges.size(); ++index) {
    const float low_range = filtered_low_scan.ranges[index];
    if (!finite_range(low_range, filtered_low_scan)) {
      continue;
    }
    ++local_statistics.low_finite_points;

    const double low_angle = static_cast<double>(filtered_low_scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(filtered_low_scan.angle_increment);
    const double low_x = static_cast<double>(low_range) * std::cos(low_angle);
    const double low_y = static_cast<double>(low_range) * std::sin(low_angle);
    const double high_x = low_to_high.x_m + cosine * low_x - sine * low_y;
    const double high_y = low_to_high.y_m + sine * low_x + cosine * low_y;
    const double projected_range = std::hypot(high_x, high_y);
    if (!std::isfinite(projected_range) || projected_range < high_scan.range_min ||
      projected_range > high_scan.range_max)
    {
      continue;
    }

    const double projected_angle = std::atan2(high_y, high_x);
    const auto output_index = angle_to_index(projected_angle, high_scan);
    if (!output_index) {
      continue;
    }
    ++local_statistics.projected_points;

    const float current = output.ranges[*output_index];
    if (!finite_range(current, high_scan) || projected_range < current) {
      output.ranges[*output_index] = static_cast<float>(projected_range);
      if (output_has_intensity) {
        output.intensities[*output_index] = low_has_intensity ?
          filtered_low_scan.intensities[index] :
          std::numeric_limits<float>::quiet_NaN();
      }
      ++local_statistics.bins_updated;
    }
  }

  if (statistics != nullptr) {
    *statistics = local_statistics;
  }
  return output;
}

}  // namespace rk3576_footbath_lidar
