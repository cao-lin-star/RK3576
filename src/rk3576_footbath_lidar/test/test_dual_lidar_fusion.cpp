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
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rk3576_footbath_lidar/dual_lidar_fusion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
sensor_msgs::msg::LaserScan make_scan(float angle_min = -1.0F, float increment = 0.5F)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = angle_min;
  scan.angle_increment = increment;
  scan.angle_max = angle_min + 4.0F * increment;
  scan.range_min = 0.10F;
  scan.range_max = 10.0F;
  scan.ranges.assign(5U, std::numeric_limits<float>::infinity());
  scan.intensities.assign(5U, 1.0F);
  return scan;
}
}  // namespace

TEST(DualLidarFusion, UsesNearestFiniteReturnAndPreservesHighMetadata)
{
  auto high = make_scan();
  auto low = make_scan();
  high.header.frame_id = "laser_high_frame";
  high.header.stamp.sec = 12;
  high.ranges[2] = 3.0F;
  high.intensities[2] = 30.0F;
  low.ranges[2] = 2.0F;
  low.intensities[2] = 20.0F;

  rk3576_footbath_lidar::FusionStatistics statistics;
  const auto output = rk3576_footbath_lidar::fuse_laser_scans(
    high, low, {}, &statistics);

  EXPECT_EQ(output.header.frame_id, "laser_high_frame");
  EXPECT_EQ(output.header.stamp.sec, 12);
  EXPECT_FLOAT_EQ(output.angle_min, high.angle_min);
  EXPECT_FLOAT_EQ(output.ranges[2], 2.0F);
  EXPECT_FLOAT_EQ(output.intensities[2], 20.0F);
  EXPECT_EQ(statistics.low_finite_points, 1U);
  EXPECT_EQ(statistics.projected_points, 1U);
  EXPECT_EQ(statistics.bins_updated, 1U);
}

TEST(DualLidarFusion, NeverUsesNanOrInfinityToClearHighObstacle)
{
  auto high = make_scan();
  auto low = make_scan();
  high.ranges[1] = 1.5F;
  high.ranges[2] = 2.5F;
  low.ranges[1] = std::numeric_limits<float>::quiet_NaN();
  low.ranges[2] = std::numeric_limits<float>::infinity();

  const auto output = rk3576_footbath_lidar::fuse_laser_scans(high, low, {});

  EXPECT_FLOAT_EQ(output.ranges[1], 1.5F);
  EXPECT_FLOAT_EQ(output.ranges[2], 2.5F);
}

TEST(DualLidarFusion, KeepsCloserHighObstacle)
{
  auto high = make_scan();
  auto low = make_scan();
  high.ranges[2] = 0.8F;
  low.ranges[2] = 1.2F;

  rk3576_footbath_lidar::FusionStatistics statistics;
  const auto output = rk3576_footbath_lidar::fuse_laser_scans(
    high, low, {}, &statistics);

  EXPECT_FLOAT_EQ(output.ranges[2], 0.8F);
  EXPECT_EQ(statistics.projected_points, 1U);
  EXPECT_EQ(statistics.bins_updated, 0U);
}

TEST(DualLidarFusion, AppliesTranslationBeforeRebinning)
{
  auto high = make_scan();
  auto low = make_scan();
  low.ranges[2] = 1.0F;

  const rk3576_footbath_lidar::PlanarTransform transform{0.5, 0.0, 0.0};
  const auto output = rk3576_footbath_lidar::fuse_laser_scans(high, low, transform);

  EXPECT_NEAR(output.ranges[2], 1.5F, 1.0e-6F);
}

TEST(DualLidarFusion, AppliesRotationAndHandlesWrappedPiBin)
{
  constexpr float pi = 3.14159265358979323846F;
  auto high = make_scan(-pi, pi / 4.0F);
  high.ranges.assign(9U, std::numeric_limits<float>::infinity());
  high.intensities.assign(9U, 1.0F);
  high.angle_max = pi;
  auto low = make_scan();
  low.ranges[2] = 1.0F;

  const rk3576_footbath_lidar::PlanarTransform transform{0.0, 0.0, pi};
  const auto output = rk3576_footbath_lidar::fuse_laser_scans(high, low, transform);

  EXPECT_TRUE(std::isfinite(output.ranges.front()) || std::isfinite(output.ranges.back()));
  EXPECT_NEAR(
    std::min(output.ranges.front(), output.ranges.back()), 1.0F, 1.0e-6F);
}

TEST(DualLidarFusion, SupportsNegativeOutputAngleIncrement)
{
  auto high = make_scan(1.0F, -0.5F);
  auto low = make_scan();
  low.ranges[2] = 1.0F;

  const auto output = rk3576_footbath_lidar::fuse_laser_scans(high, low, {});

  EXPECT_FLOAT_EQ(output.ranges[2], 1.0F);
}

TEST(DualLidarFusion, RejectsInvalidScanAndTransform)
{
  auto high = make_scan();
  auto low = make_scan();
  high.angle_increment = 0.0F;
  EXPECT_THROW(
    rk3576_footbath_lidar::fuse_laser_scans(high, low, {}), std::invalid_argument);

  high = make_scan();
  const rk3576_footbath_lidar::PlanarTransform invalid{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  EXPECT_THROW(
    rk3576_footbath_lidar::fuse_laser_scans(high, low, invalid), std::invalid_argument);
}
