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
#include <cmath>
#include <stdexcept>

#include "gtest/gtest.h"
#include "rk3576_footbath_lidar/low_lidar_angular_filter.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{
constexpr float kPi = 3.14159265358979323846F;

sensor_msgs::msg::LaserScan make_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser_low_frame";
  scan.angle_min = -kPi;
  scan.angle_increment = kPi / 4.0F;
  scan.angle_max = kPi;
  scan.ranges.assign(9U, 1.0F);
  scan.intensities.assign(9U, 10.0F);
  return scan;
}
}  // namespace

TEST(LowLidarAngularFilter, KeepsInclusiveFrontSectorAndPreservesMetadata)
{
  const auto input = make_scan();
  const auto output = rk3576_footbath_lidar::filter_scan_by_angle(
    input, -0.8, 0.8);

  EXPECT_EQ(output.header.frame_id, input.header.frame_id);
  EXPECT_EQ(output.angle_min, input.angle_min);
  EXPECT_EQ(output.angle_max, input.angle_max);
  ASSERT_EQ(output.ranges.size(), 9U);
  for (std::size_t index = 0; index < output.ranges.size(); ++index) {
    if (index >= 3U && index <= 5U) {
      EXPECT_FLOAT_EQ(output.ranges[index], 1.0F);
      EXPECT_FLOAT_EQ(output.intensities[index], 10.0F);
    } else {
      EXPECT_TRUE(std::isnan(output.ranges[index]));
      EXPECT_TRUE(std::isnan(output.intensities[index]));
    }
  }
}

TEST(LowLidarAngularFilter, NormalizesZeroToTwoPiScansAroundFront)
{
  sensor_msgs::msg::LaserScan input;
  input.header.frame_id = "laser_low_frame";
  input.angle_min = 0.0F;
  input.angle_increment = kPi / 4.0F;
  input.angle_max = 2.0F * kPi;
  input.ranges.assign(9U, 2.0F);

  const auto output = rk3576_footbath_lidar::filter_scan_by_angle(input, -0.8, 0.8);
  for (std::size_t index = 0; index < output.ranges.size(); ++index) {
    const bool expected_front = index == 0U || index == 1U || index == 7U || index == 8U;
    if (expected_front) {
      EXPECT_FLOAT_EQ(output.ranges[index], 2.0F);
    } else {
      EXPECT_TRUE(std::isnan(output.ranges[index]));
    }
  }
}

TEST(LowLidarAngularFilter, KeepsWrappedSectorAcrossPi)
{
  const auto input = make_scan();
  const auto output = rk3576_footbath_lidar::filter_scan_by_angle(
    input, 1.9, -1.9);

  for (std::size_t index = 0; index < output.ranges.size(); ++index) {
    const bool expected_front = index <= 1U || index >= 7U;
    if (expected_front) {
      EXPECT_FLOAT_EQ(output.ranges[index], 1.0F);
    } else {
      EXPECT_TRUE(std::isnan(output.ranges[index]));
    }
  }
}

TEST(LowLidarAngularFilter, RejectsReturnsInsideChassisButKeepsExternalObstacle)
{
  sensor_msgs::msg::LaserScan input;
  input.angle_min = 0.0F;
  input.angle_increment = 0.5F;
  input.angle_max = 0.5F;
  input.ranges = {0.10F, 0.40F};
  input.intensities = {1.0F, 2.0F};

  const auto output = rk3576_footbath_lidar::filter_scan_by_angle(
    input, -0.8, 0.8, 0.10, 0.0, 0.0, 0.225);

  EXPECT_TRUE(std::isnan(output.ranges[0]));
  EXPECT_TRUE(std::isnan(output.intensities[0]));
  EXPECT_FLOAT_EQ(output.ranges[1], 0.40F);
  EXPECT_FLOAT_EQ(output.intensities[1], 2.0F);
}

TEST(LowLidarAngularFilter, SelfFilterUsesCorrectedPiYaw)
{
  sensor_msgs::msg::LaserScan input;
  input.angle_min = kPi;
  input.angle_increment = 0.5F;
  input.angle_max = kPi + 0.5F;
  input.ranges = {0.10F, 0.40F};
  input.intensities = {1.0F, 2.0F};

  const auto output = rk3576_footbath_lidar::filter_scan_by_angle(
    input, 1.9, -1.9, 0.10, 0.0, kPi, 0.225);

  EXPECT_TRUE(std::isnan(output.ranges[0]));
  EXPECT_TRUE(std::isnan(output.intensities[0]));
  EXPECT_FLOAT_EQ(output.ranges[1], 0.40F);
  EXPECT_FLOAT_EQ(output.intensities[1], 2.0F);
}

TEST(LowLidarAngularFilter, RejectsInvalidSelfFilterGeometry)
{
  const auto input = make_scan();
  EXPECT_THROW(
    rk3576_footbath_lidar::filter_scan_by_angle(
      input, -1.0, 1.0, 0.10, 0.0, 0.0, -0.01),
    std::invalid_argument);
}
TEST(LowLidarAngularFilter, RejectsBoundsOutsideNormalizedRange)
{
  const auto input = make_scan();
  EXPECT_THROW(
    rk3576_footbath_lidar::filter_scan_by_angle(input, 3.2, -1.0),
    std::invalid_argument);
}

TEST(LowLidarAngularFilter, RejectsInvalidScanGeometry)
{
  auto input = make_scan();
  input.angle_increment = 0.0F;
  EXPECT_THROW(
    rk3576_footbath_lidar::filter_scan_by_angle(input, -1.0, 1.0),
    std::invalid_argument);
}
