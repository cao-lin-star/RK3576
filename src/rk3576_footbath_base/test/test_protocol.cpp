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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rk3576_footbath_base/protocol.hpp"

namespace fb = rk3576_footbath_base;

TEST(UltrasonicThree, RejectsMalformedAndNonFinite) {
  fb::UltrasonicThreePayload result;
  std::vector<uint8_t> p(36,0);
  EXPECT_TRUE(fb::decode_ultrasonic_three(p,result));
  p[10]=4; EXPECT_FALSE(fb::decode_ultrasonic_three(p,result)); p[10]=0;
  p[11]=1; EXPECT_FALSE(fb::decode_ultrasonic_three(p,result)); p[11]=0;
  p[2]=0x80; p[3]=0x7f; EXPECT_FALSE(fb::decode_ultrasonic_three(p,result));
  p.resize(35); EXPECT_FALSE(fb::decode_ultrasonic_three(p,result));
}

TEST(UltrasonicThree, DecodesChannelsAgesSequenceAndStatus) {
  fb::UltrasonicThreePayload result;
  std::vector<uint8_t> p(36,0);
  for(unsigned i=0;i<3;++i) {
    float v=0.5F+float(i); std::memcpy(p.data()+12*i,&v,4);
    p[12*i+4]=80; p[12*i+8]=254; p[12*i+9]=255; p[12*i+10]=1;
  }
  EXPECT_TRUE(fb::decode_ultrasonic_three(p,result));
  EXPECT_FLOAT_EQ(result.readings[2].distance_m,2.5F);
  EXPECT_EQ(result.readings[1].age_ms,80U);
  EXPECT_EQ(result.readings[0].sequence,65534U);
  p[22]=2; EXPECT_TRUE(fb::decode_ultrasonic_three(p,result));
  EXPECT_EQ(result.readings[1].status,2U);
  float zero=0;std::memcpy(p.data(),&zero,4);
  EXPECT_FALSE(fb::decode_ultrasonic_three(p,result));
}

namespace
{
void append_u16(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<uint8_t> & output, uint32_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

void append_i32(std::vector<uint8_t> & output, int32_t value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(output, bits);
}

void append_float(std::vector<uint8_t> & output, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(output, bits);
}
}  // namespace

TEST(Protocol, CrcMatchesCanonicalCheckVector)
{
  const char * text = "123456789";
  EXPECT_EQ(fb::crc16_ccitt_false(reinterpret_cast<const uint8_t *>(text), 9), 0x29B1U);
}

TEST(Protocol, CmdVelAndStreamParserRemainCompatible)
{
  const auto first = fb::encode_cmd_vel(7U, 0.1F, 0.2F);
  const auto second = fb::encode_stop(8U);
  std::vector<uint8_t> stream{0x00U, 0xAAU, 0x10U};
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());
  fb::StreamParser parser;
  const auto frames = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].message_type, static_cast<uint8_t>(fb::MessageType::kCmdVel));
  EXPECT_EQ(frames[1].message_type, static_cast<uint8_t>(fb::MessageType::kStop));
}

TEST(Protocol, DecodesOdometryRangeAndImuPayloads)
{
  std::vector<uint8_t> odom_bytes;
  append_float(odom_bytes, 1.25F);
  append_float(odom_bytes, -0.5F);
  append_float(odom_bytes, 1.57079632679F);
  append_float(odom_bytes, 0.15F);
  append_float(odom_bytes, -0.25F);
  append_i32(odom_bytes, 123456);
  append_i32(odom_bytes, -654321);
  fb::OdomPayload odom;
  ASSERT_TRUE(fb::decode_odom(odom_bytes, odom));
  EXPECT_EQ(odom.left_ticks, 123456);

  std::vector<uint8_t> range_bytes;
  append_float(range_bytes, 0.25F);
  append_float(range_bytes, 0.30F);
  append_float(range_bytes, 0.40F);
  append_u16(range_bytes, 0x000FU);
  append_u16(range_bytes, 0x1203U);
  fb::RangeStatusPayload ranges;
  ASSERT_TRUE(fb::decode_range_status(range_bytes, ranges));
  EXPECT_FLOAT_EQ(ranges.tof_left_m, 0.25F);
  EXPECT_EQ(ranges.obstacle_flags, 0x03U);
  EXPECT_EQ(ranges.sensor_fault_flags, 0x12U);

  std::vector<uint8_t> imu_bytes;
  append_u32(imu_bytes, 1234U);
  for (float value : {1.0F, 2.0F, 3.0F, 0.1F, 0.2F, 0.3F}) {
    append_float(imu_bytes, value);
  }
  fb::ImuRawPayload imu;
  ASSERT_TRUE(fb::decode_imu_raw(imu_bytes, imu));
  EXPECT_EQ(imu.stamp_ms, 1234U);
  EXPECT_FLOAT_EQ(imu.gyro_z_rps, 0.3F);
}

TEST(Protocol, RejectsBadLengthAndNonFiniteSensorValues)
{
  fb::RangeStatusPayload ranges;
  EXPECT_FALSE(fb::decode_range_status(std::vector<uint8_t>(15U, 0U), ranges));
  std::vector<uint8_t> payload(16U, 0U);
  const float nan = std::nanf("");
  std::memcpy(payload.data(), &nan, sizeof(nan));
  EXPECT_FALSE(fb::decode_range_status(payload, ranges));
}
