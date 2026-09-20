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

#ifndef RK3576_FOOTBATH_BASE__PROTOCOL_HPP_
#define RK3576_FOOTBATH_BASE__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rk3576_footbath_base
{

constexpr uint8_t kSof0 = 0xAA;
constexpr uint8_t kSof1 = 0x55;
constexpr uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaxPayloadLength = 1024;

enum class MessageType : uint8_t
{
  kCmdVel = 0x01,
  kOdom = 0x02,
  kHeartbeat = 0x03,
  kStop = 0x04,
  kRangeStatus = 0x05,
  kImuRaw = 0x06,
  kControlSource = 0x07,
};

struct Frame
{
  uint8_t version{kProtocolVersion};
  uint8_t message_type{0};
  uint16_t sequence{0};
  std::vector<uint8_t> payload;
};

struct OdomPayload
{
  float x_m{0.0F};
  float y_m{0.0F};
  float yaw_rad{0.0F};
  float linear_mps{0.0F};
  float angular_rps{0.0F};
  int32_t left_ticks{0};
  int32_t right_ticks{0};
};

struct HeartbeatPayload
{
  uint32_t uptime_ms{0};
  uint16_t fault_flags{0};
};

struct RangeStatusPayload
{
  float tof_left_m{0.0F};
  float tof_right_m{0.0F};
  float ultrasonic_m{0.0F};
  uint16_t valid_flags{0};
  uint8_t obstacle_flags{0};
  uint8_t sensor_fault_flags{0};
};

struct ImuRawPayload
{
  uint32_t stamp_ms{0};
  float accel_x_mps2{0.0F};
  float accel_y_mps2{0.0F};
  float accel_z_mps2{0.0F};
  float gyro_x_rps{0.0F};
  float gyro_y_rps{0.0F};
  float gyro_z_rps{0.0F};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

uint16_t crc16_ccitt_false(const uint8_t * data, std::size_t length);
std::vector<uint8_t> encode_frame(const Frame & frame);
std::vector<uint8_t> encode_cmd_vel(uint16_t sequence, float linear_mps, float angular_rps);
std::vector<uint8_t> encode_stop(uint16_t sequence);
bool decode_odom(const std::vector<uint8_t> & payload, OdomPayload & output);
bool decode_heartbeat(const std::vector<uint8_t> & payload, HeartbeatPayload & output);
bool decode_range_status(const std::vector<uint8_t> & payload, RangeStatusPayload & output);
bool decode_imu_raw(const std::vector<uint8_t> & payload, ImuRawPayload & output);
Quaternion yaw_to_quaternion(double yaw_rad);

class StreamParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t length);
  std::size_t crc_error_count() const {return crc_error_count_;}
  std::size_t length_error_count() const {return length_error_count_;}
  std::size_t dropped_byte_count() const {return dropped_byte_count_;}

private:
  std::vector<uint8_t> buffer_;
  std::size_t crc_error_count_{0};
  std::size_t length_error_count_{0};
  std::size_t dropped_byte_count_{0};
};

}  // namespace rk3576_footbath_base

#endif  // RK3576_FOOTBATH_BASE__PROTOCOL_HPP_
