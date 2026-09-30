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

#include "rk3576_footbath_base/protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace rk3576_footbath_base
{
namespace
{

void append_u16(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<uint8_t> & output, uint32_t value)
{
  for (unsigned shift = 0; shift < 32; shift += 8) {
    output.push_back(static_cast<uint8_t>((value >> shift) & 0xFFU));
  }
}

void append_float(std::vector<uint8_t> & output, float value)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "Protocol requires IEEE-754 float32");
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(output, bits);
}

uint16_t read_u16(const uint8_t * input)
{
  return static_cast<uint16_t>(input[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(input[1]) << 8U);
}

uint32_t read_u32(const uint8_t * input)
{
  return static_cast<uint32_t>(input[0]) |
         (static_cast<uint32_t>(input[1]) << 8U) |
         (static_cast<uint32_t>(input[2]) << 16U) |
         (static_cast<uint32_t>(input[3]) << 24U);
}

int32_t read_i32(const uint8_t * input)
{
  const uint32_t unsigned_value = read_u32(input);
  int32_t output = 0;
  std::memcpy(&output, &unsigned_value, sizeof(output));
  return output;
}

float read_float(const uint8_t * input)
{
  const uint32_t bits = read_u32(input);
  float output = 0.0F;
  std::memcpy(&output, &bits, sizeof(output));
  return output;
}

bool finite3(float a, float b, float c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

}  // namespace

uint16_t crc16_ccitt_false(const uint8_t * data, std::size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > kMaxPayloadLength || frame.payload.size() > 0xFFFFU) {
    throw std::length_error("payload exceeds protocol limit");
  }
  std::vector<uint8_t> output;
  output.reserve(10U + frame.payload.size());
  output.push_back(kSof0);
  output.push_back(kSof1);
  output.push_back(frame.version);
  output.push_back(frame.message_type);
  append_u16(output, frame.sequence);
  append_u16(output, static_cast<uint16_t>(frame.payload.size()));
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());
  const uint16_t crc = crc16_ccitt_false(output.data() + 2, output.size() - 2);
  append_u16(output, crc);
  return output;
}

std::vector<uint8_t> encode_cmd_vel(uint16_t sequence, float linear_mps, float angular_rps)
{
  Frame frame;
  frame.message_type = static_cast<uint8_t>(MessageType::kCmdVel);
  frame.sequence = sequence;
  append_float(frame.payload, linear_mps);
  append_float(frame.payload, angular_rps);
  return encode_frame(frame);
}

std::vector<uint8_t> encode_stop(uint16_t sequence)
{
  Frame frame;
  frame.message_type = static_cast<uint8_t>(MessageType::kStop);
  frame.sequence = sequence;
  return encode_frame(frame);
}

bool decode_odom(const std::vector<uint8_t> & payload, OdomPayload & output)
{
  if (payload.size() != 28U) {
    return false;
  }
  output.x_m = read_float(payload.data());
  output.y_m = read_float(payload.data() + 4);
  output.yaw_rad = read_float(payload.data() + 8);
  output.linear_mps = read_float(payload.data() + 12);
  output.angular_rps = read_float(payload.data() + 16);
  output.left_ticks = read_i32(payload.data() + 20);
  output.right_ticks = read_i32(payload.data() + 24);
  return finite3(output.x_m, output.y_m, output.yaw_rad) &&
         std::isfinite(output.linear_mps) && std::isfinite(output.angular_rps);
}

bool decode_heartbeat(const std::vector<uint8_t> & payload, HeartbeatPayload & output)
{
  if (payload.size() != 6U) {
    return false;
  }
  output.uptime_ms = read_u32(payload.data());
  output.fault_flags = read_u16(payload.data() + 4);
  return true;
}

bool decode_range_status(const std::vector<uint8_t> & payload, RangeStatusPayload & output)
{
  if (payload.size() != 16U) {
    return false;
  }
  output.tof_left_m = read_float(payload.data());
  output.tof_right_m = read_float(payload.data() + 4);
  output.ultrasonic_m = read_float(payload.data() + 8);
  output.valid_flags = read_u16(payload.data() + 12);
  const uint16_t packed_flags = read_u16(payload.data() + 14);
  output.obstacle_flags = static_cast<uint8_t>(packed_flags & 0xFFU);
  output.sensor_fault_flags = static_cast<uint8_t>((packed_flags >> 8U) & 0xFFU);
  return finite3(output.tof_left_m, output.tof_right_m, output.ultrasonic_m);
}

bool decode_ultrasonic_three(const std::vector<uint8_t> & payload, UltrasonicThreePayload & output)
{
  if (payload.size() != 36U) return false;
  UltrasonicThreePayload candidate;
  for (unsigned i=0; i<3; ++i) {
    const auto *p = payload.data() + 12*i;
    auto &r = candidate.readings[i];
    r.distance_m = read_float(p);
    r.age_ms = read_u32(p+4);
    r.sequence = read_u16(p+8);
    r.status = p[10];
    if (!std::isfinite(r.distance_m) || r.status > 3 || p[11] != 0) return false;
    if (r.status == 1 && (r.distance_m < 0.02F || r.distance_m > 4.0F)) return false;
  }
  output = candidate;
  return true;
}

bool decode_imu_raw(const std::vector<uint8_t> & payload, ImuRawPayload & output)
{
  if (payload.size() != 28U) {
    return false;
  }
  output.stamp_ms = read_u32(payload.data());
  output.accel_x_mps2 = read_float(payload.data() + 4);
  output.accel_y_mps2 = read_float(payload.data() + 8);
  output.accel_z_mps2 = read_float(payload.data() + 12);
  output.gyro_x_rps = read_float(payload.data() + 16);
  output.gyro_y_rps = read_float(payload.data() + 20);
  output.gyro_z_rps = read_float(payload.data() + 24);
  return finite3(output.accel_x_mps2, output.accel_y_mps2, output.accel_z_mps2) &&
         finite3(output.gyro_x_rps, output.gyro_y_rps, output.gyro_z_rps);
}

Quaternion yaw_to_quaternion(double yaw_rad)
{
  const double half_yaw = yaw_rad * 0.5;
  return Quaternion{0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)};
}

std::vector<Frame> StreamParser::feed(const uint8_t * data, std::size_t length)
{
  if (data != nullptr && length > 0U) {
    buffer_.insert(buffer_.end(), data, data + length);
  }
  std::vector<Frame> frames;
  const std::array<uint8_t, 2> sof_pattern{kSof0, kSof1};

  while (true) {
    auto sof = std::search(
      buffer_.begin(), buffer_.end(), sof_pattern.begin(), sof_pattern.end());
    if (sof == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kSof0) {
        dropped_byte_count_ += buffer_.size() - 1U;
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        dropped_byte_count_ += buffer_.size();
        buffer_.clear();
      }
      break;
    }
    const std::size_t prefix = static_cast<std::size_t>(std::distance(buffer_.begin(), sof));
    dropped_byte_count_ += prefix;
    buffer_.erase(buffer_.begin(), sof);

    if (buffer_.size() < 8U) {
      break;
    }
    const std::size_t payload_length = read_u16(buffer_.data() + 6);
    if (payload_length > kMaxPayloadLength) {
      ++length_error_count_;
      ++dropped_byte_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t frame_length = 10U + payload_length;
    if (buffer_.size() < frame_length) {
      break;
    }
    const uint16_t expected_crc = read_u16(buffer_.data() + 8U + payload_length);
    const uint16_t actual_crc = crc16_ccitt_false(buffer_.data() + 2, 6U + payload_length);
    if (actual_crc != expected_crc) {
      ++crc_error_count_;
      ++dropped_byte_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    Frame frame;
    frame.version = buffer_[2];
    frame.message_type = buffer_[3];
    frame.sequence = read_u16(buffer_.data() + 4);
    frame.payload.assign(buffer_.begin() + 8, buffer_.begin() + 8 + payload_length);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_length);
  }
  return frames;
}

}  // namespace rk3576_footbath_base
