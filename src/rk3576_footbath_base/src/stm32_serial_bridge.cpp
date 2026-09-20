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

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rk3576_footbath_base/protocol.hpp"

using namespace std::chrono_literals;

namespace rk3576_footbath_base
{
namespace
{

constexpr uint16_t kValidTofLeft = 1U << 0;
constexpr uint16_t kValidTofRight = 1U << 1;
constexpr uint16_t kValidUltrasonic = 1U << 2;
constexpr uint16_t kValidImu = 1U << 3;

speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return static_cast<speed_t>(0);
  }
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

class Stm32SerialBridge : public rclcpp::Node
{
public:
  Stm32SerialBridge()
  : Node("stm32_serial_bridge"), tf_broadcaster_(*this)
  {
    port_ = declare_parameter<std::string>("serial.port", "/dev/footbath_stm32");
    baud_ = declare_parameter<int>("serial.baudrate", 115200);
    reconnect_period_s_ = declare_parameter<double>("serial.reconnect_period_s", 1.0);
    cmd_timeout_s_ = declare_parameter<double>("safety.cmd_vel_timeout_s", 0.5);
    heartbeat_timeout_s_ = declare_parameter<double>("safety.heartbeat_timeout_s", 2.0);
    max_linear_mps_ = declare_parameter<double>("safety.max_linear_mps", 1.00);
    max_angular_rps_ = declare_parameter<double>("safety.max_angular_rps", 1.50);
    stop_on_fault_ = declare_parameter<bool>("safety.stop_on_fault", true);
    publish_tf_ = declare_parameter<bool>("publish_odom_tf", true);

    odom_frame_ = declare_parameter<std::string>("frames.odom", "odom");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("frames.imu", "imu_link");
    tof_left_frame_ = declare_parameter<std::string>("frames.tof_left", "tof_left_link");
    tof_right_frame_ = declare_parameter<std::string>("frames.tof_right", "tof_right_link");
    ultrasonic_frame_ =
      declare_parameter<std::string>("frames.ultrasonic", "ultrasonic_link");

    cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", "/cmd_vel_selected");
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/odom");
    imu_topic_ = declare_parameter<std::string>("topics.imu", "/imu/data_raw");
    tof_left_topic_ = declare_parameter<std::string>("topics.tof_left", "/range/tof_left");
    tof_right_topic_ = declare_parameter<std::string>("topics.tof_right", "/range/tof_right");
    ultrasonic_topic_ =
      declare_parameter<std::string>("topics.ultrasonic", "/range/ultrasonic");

    pose_covariance_diagonal_ = declare_parameter<std::vector<double>>(
      "odom.pose_covariance_diagonal", {0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.05});
    twist_covariance_diagonal_ = declare_parameter<std::vector<double>>(
      "odom.twist_covariance_diagonal", {0.03, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.08});

    if (baud_to_termios(baud_) == static_cast<speed_t>(0)) {
      throw std::invalid_argument("Unsupported serial baudrate: " + std::to_string(baud_));
    }
    if (pose_covariance_diagonal_.size() != 6U || twist_covariance_diagonal_.size() != 6U) {
      throw std::invalid_argument("Odometry covariance diagonal must contain six values");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    source_publisher_ = create_publisher<std_msgs::msg::UInt8>("/chassis/control_source", 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    tof_left_publisher_ =
      create_publisher<sensor_msgs::msg::Range>(tof_left_topic_, rclcpp::SensorDataQoS());
    tof_right_publisher_ =
      create_publisher<sensor_msgs::msg::Range>(tof_right_topic_, rclcpp::SensorDataQoS());
    ultrasonic_publisher_ =
      create_publisher<sensor_msgs::msg::Range>(ultrasonic_topic_, rclcpp::SensorDataQoS());
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&Stm32SerialBridge::on_cmd_vel, this, std::placeholders::_1));

    io_timer_ = create_wall_timer(10ms, std::bind(&Stm32SerialBridge::service_serial, this));
    diagnostic_timer_ =
      create_wall_timer(1s, std::bind(&Stm32SerialBridge::publish_diagnostics, this));
    next_reconnect_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "STM32 serial bridge ready: %s @ %d", port_.c_str(), baud_);
  }

  ~Stm32SerialBridge() override
  {
    if (fd_ >= 0) {
      send_stop("ROS node shutdown");
      send_stop("ROS node shutdown");
      ::tcdrain(fd_);
    }
    close_port();
  }

private:
  bool open_port()
  {
    const int descriptor = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (descriptor < 0) {
      return false;
    }
    termios options{};
    if (::tcgetattr(descriptor, &options) != 0) {
      ::close(descriptor);
      return false;
    }
    ::cfmakeraw(&options);
    const speed_t speed = baud_to_termios(baud_);
    ::cfsetispeed(&options, speed);
    ::cfsetospeed(&options, speed);
    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::tcsetattr(descriptor, TCSANOW, &options) != 0) {
      ::close(descriptor);
      return false;
    }
    ::tcflush(descriptor, TCIOFLUSH);
    fd_ = descriptor;
    parser_ = StreamParser{};
    connected_since_ = std::chrono::steady_clock::now();
    last_heartbeat_ = {};
    heartbeat_stale_stop_sent_ = false;
    RCLCPP_INFO(get_logger(), "Connected to STM32 on %s", port_.c_str());
    send_stop("serial link connected; clear stale motion");
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
      ++disconnect_count_;
      RCLCPP_WARN(get_logger(), "STM32 serial link disconnected; reconnecting");
    }
    next_reconnect_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_period_s_));
  }

  bool write_bytes(const std::vector<uint8_t> & bytes)
  {
    if (fd_ < 0) {
      return false;
    }
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const ssize_t written = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd item{fd_, POLLOUT, 0};
        if (::poll(&item, 1, 20) > 0) {
          continue;
        }
      }
      ++write_error_count_;
      close_port();
      return false;
    }
    return true;
  }

  void send_stop(const char * reason)
  {
    if (fd_ >= 0 && write_bytes(encode_stop(tx_sequence_++))) {
      ++stop_count_;
      RCLCPP_WARN(get_logger(), "STOP sent: %s", reason);
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    last_cmd_vel_ = std::chrono::steady_clock::now();
    received_cmd_vel_ = true;
    timeout_stop_sent_ = false;
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      ++invalid_cmd_count_;
      send_stop("cmd_vel contains NaN or Inf");
      return;
    }
    const double linear = std::clamp(message->linear.x, -max_linear_mps_, max_linear_mps_);
    const double angular = std::clamp(message->angular.z, -max_angular_rps_, max_angular_rps_);
    if (linear != message->linear.x || angular != message->angular.z) {
      ++clamped_cmd_count_;
    }
    if (fd_ >= 0 && write_bytes(
        encode_cmd_vel(tx_sequence_++, static_cast<float>(linear), static_cast<float>(angular))))
    {
      ++cmd_vel_tx_count_;
    }
  }

  void service_serial()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
      if (steady_now >= next_reconnect_) {
        ++reconnect_attempt_count_;
        if (!open_port()) {
          next_reconnect_ = steady_now +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(reconnect_period_s_));
        }
      }
      return;
    }

    read_available();
    if (fd_ < 0) {
      return;
    }
    if (received_cmd_vel_ && !timeout_stop_sent_ &&
      std::chrono::duration<double>(steady_now - last_cmd_vel_).count() > cmd_timeout_s_)
    {
      send_stop("cmd_vel timeout");
      timeout_stop_sent_ = true;
    }
    const auto heartbeat_reference = last_heartbeat_.time_since_epoch().count() == 0 ?
      connected_since_ : last_heartbeat_;
    if (!heartbeat_stale_stop_sent_ &&
      std::chrono::duration<double>(steady_now - heartbeat_reference).count() >
      heartbeat_timeout_s_)
    {
      send_stop("MCU heartbeat timeout");
      heartbeat_stale_stop_sent_ = true;
    }
  }

  void read_available()
  {
    std::array<uint8_t, 512> buffer{};
    while (fd_ >= 0) {
      const ssize_t length = ::read(fd_, buffer.data(), buffer.size());
      if (length > 0) {
        rx_byte_count_ += static_cast<uint64_t>(length);
        const auto frames = parser_.feed(buffer.data(), static_cast<std::size_t>(length));
        for (const auto & frame : frames) {
          handle_frame(frame);
        }
        continue;
      }
      if (length == 0 || (length < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
        break;
      }
      if (length < 0 && errno == EINTR) {
        continue;
      }
      ++read_error_count_;
      close_port();
      break;
    }
  }

  void handle_frame(const Frame & frame)
  {
    if (frame.version != kProtocolVersion) {
      ++version_error_count_;
      send_stop("unsupported protocol version");
      return;
    }
    ++rx_frame_count_;
    switch (static_cast<MessageType>(frame.message_type)) {
      case MessageType::kOdom: {
          OdomPayload odom;
          if (!decode_odom(frame.payload, odom)) {
            ++payload_error_count_;
          } else {
            last_left_ticks_ = odom.left_ticks;
            last_right_ticks_ = odom.right_ticks;
            publish_odom(odom);
          }
          break;
        }
      case MessageType::kHeartbeat: {
          HeartbeatPayload heartbeat;
          if (!decode_heartbeat(frame.payload, heartbeat)) {
            ++payload_error_count_;
            break;
          }
          last_heartbeat_ = std::chrono::steady_clock::now();
          heartbeat_stale_stop_sent_ = false;
          mcu_uptime_ms_ = heartbeat.uptime_ms;
          const uint16_t previous_fault = fault_flags_;
          fault_flags_ = heartbeat.fault_flags;
          if (stop_on_fault_ && fault_flags_ != 0U && previous_fault == 0U) {
            send_stop("MCU reported fault flags");
          }
          break;
        }
      case MessageType::kRangeStatus: {
          RangeStatusPayload ranges;
          if (!decode_range_status(frame.payload, ranges)) {
            ++payload_error_count_;
          } else {
            valid_sensor_flags_ = ranges.valid_flags;
            obstacle_flags_ = ranges.obstacle_flags;
            sensor_fault_flags_ = ranges.sensor_fault_flags;
            publish_ranges(ranges);
          }
          break;
        }
      case MessageType::kControlSource: {
          if (frame.payload.size() == 1 && frame.payload[0] <= 3) {
            std_msgs::msg::UInt8 source;
            source.data = frame.payload[0];
            source_publisher_->publish(source);
          } else { ++payload_error_count_; }
          break;
        }
      case MessageType::kImuRaw: {
          ImuRawPayload imu;
          if (!decode_imu_raw(frame.payload, imu)) {
            ++payload_error_count_;
          } else if ((valid_sensor_flags_ & kValidImu) != 0U) {
            publish_imu(imu);
          }
          break;
        }
      default:
        ++unknown_message_count_;
        break;
    }
  }

  void publish_odom(const OdomPayload & input)
  {
    const auto stamp = now();
    const Quaternion quaternion = yaw_to_quaternion(input.yaw_rad);
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = input.x_m;
    message.pose.pose.position.y = input.y_m;
    message.pose.pose.orientation.x = quaternion.x;
    message.pose.pose.orientation.y = quaternion.y;
    message.pose.pose.orientation.z = quaternion.z;
    message.pose.pose.orientation.w = quaternion.w;
    message.twist.twist.linear.x = input.linear_mps;
    message.twist.twist.angular.z = input.angular_rps;
    for (std::size_t index = 0; index < 6U; ++index) {
      message.pose.covariance[index * 6U + index] = pose_covariance_diagonal_[index];
      message.twist.covariance[index * 6U + index] = twist_covariance_diagonal_[index];
    }
    odom_publisher_->publish(message);
    ++odom_publish_count_;

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = input.x_m;
      transform.transform.translation.y = input.y_m;
      transform.transform.rotation = message.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  void publish_one_range(
    float value, const std::string & frame, uint8_t radiation_type,
    float field_of_view, float min_range, float max_range,
    const rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr & publisher)
  {
    sensor_msgs::msg::Range message;
    message.header.stamp = now();
    message.header.frame_id = frame;
    message.radiation_type = radiation_type;
    message.field_of_view = field_of_view;
    message.min_range = min_range;
    message.max_range = max_range;
    message.range = value;
    publisher->publish(message);
    ++range_publish_count_;
  }

  void publish_ranges(const RangeStatusPayload & ranges)
  {
    if ((ranges.valid_flags & kValidTofLeft) != 0U) {
      publish_one_range(
        ranges.tof_left_m, tof_left_frame_, sensor_msgs::msg::Range::INFRARED,
        0.05F, 0.02F, 12.0F, tof_left_publisher_);
    }
    if ((ranges.valid_flags & kValidTofRight) != 0U) {
      publish_one_range(
        ranges.tof_right_m, tof_right_frame_, sensor_msgs::msg::Range::INFRARED,
        0.05F, 0.02F, 12.0F, tof_right_publisher_);
    }
    if ((ranges.valid_flags & kValidUltrasonic) != 0U) {
      publish_one_range(
        ranges.ultrasonic_m, ultrasonic_frame_, sensor_msgs::msg::Range::ULTRASOUND,
        0.50F, 0.02F, 4.0F, ultrasonic_publisher_);
    }
  }

  void publish_imu(const ImuRawPayload & input)
  {
    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = imu_frame_;
    message.orientation_covariance[0] = -1.0;  // Six-axis IMU has no orientation estimate.
    message.linear_acceleration.x = input.accel_x_mps2;
    message.linear_acceleration.y = input.accel_y_mps2;
    message.linear_acceleration.z = input.accel_z_mps2;
    message.angular_velocity.x = input.gyro_x_rps;
    message.angular_velocity.y = input.gyro_y_rps;
    message.angular_velocity.z = input.gyro_z_rps;
    message.linear_acceleration_covariance[0] = 0.04;
    message.linear_acceleration_covariance[4] = 0.04;
    message.linear_acceleration_covariance[8] = 0.04;
    message.angular_velocity_covariance[0] = 0.02;
    message.angular_velocity_covariance[4] = 0.02;
    message.angular_velocity_covariance[8] = 0.02;
    imu_publisher_->publish(message);
    ++imu_publish_count_;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rk3576_footbath/stm32_serial_bridge";
    status.hardware_id = port_;
    const auto steady_now = std::chrono::steady_clock::now();
    const bool heartbeat_seen = last_heartbeat_.time_since_epoch().count() != 0;
    const double heartbeat_age = heartbeat_seen ?
      std::chrono::duration<double>(steady_now - last_heartbeat_).count() : -1.0;
    if (fd_ < 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "STM32 serial link disconnected";
    } else if (fault_flags_ != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "MCU fault flags are non-zero; STOP requested";
    } else if (!heartbeat_seen || heartbeat_age > heartbeat_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "MCU heartbeat stale; STOP requested";
    } else if (sensor_fault_flags_ != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Distance sensor model/protocol still requires validation";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Serial, odometry, IMU and range link healthy";
    }
    status.values = {
      key_value("connected", fd_ >= 0 ? "true" : "false"),
      key_value("baudrate", std::to_string(baud_)),
      key_value("heartbeat_age_s", std::to_string(heartbeat_age)),
      key_value("mcu_uptime_ms", std::to_string(mcu_uptime_ms_)),
      key_value("fault_flags", std::to_string(fault_flags_)),
      key_value("valid_sensor_flags", std::to_string(valid_sensor_flags_)),
      key_value("obstacle_flags", std::to_string(obstacle_flags_)),
      key_value("sensor_fault_flags", std::to_string(sensor_fault_flags_)),
      key_value("left_ticks", std::to_string(last_left_ticks_)),
      key_value("right_ticks", std::to_string(last_right_ticks_)),
      key_value("rx_bytes", std::to_string(rx_byte_count_)),
      key_value("rx_frames", std::to_string(rx_frame_count_)),
      key_value("odom_published", std::to_string(odom_publish_count_)),
      key_value("imu_published", std::to_string(imu_publish_count_)),
      key_value("range_published", std::to_string(range_publish_count_)),
      key_value("cmd_vel_sent", std::to_string(cmd_vel_tx_count_)),
      key_value("stop_sent", std::to_string(stop_count_)),
      key_value("crc_errors", std::to_string(parser_.crc_error_count())),
      key_value("length_errors", std::to_string(parser_.length_error_count())),
      key_value("dropped_bytes", std::to_string(parser_.dropped_byte_count())),
      key_value("payload_errors", std::to_string(payload_error_count_)),
      key_value("version_errors", std::to_string(version_error_count_)),
      key_value("unknown_messages", std::to_string(unknown_message_count_)),
      key_value("read_errors", std::to_string(read_error_count_)),
      key_value("write_errors", std::to_string(write_error_count_)),
      key_value("disconnects", std::to_string(disconnect_count_)),
      key_value("reconnect_attempts", std::to_string(reconnect_attempt_count_)),
      key_value("clamped_cmd_vel", std::to_string(clamped_cmd_count_)),
      key_value("invalid_cmd_vel", std::to_string(invalid_cmd_count_)),
    };
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  std::string port_;
  int baud_{115200};
  double reconnect_period_s_{1.0};
  double cmd_timeout_s_{0.5};
  double heartbeat_timeout_s_{2.0};
  double max_linear_mps_{1.0};
  double max_angular_rps_{1.5};
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr source_publisher_;
  bool stop_on_fault_{true};
  bool publish_tf_{true};
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string tof_left_frame_;
  std::string tof_right_frame_;
  std::string ultrasonic_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string tof_left_topic_;
  std::string tof_right_topic_;
  std::string ultrasonic_topic_;
  std::vector<double> pose_covariance_diagonal_;
  std::vector<double> twist_covariance_diagonal_;

  int fd_{-1};
  StreamParser parser_;
  uint16_t tx_sequence_{0};
  uint16_t fault_flags_{0};
  uint16_t valid_sensor_flags_{0};
  uint8_t obstacle_flags_{0};
  uint8_t sensor_fault_flags_{0};
  uint32_t mcu_uptime_ms_{0};
  int32_t last_left_ticks_{0};
  int32_t last_right_ticks_{0};
  bool received_cmd_vel_{false};
  bool timeout_stop_sent_{false};
  bool heartbeat_stale_stop_sent_{false};
  std::chrono::steady_clock::time_point last_cmd_vel_{};
  std::chrono::steady_clock::time_point last_heartbeat_{};
  std::chrono::steady_clock::time_point connected_since_{};
  std::chrono::steady_clock::time_point next_reconnect_{};

  uint64_t rx_byte_count_{0};
  uint64_t rx_frame_count_{0};
  uint64_t odom_publish_count_{0};
  uint64_t imu_publish_count_{0};
  uint64_t range_publish_count_{0};
  uint64_t cmd_vel_tx_count_{0};
  uint64_t stop_count_{0};
  uint64_t payload_error_count_{0};
  uint64_t version_error_count_{0};
  uint64_t unknown_message_count_{0};
  uint64_t read_error_count_{0};
  uint64_t write_error_count_{0};
  uint64_t disconnect_count_{0};
  uint64_t reconnect_attempt_count_{0};
  uint64_t clamped_cmd_count_{0};
  uint64_t invalid_cmd_count_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_right_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ultrasonic_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace rk3576_footbath_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rk3576_footbath_base::Stm32SerialBridge>());
  rclcpp::shutdown();
  return 0;
}
