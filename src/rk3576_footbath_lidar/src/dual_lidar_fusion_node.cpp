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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rk3576_footbath_lidar/dual_lidar_fusion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rk3576_footbath_lidar
{

class DualLidarFusionNode final : public rclcpp::Node
{
public:
  DualLidarFusionNode()
  : Node("dual_lidar_mapping_fusion"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    high_topic_ = declare_parameter<std::string>("topics.high", "/scan_high");
    low_topic_ = declare_parameter<std::string>("topics.low", "/scan_low_front");
    output_topic_ = declare_parameter<std::string>("topics.output", "/scan_mapping_fused");
    diagnostics_topic_ = declare_parameter<std::string>("topics.diagnostics", "/diagnostics");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    max_pair_delta_s_ = declare_parameter<double>("max_pair_delta_s", 0.060);
    pair_wait_timeout_s_ = declare_parameter<double>("pair_wait_timeout_s", 0.080);
    high_stale_timeout_s_ = declare_parameter<double>("high_stale_timeout_s", 0.150);
    low_stale_timeout_s_ = declare_parameter<double>("low_stale_timeout_s", 0.150);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.050);
    diagnostic_period_s_ = declare_parameter<double>("diagnostic_period_s", 1.0);

    if (high_topic_.empty() || low_topic_.empty() || output_topic_.empty() ||
      diagnostics_topic_.empty() || fixed_frame_.empty() || max_pair_delta_s_ <= 0.0 ||
      high_stale_timeout_s_ <= 0.0 || low_stale_timeout_s_ <= 0.0 ||
      pair_wait_timeout_s_ < max_pair_delta_s_ ||
      transform_timeout_s_ < 0.0 || diagnostic_period_s_ <= 0.0)
    {
      throw std::invalid_argument("dual lidar fusion parameters are invalid");
    }

    fused_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    low_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      low_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DualLidarFusionNode::on_low_scan, this, std::placeholders::_1));
    high_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      high_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DualLidarFusionNode::on_high_scan, this, std::placeholders::_1));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostic_period_s_),
      std::bind(&DualLidarFusionNode::publish_diagnostics, this));
    pairing_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&DualLidarFusionNode::flush_pending_high_scans, this));

    RCLCPP_INFO(
      get_logger(),
      "Dual-lidar mapping fusion ready: high=%s low=%s output=%s fixed_frame=%s pair<=%.3fs",
      high_topic_.c_str(), low_topic_.c_str(), output_topic_.c_str(), fixed_frame_.c_str(),
      max_pair_delta_s_);
  }

private:
  struct PendingHighScan
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr message;
    rclcpp::Time receive_time;
  };

  static diagnostic_msgs::msg::KeyValue value(const std::string & key, const std::string & data)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = data;
    return item;
  }

  void on_low_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    latest_low_scan_ = message;
    latest_low_consumed_ = false;
    last_low_receive_time_ = now();
    ++low_scan_count_;
    try_pair_pending_high_scans();
  }

  void publish_high_only(
    const sensor_msgs::msg::LaserScan & high_scan,
    const std::string & reason)
  {
    fused_publisher_->publish(high_scan);
    ++fallback_count_;
    low_used_last_output_ = false;
    last_result_ = reason;
    last_low_finite_points_ = 0U;
    last_projected_points_ = 0U;
    last_bins_updated_ = 0U;
  }

  void fuse_pair(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & high_scan,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & low_scan)
  {
    const rclcpp::Time high_stamp(high_scan->header.stamp, get_clock()->get_clock_type());
    const rclcpp::Time low_stamp(low_scan->header.stamp, get_clock()->get_clock_type());
    const double pair_delta_s = std::abs((high_stamp - low_stamp).seconds());
    last_pair_delta_s_ = pair_delta_s;
    max_observed_pair_delta_s_ = std::max(max_observed_pair_delta_s_, pair_delta_s);
    if (!std::isfinite(pair_delta_s) || pair_delta_s > max_pair_delta_s_) {
      ++pair_reject_count_;
      publish_high_only(*high_scan, "high-only: scan timestamp delta exceeded limit");
      return;
    }
    if (high_scan->header.frame_id.empty() || low_scan->header.frame_id.empty()) {
      ++transform_error_count_;
      publish_high_only(*high_scan, "high-only: lidar frame_id is empty");
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        high_scan->header.frame_id, high_stamp,
        low_scan->header.frame_id, low_stamp,
        fixed_frame_, tf2::durationFromSec(transform_timeout_s_));
      const auto & rotation = transform.transform.rotation;
      const double low_to_high_yaw = std::atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      const PlanarTransform low_to_high{
        transform.transform.translation.x,
        transform.transform.translation.y,
        low_to_high_yaw};
      FusionStatistics statistics;
      auto fused = fuse_laser_scans(*high_scan, *low_scan, low_to_high, &statistics);
      fused_publisher_->publish(fused);
      ++fused_count_;
      low_used_last_output_ = true;
      last_result_ = "fused";
      last_low_finite_points_ = statistics.low_finite_points;
      last_projected_points_ = statistics.projected_points;
      last_bins_updated_ = statistics.bins_updated;
    } catch (const tf2::TransformException & error) {
      ++transform_error_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Time-compensated low-to-high transform unavailable: %s", error.what());
      publish_high_only(*high_scan, "high-only: transform unavailable");
    } catch (const std::invalid_argument & error) {
      ++invalid_scan_count_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      publish_high_only(*high_scan, "high-only: invalid scan metadata");
    }
  }

  void try_pair_pending_high_scans()
  {
    if (!latest_low_scan_ || latest_low_consumed_) {
      return;
    }

    const rclcpp::Time low_stamp(
      latest_low_scan_->header.stamp, get_clock()->get_clock_type());
    while (!pending_high_scans_.empty()) {
      const auto pending = pending_high_scans_.front();
      const rclcpp::Time high_stamp(
        pending.message->header.stamp, get_clock()->get_clock_type());
      const double signed_delta_s = (low_stamp - high_stamp).seconds();
      const double pair_delta_s = std::abs(signed_delta_s);
      if (std::isfinite(pair_delta_s) && pair_delta_s <= max_pair_delta_s_) {
        pending_high_scans_.pop_front();
        latest_low_consumed_ = true;
        fuse_pair(pending.message, latest_low_scan_);
        return;
      }
      if (!std::isfinite(pair_delta_s) || signed_delta_s > max_pair_delta_s_) {
        pending_high_scans_.pop_front();
        ++pair_reject_count_;
        publish_high_only(
          *pending.message, "high-only: no low scan inside pairing window");
        continue;
      }
      return;
    }
  }

  void flush_pending_high_scans()
  {
    const auto stamp = now();
    while (!pending_high_scans_.empty()) {
      const auto & pending = pending_high_scans_.front();
      if ((stamp - pending.receive_time).seconds() < pair_wait_timeout_s_) {
        return;
      }
      const auto expired = pending;
      pending_high_scans_.pop_front();
      ++pair_reject_count_;
      publish_high_only(*expired.message, "high-only: low scan pairing timeout");
    }
  }

  void on_high_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr high_scan)
  {
    const auto receive_time = now();
    last_high_receive_time_ = receive_time;
    ++high_scan_count_;

    if (latest_low_scan_ && !latest_low_consumed_) {
      const rclcpp::Time high_stamp(
        high_scan->header.stamp, get_clock()->get_clock_type());
      const rclcpp::Time low_stamp(
        latest_low_scan_->header.stamp, get_clock()->get_clock_type());
      const double pair_delta_s = std::abs((high_stamp - low_stamp).seconds());
      if (std::isfinite(pair_delta_s) && pair_delta_s <= max_pair_delta_s_) {
        latest_low_consumed_ = true;
        fuse_pair(high_scan, latest_low_scan_);
        return;
      }
    }

    pending_high_scans_.push_back(PendingHighScan{high_scan, receive_time});
    constexpr std::size_t kMaximumPendingHighScans = 4U;
    if (pending_high_scans_.size() > kMaximumPendingHighScans) {
      const auto overflow = pending_high_scans_.front();
      pending_high_scans_.pop_front();
      ++pair_reject_count_;
      publish_high_only(*overflow.message, "high-only: pairing queue overflow");
    }
  }

  void publish_diagnostics()
  {
    const auto stamp = now();
    const double high_age_s = high_scan_count_ == 0U ?
      std::numeric_limits<double>::infinity() : (stamp - last_high_receive_time_).seconds();
    const double low_age_s = low_scan_count_ == 0U ?
      std::numeric_limits<double>::infinity() : (stamp - last_low_receive_time_).seconds();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rk3576_footbath/dual_lidar_mapping_fusion";
    status.hardware_id = "rplidar_c1_high+rplidar_c1_low";
    if (high_age_s > high_stale_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "high lidar stale; fused mapping input unavailable";
    } else if (low_age_s > low_stale_timeout_s_ || !low_used_last_output_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = last_result_;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "time-compensated high/low fusion active";
    }
    status.values = {
      value("last_result", last_result_),
      value("high_age_s", std::to_string(high_age_s)),
      value("low_age_s", std::to_string(low_age_s)),
      value("last_pair_delta_s", std::to_string(last_pair_delta_s_)),
      value("max_observed_pair_delta_s", std::to_string(max_observed_pair_delta_s_)),
      value("pending_high_scans", std::to_string(pending_high_scans_.size())),
      value("high_scan_count", std::to_string(high_scan_count_)),
      value("low_scan_count", std::to_string(low_scan_count_)),
      value("fused_count", std::to_string(fused_count_)),
      value("fallback_count", std::to_string(fallback_count_)),
      value("pair_reject_count", std::to_string(pair_reject_count_)),
      value("transform_error_count", std::to_string(transform_error_count_)),
      value("invalid_scan_count", std::to_string(invalid_scan_count_)),
      value("last_low_finite_points", std::to_string(last_low_finite_points_)),
      value("last_projected_points", std::to_string(last_projected_points_)),
      value("last_bins_updated", std::to_string(last_bins_updated_))};

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string high_topic_;
  std::string low_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string fixed_frame_;
  double max_pair_delta_s_{0.060};
  double high_stale_timeout_s_{0.150};
  double low_stale_timeout_s_{0.150};
  double pair_wait_timeout_s_{0.080};
  double transform_timeout_s_{0.050};
  double diagnostic_period_s_{1.0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_low_scan_;
  rclcpp::Time last_high_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_low_receive_time_{0, 0, RCL_ROS_TIME};
  std::deque<PendingHighScan> pending_high_scans_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr high_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr low_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::string last_result_{"waiting for scans"};
  double last_pair_delta_s_{0.0};
  rclcpp::TimerBase::SharedPtr pairing_timer_;
  double max_observed_pair_delta_s_{0.0};
  std::size_t last_low_finite_points_{0U};
  std::size_t last_projected_points_{0U};
  std::size_t last_bins_updated_{0U};
  std::uint64_t high_scan_count_{0U};
  std::uint64_t low_scan_count_{0U};
  std::uint64_t fused_count_{0U};
  std::uint64_t fallback_count_{0U};
  std::uint64_t pair_reject_count_{0U};
  std::uint64_t transform_error_count_{0U};
  std::uint64_t invalid_scan_count_{0U};
  bool latest_low_consumed_{true};
  bool low_used_last_output_{false};
};

}  // namespace rk3576_footbath_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rk3576_footbath_lidar::DualLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
