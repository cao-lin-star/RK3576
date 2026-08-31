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
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rk3576_footbath_lidar/low_lidar_angular_filter.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace rk3576_footbath_lidar
{

class LowLidarAngularFilterNode final : public rclcpp::Node
{
public:
  LowLidarAngularFilterNode()
  : Node("low_lidar_angular_filter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/scan_low_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/scan_low_front");
    valid_angle_min_ = declare_parameter<double>("valid_angle_min", 1.9634954085);
    valid_angle_max_ = declare_parameter<double>("valid_angle_max", -1.9634954085);
    sensor_x_m_ = declare_parameter<double>("sensor_x_m", 0.10);
    sensor_y_m_ = declare_parameter<double>("sensor_y_m", 0.0);
    sensor_yaw_rad_ = declare_parameter<double>("sensor_yaw_rad", 3.1415926536);
    self_filter_radius_m_ = declare_parameter<double>("self_filter_radius_m", 0.225);
    constexpr double kPi = 3.14159265358979323846;
    if (!std::isfinite(valid_angle_min_) || !std::isfinite(valid_angle_max_) ||
      valid_angle_min_ < -kPi || valid_angle_min_ > kPi ||
      valid_angle_max_ < -kPi || valid_angle_max_ > kPi ||
      self_filter_radius_m_ < 0.0)
    {
      throw std::invalid_argument("low lidar filter parameters are invalid");
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LowLidarAngularFilterNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering %s -> %s; angle [%.6f, %.6f] rad%s, sensor=(%.3f, %.3f, %.3f), self radius=%.3f m",
      input_topic.c_str(), output_topic.c_str(), valid_angle_min_, valid_angle_max_,
      valid_angle_min_ > valid_angle_max_ ? " (wraps at +/-pi)" : "",
      sensor_x_m_, sensor_y_m_, sensor_yaw_rad_, self_filter_radius_m_);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    try {
      publisher_->publish(
        filter_scan_by_angle(
          *message, valid_angle_min_, valid_angle_max_, sensor_x_m_, sensor_y_m_,
          sensor_yaw_rad_, self_filter_radius_m_));
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  double valid_angle_min_;
  double valid_angle_max_;
  double sensor_x_m_;
  double sensor_y_m_;
  double sensor_yaw_rad_;
  double self_filter_radius_m_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace rk3576_footbath_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rk3576_footbath_lidar::LowLidarAngularFilterNode>());
  rclcpp::shutdown();
  return 0;
}
