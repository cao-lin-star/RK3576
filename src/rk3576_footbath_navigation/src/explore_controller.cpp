#include <algorithm>
#include <iomanip>
#include <chrono>
#include <limits>
#include <sstream>
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "rk3576_footbath_navigation/alignment_guard.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"

namespace footbath {
class ExploreController : public nav2_rotation_shim_controller::RotationShimController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    RotationShimController::configure(parent, name, tf, costmap);
    phase_pub_ = parent.lock()->create_publisher<std_msgs::msg::String>(
      "/explore/controller_phase", rclcpp::QoS(10).reliable());
    departure_timer_=parent.lock()->create_wall_timer(std::chrono::milliseconds(200), [this]() {
      std::lock_guard<std::mutex> path_lock(departure_mutex_);
      std::lock_guard<std::mutex> parameter_lock(mutex_);
      if(!phase_pub_ || !phase_pub_->is_activated() || current_path_.poses.empty()) return;
      bool clear=false;
      try {
        geometry_msgs::msg::PoseStamped pose;
        if(!costmap_ros_->isCurrent() || !costmap_ros_->getRobotPose(pose))
          throw std::runtime_error("Departure costmap or pose unavailable");
        const auto path_robot=tf_->lookupTransform(current_path_.header.frame_id,
          costmap_ros_->getBaseFrameID(),tf2::TimePointZero);
        const auto& robot=path_robot.transform.translation;
        // Path points are in the global plan frame; the footprint stays in
        // the local costmap frame. Never compare map coordinates with odom.
        // Re-evaluate the retained path from the current pose after retreat.
        size_t nearest=0;double best=std::numeric_limits<double>::infinity();
        for(size_t i=0;i<current_path_.poses.size();++i) {
          const auto& p=current_path_.poses[i].pose.position;
          const double d=std::hypot(p.x-robot.x,p.y-robot.y);
          if(d<best) {best=d;nearest=i;}
        }
        auto sample=current_path_.poses.back();
        for(size_t i=nearest;i<current_path_.poses.size();++i) {
          const auto& p=current_path_.poses[i].pose.position;
          if(std::hypot(p.x-robot.x,p.y-robot.y)>=forward_sampling_distance_) {
            sample=current_path_.poses[i];break;
          }
        }
        sample.header=current_path_.header;sample.header.stamp=clock_->now();
        const auto target=transformPoseToBaseFrame(sample);
        const double error=std::atan2(target.position.y,target.position.x);
        checkDeparture(pose, std::abs(error)<=angular_dist_threshold_ ? 0. : error);
        clear=true;
      } catch(const std::exception&) {}
      publishPhase(clear ? "departure_clear" : "departure_blocked");
    });
  }
  void activate() override {
    RotationShimController::activate(); phase_pub_->on_activate();
  }
  void deactivate() override {
    phase_pub_->on_deactivate(); RotationShimController::deactivate();
  }
  void cleanup() override {
    departure_timer_.reset(); phase_pub_.reset(); RotationShimController::cleanup();
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> path_lock(departure_mutex_);
    const bool same = path.header == current_path_.header && path.poses == current_path_.poses;
    RotationShimController::setPlan(path);
    // Duplicate delivery must not extend an alignment deadline.
    if (!same) {stage_ = Stage::Pending; guard_.reset(); settle_at_ = -1.;}
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * checker) override {
    std::lock_guard<std::mutex> path_lock(departure_mutex_);
    std::lock_guard<std::mutex> parameter_lock(mutex_);
    if (stage_ == Stage::Failed) {
      publishPhase("alignment_failed");
      throw nav2_core::PlannerException("Initial alignment failed; select another frontier");
    }
    if (stage_ == Stage::Following) {
      publishPhase("following");
      return primary_controller_->computeVelocityCommands(pose, velocity, checker);
    }
    try {
      if (current_path_.poses.empty()) throw std::runtime_error("Empty exploration path");
      auto sample = current_path_.poses.back();
      const auto & first = current_path_.poses.front().pose.position;
      for (const auto & pt : current_path_.poses) {
        if (std::hypot(pt.pose.position.x - first.x, pt.pose.position.y - first.y) >=
          forward_sampling_distance_) {sample = pt; break;}
      }
      sample.header = current_path_.header; sample.header.stamp = clock_->now();
      const auto target = transformPoseToBaseFrame(sample);
      const double error = std::atan2(target.position.y, target.position.x);
      const double now = clock_->now().seconds();
      if (stage_ == Stage::Pending) {
        if (std::hypot(target.position.x, target.position.y) < 0.10 ||
          std::abs(error) <= angular_dist_threshold_) {
          stage_ = Stage::Following; publishPhase("following");
          return primary_controller_->computeVelocityCommands(pose, velocity, checker);
        }
        stage_ = Stage::Aligning;
      }
      const double continuous_error = guard_.update(now, error);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = pose.header; cmd.header.stamp = clock_->now();
      publishPhase("aligning");
      // Settle before handing motion to the translating controller.
      if (std::abs(continuous_error) <= angular_disengage_threshold_) {
        if (std::abs(velocity.angular.z) < 0.03 && std::abs(velocity.linear.x) < 0.02) {
          if (settle_at_ < 0.) settle_at_ = now;
          if (now - settle_at_ >= 0.3) {stage_ = Stage::Following; publishPhase("following");}
        } else {settle_at_ = -1.;}
        return cmd;
      }
      settle_at_ = -1.;
      const double target_w = std::clamp(continuous_error * 0.8,
        -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);
      const double delta = max_angular_accel_ * control_duration_;
      cmd.twist.angular.z = std::clamp(target_w, velocity.angular.z - delta, velocity.angular.z + delta);
      checkRotation(pose, cmd.twist.angular.z);
      return cmd;
    } catch (const std::exception & e) {
      // Nav2 commands zero on failure; never fall through to DWB turning.
      stage_ = Stage::Failed; publishPhase("alignment_failed");
      throw nav2_core::PlannerException(e.what());
    }
  }
private:
  std::mutex departure_mutex_;
  rclcpp::TimerBase::SharedPtr departure_timer_;
  // Probe the complete initial turn and a short forward departure using the
  // same footprint checker as motion control. This timer never sends velocity.
  void checkDeparture(const geometry_msgs::msg::PoseStamped& pose,double turn) {
    auto* map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto footprint=costmap_ros_->getRobotFootprint();
    const double yaw=tf2::getYaw(pose.pose.orientation);
    const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(turn)/.05)));
    auto check=[&](double x,double y,double a) {
      const double c=collision_checker_->footprintCostAtPose(x,y,a,footprint);
      if(c>=nav2_costmap_2d::LETHAL_OBSTACLE || c<0.) throw std::runtime_error("Departure blocked");
    };
    for(int i=0;i<=steps;++i) check(pose.pose.position.x,pose.pose.position.y,yaw+turn*i/steps);
    for(int i=1;i<=5;++i) check(pose.pose.position.x+.02*i*std::cos(yaw+turn),
      pose.pose.position.y+.02*i*std::sin(yaw+turn),yaw+turn);
  }
  enum class Stage {Pending, Aligning, Following, Failed};
  Stage stage_{Stage::Pending};
  AlignmentGuard guard_;
  double settle_at_{-1.};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr phase_pub_;
  void publishPhase(const char * phase) {
    if (current_path_.poses.empty()) return;
    const auto & p = current_path_.poses.back().pose.position;
    std::ostringstream out;
    out << std::setprecision(17) << phase << ' ' << clock_->now().seconds() << ' '
        << rclcpp::Time(current_path_.header.stamp).seconds() << ' '
        << p.x << ' ' << p.y << ' ' << current_path_.header.frame_id;
    std_msgs::msg::String msg; msg.data = out.str(); phase_pub_->publish(msg);
  }
  void checkRotation(const geometry_msgs::msg::PoseStamped & pose, double w) {
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto footprint = costmap_ros_->getRobotFootprint();
    const double yaw = tf2::getYaw(pose.pose.orientation);
    // Check the current footprint and next second for either turn direction.
    for (int i = 0; i <= 20; ++i) {
      const double cost = collision_checker_->footprintCostAtPose(
        pose.pose.position.x, pose.pose.position.y, yaw + w * i * 0.05, footprint);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost < 0.)
        throw std::runtime_error("Initial alignment footprint blocked or unknown");
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(footbath::ExploreController, nav2_core::Controller)
