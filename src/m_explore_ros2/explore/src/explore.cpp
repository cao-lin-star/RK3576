/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <stdexcept>
#include <thread>
#include <sstream>
#include <iomanip>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  planner_grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap",rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if(msg->header.frame_id!=costmap_client_.getGlobalFrameID() || msg->info.resolution<=0 ||
           msg->data.size()!=size_t(msg->info.width)*msg->info.height || msg->data.size()>4000000)return;
        planner_grid_=std::make_shared<nav_msgs::msg::OccupancyGrid>(*msg);
        planner_grid_at_=this->now().seconds();
      });
  planner_updates_sub_=create_subscription<map_msgs::msg::OccupancyGridUpdate>(
      "/global_costmap/costmap_updates",10,
      [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
        if(!planner_grid_ || msg->header.frame_id!=planner_grid_->header.frame_id ||
           size_t(msg->x)+msg->width>planner_grid_->info.width ||
           size_t(msg->y)+msg->height>planner_grid_->info.height ||
           msg->data.size()!=size_t(msg->width)*msg->height)return;
        for(unsigned y=0;y<msg->height;++y)
          std::copy_n(msg->data.begin()+size_t(y)*msg->width,msg->width,
            planner_grid_->data.begin()+size_t(y+msg->y)*planner_grid_->info.width+msg->x);
        planner_grid_at_=this->now().seconds();
      });
  require_health_lease_=declare_parameter<bool>("require_health_lease", false);
  health_lease_sub_=create_subscription<std_msgs::msg::Bool>("/safety/auto_motion_lease",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        health_lease_=msg->data; health_lease_at_=this->now().seconds();
        if(!msg->data) completion_.reset();
      });
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<double>("blacklist_radius", 0.65);
  this->declare_parameter<std::string>("navigation_behavior_tree", "");

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("blacklist_radius", blacklist_radius_);
  this->get_parameter("navigation_behavior_tree", navigation_behavior_tree_);

  if (blacklist_radius_ <= 0.0) {
    throw std::invalid_argument("blacklist_radius must be positive");
  }

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  view_pub_ = this->create_publisher<std_msgs::msg::String>("/explore/view", 10);
  start_space_pub_=create_publisher<std_msgs::msg::Bool>("/explore/start_space",10);
  view_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
    monitorTracking(); publishView();
  });

  plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        if (!goal_active_ || paused_ || msg->poses.empty() ||
            msg->header.frame_id!=costmap_client_.getGlobalFrameID() ||
            rclcpp::Time(msg->header.stamp).seconds()<goal_sent_at_) return;
        const auto &end=msg->poses.back().pose.position;
        if (std::hypot(end.x-prev_goal_.x,end.y-prev_goal_.y)<=.5 && !goal_had_path_) {
          goal_had_path_=true;
          std::vector<std::pair<double,double>> points;
          for(const auto& p:msg->poses) points.emplace_back(p.pose.position.x,p.pose.position.y);
          tracking_guard_.path(std::move(points));
        }
      });

  controller_phase_sub_ = create_subscription<std_msgs::msg::String>(
      "/explore/controller_phase", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::istringstream in(msg->data);
        std::string phase, frame; double stamp, plan_stamp, x, y;
        if (!(in >> phase >> stamp >> plan_stamp >> x >> y >> frame)) return;
        const double now = this->now().seconds();
        if (!std::isfinite(stamp) || !std::isfinite(plan_stamp) ||
            !std::isfinite(x) || !std::isfinite(y) || now-stamp<0. || now-stamp>1. ||
            plan_stamp<goal_sent_at_ || frame!=costmap_client_.getGlobalFrameID() ||
            std::hypot(x-prev_goal_.x,y-prev_goal_.y)>.5) return;
        // Collision checks keep running while navigation is canceled, so the
        // supervisor cannot mistake global connectivity for a safe departure.
        if (phase=="departure_clear" || phase=="departure_blocked") {
          departure_clear_=phase=="departure_clear";departure_checked_at_=stamp;
          return;
        }
        if(!goal_active_ || paused_) return;
        if(phase=="alignment_failed") {
          goal_alignment_failed_=true;alignment_recovery_required_=true;
          departure_clear_=false;
          return;
        }
        if (phase=="following" && !goal_started_following_) {
          bool valid=false;
          const auto pose=costmap_client_.getRobotPose(&valid);
          if (!valid) return;
          goal_started_following_=true;
          following_started_at_=now;
          goal_start_=pose.position;
          progress_guard_.reset(now,pose.position.x,pose.position.y);
          last_progress_=this->now();
        }
      });

  // Subscription to resume or stop exploration
  hazard_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/safety/hazard_zones", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
        if (msg->header.frame_id == costmap_client_.getGlobalFrameID()) {
          if(hazard_zones_!=msg->poses) {
            completion_.reset();
            // Changed physical/manual barriers can reopen previously failed routes.
            progress_guard_.clearFailures(); frontier_blacklist_.clear(); failed_view_.clear();
          }
          hazard_zones_ = msg->poses;
        }
      });
  suppress_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/defer_current", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) { progress_guard_.defer(this->now().seconds(), prev_goal_.x, prev_goal_.y); publishView(); }
      });
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // A supervisor may require Nav2 and home capture before the first goal.
  if (this->declare_parameter<bool>("start_paused", false)) {
    stop();
    return;
  }
  // Start exploration right away when no external startup gate is requested.
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

bool Explore::readStartSpace(const geometry_msgs::msg::Pose& pose)
{
  const double age=this->now().seconds()-planner_grid_at_;
  if(!planner_grid_ || age<0. || age>3.) return false;
  const auto& m=*planner_grid_;const auto& q=m.info.origin.orientation;
  const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  ReachableGrid grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,
                    m.info.origin.position.y,yaw,m.data,pose.position.x,pose.position.y,true);
  reachable_space_=grid.hasManeuverSpace();
  if(alignment_recovery_required_) {
    const double check_age=this->now().seconds()-departure_checked_at_;
    reachable_space_=reachable_space_ && departure_clear_ && check_age>=0. && check_age<=1.;
  }
  return true;
}

void Explore::monitorTracking()
{
  bool valid=false;const auto pose=costmap_client_.getRobotPose(&valid);
  std_msgs::msg::Bool space;
  space.data=valid && costmap_client_.mapFresh() && readStartSpace(pose) && reachable_space_;
  start_space_pub_->publish(space);
  const double now=this->now().seconds();
  if(paused_ || !goal_active_ || !goal_started_following_ || !navigation_goal_handle_ ||
     recovery_cancel_pending_ || !valid || !costmap_client_.mapFresh() ||
     (require_health_lease_ && (!health_lease_ || now-health_lease_at_<0. || now-health_lease_at_>1.))) return;
  if(tracking_guard_.stalled(now,pose.position.x,pose.position.y)) {
    recovery_cancel_pending_=true;recovery_cancel_at_=now;
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
    auto status=explore_lite_msgs::msg::ExploreStatus();status.status="reevaluating_frontier";
    status_pub_->publish(status);
    RCLCPP_WARN(logger_,"No forward route progress for 8s; cancel and settle before route re-evaluation");
  }
}

void Explore::makePlan()
{
  if (paused_) {
    return;
  }
  auto planning_status = explore_lite_msgs::msg::ExploreStatus();
  planning_status.status = (recovery_cancel_pending_ || execution_failure_pending_) ? "reevaluating_frontier" : this->now().seconds() < nav_recovery_until_ ? "navigation_recovery" :
      (goal_active_ ? (goal_started_following_ ? "following_frontier" : "aligning_frontier") : "selecting_frontier");
  status_pub_->publish(planning_status);
  // find frontiers
  bool pose_valid=false;
  auto pose = costmap_client_.getRobotPose(&pose_valid);
  const double now=this->now().seconds();
  if(!pose_valid || !costmap_client_.mapFresh() ||
      (require_health_lease_ && (!health_lease_ || now-health_lease_at_>1. || now<health_lease_at_))) {
    completion_.reset();
    planning_status.status="waiting_exploration_health"; status_pub_->publish(planning_status);
    return;
  }
  if (recovery_cancel_pending_) {
    if (this->now().seconds() - recovery_cancel_at_ > 10.0) {
      stop(false);
      auto status = explore_lite_msgs::msg::ExploreStatus();
      status.status = "exploration_blocked";
      status_pub_->publish(status);
      RCLCPP_ERROR(logger_, "Recovery cancellation did not terminate in 10s; operator review required");
    }
    return;  // Never overlap a replacement goal with an unconfirmed cancellation.
  }
  planning_failures_.observe(this->now().seconds(),pose.position.x,pose.position.y);
  const bool motion_stalled = progress_guard_.stalled(
      this->now().seconds(), pose.position.x, pose.position.y, progress_timeout_);
  if (goal_active_ && goal_started_following_ && motion_stalled && !resuming_) {
    stop(false);  // Cancel first; supervisor must acquire motion ownership.
    auto status = explore_lite_msgs::msg::ExploreStatus();
    status.status = "escape_requested";
    status_pub_->publish(status);
    RCLCPP_WARN(logger_, "Stalled/oscillating: pause frontier goals for supervised retreat");
    return;
  }
  // Keep a valid goal until Nav2 finishes/recovery times out. Small changes in
  // frontier centroids must not repeatedly preempt and reset Nav2 progress.
  if (goal_active_ && !resuming_) return;
  if(execution_failure_pending_) {
    if(now<execution_retry_at_) return;  // Let near-obstacle supervision acquire ownership first.
    if(!readStartSpace(pose)) {
      completion_.reset();planning_status.status="waiting_costmap";status_pub_->publish(planning_status);return;
    }
    if(!reachable_space_) {
      planningFailure(pose,"Current departure space blocked; retain target, do not blacklist it");
      return;
    }
    const auto& m=*planner_grid_; const auto& q=m.info.origin.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    ReachableGrid grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,
                      m.info.origin.position.y,yaw,m.data,pose.position.x,pose.position.y);
    double x=0.,y=0.;
    if(!goalOnBlacklist(prev_goal_) && grid.approach(prev_goal_.x,prev_goal_.y,
        pose.position.x,pose.position.y,x,y) &&
        (retain_recovery_target_ || route_retry_guard_.allow(pose.position.x,pose.position.y,prev_goal_.x,prev_goal_.y))) {
      retain_recovery_target_=false;
      execution_failure_pending_=false;resuming_=false;
      auto target=prev_goal_;target.x=x;target.y=y;prev_goal_=target;
      RCLCPP_WARN(logger_,"Stopped route retry: departure space clear, replan toward the same frontier");
      sendFrontier(target,pose);return;
    }
    execution_failure_pending_=false;retain_recovery_target_=false;
    suppressGoal(now,prev_goal_.x,prev_goal_.y);
    RCLCPP_WARN(logger_,"Departure space clear but target route unavailable after bounded retry; select another frontier");
  }
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  CompletionGuard::Regions regions;
  for(const auto& f:frontiers) regions.emplace_back(f.centroid.x,f.centroid.y);
  const auto raw_count=frontiers.size();
  if(!filterApproaches(frontiers,pose)) { completion_.reset(); return; }
  if(!reachable_space_) {
    completion_.reset();
    planningFailure(pose,"Robot cell blocked or known reachable pocket too small");
    return;
  }
  if(frontiers.empty()) {
    filtered_unreachable_=raw_count>0;
    confirmCompletion(regions);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    const auto retry = std::find_if(frontiers.begin(), frontiers.end(),
        [this](const frontier_exploration::Frontier &f) {
          // A physical hazard or permanent exclusion is not a cooldown.
          if (!progress_guard_.temporary(this->now().seconds(),f.centroid.x,f.centroid.y)) return false;
          for (const auto &z:hazard_zones_)
            if (std::hypot(f.centroid.x-z.position.x,f.centroid.y-z.position.y)<=z.position.z+.30) return false;
          return true;
        });
    if (retry != frontiers.end()) {
      completion_.reset();
      const double now=this->now().seconds();
      if (retry_wait_at_<0.) retry_wait_at_=now;
      if (now-retry_wait_at_>=2.) {
        progress_guard_.retryNow(now,retry->centroid.x,retry->centroid.y);
        retry_wait_at_=-1.;
      }
      auto status=explore_lite_msgs::msg::ExploreStatus();
      status.status="waiting_frontier_retry"; status_pub_->publish(status);
    } else confirmCompletion(regions);
    return;
  }
  completion_.reset(); retry_wait_at_=-1.;
  // A short health pause must not turn into a change of exploration intent.
  // Revalidate the old region against the current costmap/blacklist first.
  if(resuming_ && goal_sent_at_>0.) {
    auto preferred=std::min_element(frontiers.begin(),frontiers.end(),[this](const auto& a,const auto& b) {
      return std::hypot(a.centroid.x-prev_goal_.x,a.centroid.y-prev_goal_.y)<
             std::hypot(b.centroid.x-prev_goal_.x,b.centroid.y-prev_goal_.y);
    });
    if(preferred!=frontiers.end() && !goalOnBlacklist(preferred->centroid) &&
        std::hypot(preferred->centroid.x-prev_goal_.x,preferred->centroid.y-prev_goal_.y)<=.65)
      frontier=preferred;
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ - frontier->min_distance >= 0.05) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (goal_active_ && this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_) && !resuming_) {
    addToBlacklist(target_position, "no exploration progress");
    goal_active_ = false;
    if (navigation_goal_handle_) {
      move_base_client_->async_cancel_goal(navigation_goal_handle_);
    }
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }

  sendFrontier(target_position,pose);
}

void Explore::sendFrontier(const geometry_msgs::msg::Point& target_position,
                           const geometry_msgs::msg::Pose& pose)
{
  tracking_guard_.clear();
  last_progress_=this->now();
  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  const double approach_yaw = std::atan2(target_position.y - pose.position.y, target_position.x - pose.position.x);
  goal.pose.pose.orientation.z = std::sin(approach_yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(approach_yaw * 0.5);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();
  goal.behavior_tree = navigation_behavior_tree_;

  goal_sent_at_ = this->now().seconds();
  goal_start_=pose.position;
  goal_had_path_=false;
  goal_started_following_=false;goal_alignment_failed_=false;
  // Goal replacement is not physical progress: keep the shared motion window.
  navigation_goal_handle_.reset();
  goal_active_ = true;
  const auto generation = ++request_generation_;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  nav_recovery_count_ = 0;
  send_goal_options.feedback_callback =
      [this, generation](NavigationGoalHandle::SharedPtr,
          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
        if (paused_ || generation != request_generation_) return;
        if (feedback->number_of_recoveries > nav_recovery_count_) {
          nav_recovery_count_ = feedback->number_of_recoveries;
          nav_recovery_until_ = this->now().seconds() + 3.;
        }
      };
  send_goal_options.goal_response_callback =
      [this, generation](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (paused_ || generation != request_generation_) {
          if (goal_handle) {
            move_base_client_->async_cancel_goal(goal_handle);
          }
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
          stop(false);
          auto status=explore_lite_msgs::msg::ExploreStatus();
          status.status="exploration_blocked"; status_pub_->publish(status);
        } else {
          navigation_goal_handle_ = goal_handle;
          active_goal_id_ = goal_handle->get_goal_id();
          publishView();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.result_callback =
      [this, generation,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        if (paused_ || generation != request_generation_) {
          return;
        }
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::confirmCompletion(const CompletionGuard::Regions& regions)
{
  if(completion_.observe(this->now().seconds(),true,regions,
                         std::max(7.5,2./planner_frequency_))) {
    completeReachable();
  } else {
    auto status=explore_lite_msgs::msg::ExploreStatus();
    status.status="confirming_exploration_complete"; status_pub_->publish(status);
  }
}

void Explore::completeReachable()
{
  const bool blocked=filtered_unreachable_ || progress_guard_.had_failures() || !frontier_blacklist_.empty() || !hazard_zones_.empty();
  stop(true);
  auto status=explore_lite_msgs::msg::ExploreStatus();
  status.status=blocked ? "exploration_complete_with_unreachable" :
      explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
  status_pub_->publish(status);
}

void Explore::suppressGoal(double now, double x, double y)
{
  progress_guard_.suppress(now, x, y);
  failed_view_.erase(std::remove_if(failed_view_.begin(), failed_view_.end(),
      [now,x,y](const FailedView &v) { return v.until<=now || std::hypot(v.x-x,v.y-y)<=.65; }), failed_view_.end());
  if (failed_view_.size()>=128) failed_view_.erase(failed_view_.begin());
  failed_view_.push_back({x,y,now+180.});
}

void Explore::publishView()
{
  if (!view_pub_) return;
  const double now = this->now().seconds();
  failed_view_.erase(std::remove_if(failed_view_.begin(),failed_view_.end(),
      [this,now](const FailedView &v) {return v.until<=now && !progress_guard_.exhausted(v.x,v.y);}),failed_view_.end());
  std::ostringstream out;
  out << std::setprecision(16) << "{\"stamp\":" << now << ",\"goal\":";
  if (!paused_ && goal_active_ && navigation_goal_handle_ &&
      costmap_client_.getGlobalFrameID()=="map") {
    out << "{\"x\":" << prev_goal_.x << ",\"y\":" << prev_goal_.y
        << ",\"sent_at\":" << goal_sent_at_ << ",\"id\":\""
        << rclcpp_action::to_string(active_goal_id_) << "\"}";
  } else out << "null";
  out << ",\"unreachable\":[";
  bool first = true;
  for (const auto &v:failed_view_) {
    if (!first) out << ',';
    first = false;
    out << "{\"x\":" << v.x << ",\"y\":" << v.y
        << ",\"radius\":0.65,\"remaining_s\":" << (progress_guard_.exhausted(v.x,v.y) ? -1. : v.until-now) << '}';
  }
  out << "]}";
  std_msgs::msg::String msg; msg.data=out.str(); view_pub_->publish(msg);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  if (progress_guard_.cooling(this->now().seconds(), goal.x, goal.y)) return true;
  for (const auto &zone : hazard_zones_) {
    if (std::hypot(goal.x-zone.position.x, goal.y-zone.position.y)
        <= zone.position.z + 0.30) return true;
  }
  for (const auto& frontier_goal : frontier_blacklist_) {
    const double x_diff = goal.x - frontier_goal.x;
    const double y_diff = goal.y - frontier_goal.y;
    if (std::hypot(x_diff, y_diff) <= blacklist_radius_) {
      return true;
    }
  }
  return false;
}

void Explore::addToBlacklist(const geometry_msgs::msg::Point& goal,
                             const char* reason)
{
  if (!goalOnBlacklist(goal)) {
    frontier_blacklist_.push_back(goal);
  }
  RCLCPP_WARN(
      logger_, "Frontier (%.2f, %.2f) blocked for %.2f m: %s",
      goal.x, goal.y, blacklist_radius_, reason);
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  navigation_goal_handle_.reset();
  goal_active_ = false;
  if (recovery_cancel_pending_) {
    recovery_cancel_pending_ = false;
    const auto pose = costmap_client_.getRobotPose();
    progress_guard_.reset(this->now().seconds(), pose.position.x, pose.position.y);
    execution_failure_pending_=true;execution_retry_at_=this->now().seconds()+1.;
    RCLCPP_INFO(logger_, "Tracking stopped and cancellation acknowledged; re-evaluate departure space and original target");
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      progress_guard_.succeeded(this->now().seconds(), frontier_goal.x, frontier_goal.y);
      RCLCPP_INFO(logger_, "Reached frontier (%.2f, %.2f); exclude 0.35m neighborhood for 60s",
                  frontier_goal.x, frontier_goal.y);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      {
        bool valid=false;const auto pose=costmap_client_.getRobotPose(&valid);
        const bool repeated=valid && departure_failures_.failed(this->now().seconds(),pose.position.x,pose.position.y);
        if(goal_alignment_failed_ || repeated) {
          requestDepartureEscape(goal_alignment_failed_ ?
            "Initial alignment blocked; retain frontier and acquire supervised exit" :
            "Repeated execution failures at the same position across goals; acquire supervised exit");
          return;
        }
      }
      // Failure at the departure pose is not evidence against distant frontiers.
      // Re-evaluate fresh health/local space after zero-speed settling first.
      execution_failure_pending_=true;execution_retry_at_=this->now().seconds()+1.;
      completion_.reset();
      RCLCPP_WARN(logger_,"Navigation failed; stop and re-evaluate departure space before retrying or changing frontier");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  // The existing planning timer selects the next frontier; do not recurse
  // from success callbacks or bypass planner_frequency.
}

void Explore::requestDepartureEscape(const char* reason)
{
  retain_recovery_target_=true;
  stop(false); // Called only after the active action has terminated.
  auto status=explore_lite_msgs::msg::ExploreStatus();
  status.status="escape_requested";status_pub_->publish(status);
  RCLCPP_WARN(logger_,"%s",reason);
}

bool Explore::planningFailure(const geometry_msgs::msg::Pose& pose,const char* reason)
{
  auto status=explore_lite_msgs::msg::ExploreStatus();
  if(planning_failures_.failed(this->now().seconds(),pose.position.x,pose.position.y)) {
    // Hand over only to the existing supervised exit, never command motion here.
    stop(false);
    status.status="escape_requested";status_pub_->publish(status);
    RCLCPP_WARN(logger_,"Cross-goal planning stalled: %s; request supervised exit",reason);
    return true;
  }
  status.status="waiting_reachable_frontier";status_pub_->publish(status);
  RCLCPP_WARN(logger_,"Frontier planning unavailable: %s",reason);
  return false;
}

bool Explore::filterApproaches(std::vector<frontier_exploration::Frontier>& frontiers,
                              const geometry_msgs::msg::Pose& pose)
{
  if(!planner_grid_ || this->now().seconds()-planner_grid_at_>3.) {
    auto status=explore_lite_msgs::msg::ExploreStatus();
    status.status="waiting_costmap";status_pub_->publish(status);
    return false; // Missing data is not evidence of completion or permission to retreat.
  }
  const auto& m=*planner_grid_;const auto& q=m.info.origin.orientation;
  double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  ReachableGrid grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,
                    m.info.origin.position.y,yaw,m.data,pose.position.x,pose.position.y);
  reachable_space_=grid.hasManeuverSpace();
  frontiers.erase(std::remove_if(frontiers.begin(),frontiers.end(),
    [&](frontier_exploration::Frontier& f) {
      double x=0.,y=0.;
      bool found=grid.approach(f.centroid.x,f.centroid.y,pose.position.x,pose.position.y,x,y);
      if(!found) {
        const size_t step=std::max<size_t>(1,(f.points.size()+63)/64);
        for(size_t i=0;i<f.points.size();i+=step)
          if(grid.approach(f.points[i].x,f.points[i].y,pose.position.x,pose.position.y,x,y)) {found=true;break;}
      }
      if(found) {f.centroid.x=x;f.centroid.y=y;}
      return !found;
    }),frontiers.end());
  return true;
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  paused_ = true;
  completion_.reset();
  recovery_cancel_pending_ = false;
  execution_failure_pending_=false;tracking_guard_.clear();
  ++request_generation_;
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  // Cancel only our own goal: a supervisor may already be returning home.
  if (navigation_goal_handle_) {
    move_base_client_->async_cancel_goal(navigation_goal_handle_);
    navigation_goal_handle_.reset();
  }
  exploring_timer_->cancel();
  publishView();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  if (!paused_) {
    return;
  }
  paused_ = false;
  recovery_attempts_ = 0;
  resuming_ = true;
  const auto pose = costmap_client_.getRobotPose();
  progress_guard_.reset(this->now().seconds(), pose.position.x, pose.position.y);
  planning_failures_.reset(this->now().seconds(),pose.position.x,pose.position.y);
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // A blocked departure is not evidence that the original frontier is bad.
  if(retain_recovery_target_) {
    execution_failure_pending_=true;execution_retry_at_=this->now().seconds();
  }
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
