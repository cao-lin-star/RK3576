#include "explore/tracking_guard.h"
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
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/progress_guard.h>
#include <explore/planning_guard.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();
  void monitorTracking();
  bool readStartSpace(const geometry_msgs::msg::Pose& pose);
  void sendFrontier(const geometry_msgs::msg::Point& target_position,
                    const geometry_msgs::msg::Pose& pose);
  TrackingGuard tracking_guard_;
  RouteRetryGuard route_retry_guard_;
  DepartureFailureGuard departure_failures_;
  bool goal_alignment_failed_{false};
  bool alignment_recovery_required_{false};
  bool retain_recovery_target_{false};
  bool departure_clear_{false};
  double departure_checked_at_{-1.};
  void requestDepartureEscape(const char* reason);
  bool execution_failure_pending_{false};
  double execution_retry_at_{0.};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_space_pub_;
  void publishView();
  void suppressGoal(double now, double x, double y);
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr view_pub_;
  rclcpp::TimerBase::SharedPtr view_timer_;
  struct FailedView { double x,y,until; };
  std::vector<FailedView> failed_view_;
  double goal_sent_at_{0.};
  bool goal_had_path_{false};
  bool goal_started_following_{false};
  double following_started_at_{0.};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_phase_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
  geometry_msgs::msg::Point goal_start_;
  CompletionGuard completion_;
  bool reachable_space_{false};
  bool filtered_unreachable_{false};
  bool require_health_lease_{false};
  bool health_lease_{false};
  double health_lease_at_{-1.};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_lease_sub_;
  void confirmCompletion(const CompletionGuard::Regions& regions);
  double retry_wait_at_{-1.};
  void completeReachable();


  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  void addToBlacklist(const geometry_msgs::msg::Point& goal,
                      const char* reason);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suppress_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  ProgressGuard progress_guard_;
  PlanningFailureGuard planning_failures_;
  nav_msgs::msg::OccupancyGrid::SharedPtr planner_grid_;
  double planner_grid_at_{-1.};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr planner_grid_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr planner_updates_sub_;
  bool planningFailure(const geometry_msgs::msg::Pose& pose,const char* reason);
  bool filterApproaches(std::vector<frontier_exploration::Frontier>& frontiers,
                        const geometry_msgs::msg::Pose& pose);
  bool recovery_cancel_pending_{false};
  double recovery_cancel_at_{0};
  unsigned recovery_attempts_{0};
  double nav_recovery_until_{0.};
  int nav_recovery_count_{0};
  std::vector<geometry_msgs::msg::Pose> hazard_zones_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr hazard_subscription_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  double blacklist_radius_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  std::string navigation_behavior_tree_;
  bool resuming_ = false;
  bool goal_active_{false};
  bool paused_{false};
  std::size_t request_generation_{0};
  rclcpp_action::GoalUUID active_goal_id_;
};
}  // namespace explore

#endif
