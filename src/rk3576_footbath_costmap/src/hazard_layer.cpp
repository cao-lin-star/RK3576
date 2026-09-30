#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/create_publisher.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"

namespace rk3576_footbath_costmap {
// Independent layer: laser clearing cannot erase remembered cliff edges.
class HazardLayer : public nav2_costmap_2d::Layer {
  struct Zone { double x,y,r; };
  std::mutex mutex_;
  std::vector<Zone> zones_, transformed_, previous_;
  std_msgs::msg::Header received_header_, rendered_header_;
  uint64_t received_revision_{0}, rendered_revision_{0};
  bool rendered_valid_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr applied_pub_;
public:
  void onInitialize() override {
    auto node=node_.lock();
    enabled_=true; current_=true;
    // A normal publisher is intentional: this plugin has no lifecycle callback
    // forwarding. It only publishes from a completed costmap update below.
    std::string scope=node->get_namespace();
    if(scope=="/") scope.clear();
    auto parameters=node->get_node_parameters_interface();
    auto topics=node->get_node_topics_interface();
    applied_pub_=rclcpp::create_publisher<std_msgs::msg::Header>(
      parameters,topics,scope+"/hazard_zones_applied",rclcpp::QoS(1).reliable());
    rclcpp::SubscriptionOptions options;
    options.callback_group=callback_group_;
    sub_=node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/safety/hazard_zones",rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
        if(msg->header.frame_id!="map") return;
        std::vector<Zone> next;
        for(const auto &p:msg->poses) {
          if(!std::isfinite(p.position.x)||!std::isfinite(p.position.y)||
             !std::isfinite(p.position.z)||p.position.z<=0||p.position.z>.3) return;
          next.push_back({p.position.x,p.position.y,p.position.z});
        }
        std::lock_guard<std::mutex> lock(mutex_);
        zones_=std::move(next);
        received_header_=msg->header;
        ++received_revision_;
      },options);
  }
  void updateBounds(double,double,double,double *minx,double *miny,double *maxx,double *maxy) override {
    std::lock_guard<std::mutex> lock(mutex_);
    transformed_.clear();
    rendered_valid_=false;
    try {
      double x=0,y=0,yaw=0;
      if(layered_costmap_->getGlobalFrameID()!="map") {
        auto tf=tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),"map",tf2::TimePointZero);
        const auto &q=tf.transform.rotation;
        x=tf.transform.translation.x; y=tf.transform.translation.y;
        yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      }
      for(auto z:zones_) transformed_.push_back({x+std::cos(yaw)*z.x-std::sin(yaw)*z.y,
                                                y+std::sin(yaw)*z.x+std::cos(yaw)*z.y,z.r});
      current_=true;
      rendered_header_=received_header_;
      rendered_revision_=received_revision_;
      rendered_valid_=received_revision_!=0;
    } catch(const std::exception &) { current_=false; transformed_=previous_; }
    // Also revisit removed/expired regions so the master can reconstruct them.
    for(const auto &list:{previous_,transformed_}) for(auto z:list) {
      *minx=std::min(*minx,z.x-z.r); *miny=std::min(*miny,z.y-z.r);
      *maxx=std::max(*maxx,z.x+z.r); *maxy=std::max(*maxy,z.y+z.r);
    }
    previous_=transformed_;
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master,int min_i,int min_j,int max_i,int max_j) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!enabled_ || min_i>=max_i || min_j>=max_j) return;
    bool complete=true;
    for(const auto &z:transformed_) {
      if(z.x+z.r<master.getOriginX() || z.y+z.r<master.getOriginY() ||
         z.x-z.r>=master.getOriginX()+master.getSizeInCellsX()*master.getResolution() ||
         z.y-z.r>=master.getOriginY()+master.getSizeInCellsY()*master.getResolution()) continue;
      int x0,y0,x1,y1;
      master.worldToMapEnforceBounds(z.x-z.r,z.y-z.r,x0,y0);
      master.worldToMapEnforceBounds(z.x+z.r,z.y+z.r,x1,y1);
      complete=complete && min_i<=x0 && min_j<=y0 && max_i>x1 && max_j>y1;
      for(int j=std::max(y0,min_j);j<std::min(y1+1,max_j);++j)
        for(int i=std::max(x0,min_i);i<std::min(x1+1,max_i);++i) {
          double x,y; master.mapToWorld(i,j,x,y);
          if(std::hypot(x-z.x,y-z.y)<=z.r+master.getResolution()*.71)
            master.setCost(i,j,nav2_costmap_2d::LETHAL_OBSTACLE);
        }
    }
    // Bounds and costs may interleave with a subscription callback. Never
    // confirm a newly received header while rendering the previous revision,
    // nor confirm stale/fallback geometry after a failed TF transform.
    if(complete && current_ && rendered_valid_ && rendered_revision_==received_revision_) {
      applied_pub_->publish(rendered_header_);
    }
  }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    rendered_valid_=false;
    current_=false;
  } // Clear-costmap cannot remove cliff memory; require a fresh bounds/costs pass.
  bool isClearable() override { return false; }
};
}
PLUGINLIB_EXPORT_CLASS(rk3576_footbath_costmap::HazardLayer,nav2_costmap_2d::Layer)
