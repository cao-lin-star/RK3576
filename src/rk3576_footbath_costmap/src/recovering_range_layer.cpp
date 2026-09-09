#include "nav2_costmap_2d/range_sensor_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "recovery.hpp"

namespace rk3576_footbath_costmap {
class RecoveringRangeLayer : public nav2_costmap_2d::RangeSensorLayer {
 public:
  void updateBounds(double x, double y, double yaw, double *min_x, double *min_y,
                    double *max_x, double *max_y) override {
    nav2_costmap_2d::RangeSensorLayer::updateBounds(x,y,yaw,min_x,min_y,max_x,max_y);
    // Humble marks current_ false on timeout but only resets it after resetMaps.
    // buffered_readings_ counts measurements actually accepted AND transformed,
    // not merely arriving messages. Preserve timeout and invalid-data behavior.
    const bool before=current_;
    current_=recovered_current(current_,enabled_,buffered_readings_);
    if (!before && current_) {
      RCLCPP_INFO(logger_, "Range layer recovered after valid transformed measurement");
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(rk3576_footbath_costmap::RecoveringRangeLayer,nav2_costmap_2d::Layer)
