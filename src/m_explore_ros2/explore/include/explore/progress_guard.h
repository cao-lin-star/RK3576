#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace explore {
// Independent from action results: success and target changes are not motion.
class ProgressGuard {
  struct Visit { double x,y,until; };
  std::vector<Visit> visits_;
  bool initialized_{false};
  double x_{0},y_{0},progress_at_{0};
public:
  void reset(double now,double x,double y) {
    initialized_=true; x_=x; y_=y; progress_at_=now;
  }
  bool stalled(double now,double x,double y,double timeout) {
    if(!initialized_ || now<progress_at_ || std::hypot(x-x_,y-y_)>=.05)
      reset(now,x,y);
    return now-progress_at_>=timeout;
  }
  void succeeded(double now,double x,double y) {
    visits_.erase(std::remove_if(visits_.begin(),visits_.end(),
      [now](const Visit &v){return v.until<=now;}),visits_.end());
    visits_.push_back({x,y,now+60.});
  }
  bool cooling(double now,double x,double y) const {
    for(const auto &v:visits_)
      if(now<v.until && std::hypot(x-v.x,y-v.y)<=.35) return true;
    return false;
  }
};
}
