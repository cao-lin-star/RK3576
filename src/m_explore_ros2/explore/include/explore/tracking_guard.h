#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace explore {
// Progress along the committed route, not wheel travel or changes of heading.
class TrackingGuard {
  std::vector<std::pair<double,double>> path_;
  bool initialized_{false};
  double best_{0.}, projection_{0.}, progress_at_{0.};
public:
  void clear() { path_.clear(); initialized_=false; }
  void path(std::vector<std::pair<double,double>> points) {
    path_=std::move(points); initialized_=false;
  }
  bool stalled(double now,double x,double y) {
    if(path_.size()<2 || !std::isfinite(now) || !std::isfinite(x) || !std::isfinite(y)) return false;
    double arc=0., nearest=std::numeric_limits<double>::infinity(), projected=0.;
    for(size_t i=1;i<path_.size();++i) {
      auto a=path_[i-1], b=path_[i];
      double dx=b.first-a.first,dy=b.second-a.second,len=std::hypot(dx,dy);
      if(len<1e-8) continue;
      const double t=std::max(0.,std::min(1.,((x-a.first)*dx+(y-a.second)*dy)/(len*len)));
      const double along=arc+t*len;
      const double distance=std::hypot(x-a.first-t*dx,y-a.second-t*dy);
      // Avoid jumping to a distant leg where the path crosses itself.
      if((!initialized_ || (along>=projection_-.30 && along<=projection_+.60)) && distance<nearest) {
        nearest=distance; projected=along;
      }
      arc+=len;
    }
    // Lateral oscillation and revisiting an old maximum cannot reset the clock.
    if(!initialized_ || now<progress_at_) {
      initialized_=true;best_=projected;projection_=projected;progress_at_=now;
    } else if(std::isfinite(nearest) && nearest<=.35) {
      projection_=projected;
      if(projected>=best_+.08) { best_=projected;progress_at_=now; }
    }
    return now-progress_at_>=8.;
  }
};

// One stopped replan at the same physical place/goal. Health pauses do not
// replenish the budget. Ordinary travel or a distinct target starts a new one.
class RouteRetryGuard {
  bool initialized_{false};
  double x_{0.},y_{0.},gx_{0.},gy_{0.};
public:
  bool allow(double x,double y,double gx,double gy) {
    if(!initialized_ || std::hypot(x-x_,y-y_)>=.60 || std::hypot(gx-gx_,gy-gy_)>.65) {
      initialized_=true;x_=x;y_=y;gx_=gx;gy_=gy;return true;
    }
    return false;
  }
};
// Failed action results are counted across targets. Only real displacement
// starts a new episode; neither pauses nor heading changes refill the budget.
class DepartureFailureGuard {
  bool initialized_{false};
  double x_{0},y_{0},since_{0};
  unsigned failures_{0};
public:
  bool failed(double now,double x,double y) {
    if(!initialized_ || now<since_ || std::hypot(x-x_,y-y_)>=.25) {
      initialized_=true;x_=x;y_=y;since_=now;failures_=0;
    }
    ++failures_;
    return failures_>=3 && now-since_>=6.;
  }
};

}
