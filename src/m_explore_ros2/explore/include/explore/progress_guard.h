#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <deque>

namespace explore {
inline bool blockedExecution(bool had_path,double elapsed,double displacement) {
  return had_path && std::isfinite(elapsed) && std::isfinite(displacement) &&
      elapsed>=5. && displacement<.10;
}

// Independent from action results: success and target changes are not motion.
class ProgressGuard {
  struct Visit { double x,y,until; };
  std::vector<Visit> visits_;
  struct Failure { double x,y,until; unsigned count; };
  std::vector<Failure> failures_;
  std::vector<Failure> reached_;

  struct Sample { double t,x,y; };
  std::deque<Sample> motion_;
  bool initialized_{false};
  double x_{0},y_{0},progress_at_{0};
public:
  void reset(double now,double x,double y) {
    initialized_=true; x_=x; y_=y; progress_at_=now;
    motion_.clear(); motion_.push_back({now,x,y});
  }
  bool stalled(double now,double x,double y,double timeout) {
    if(!std::isfinite(now) || !std::isfinite(x) || !std::isfinite(y)) return true;
    if(!initialized_ || now<progress_at_) reset(now,x,y);
    if(std::hypot(x-x_,y-y_)>=.05) { x_=x; y_=y; progress_at_=now; }
    if(motion_.empty() || now-motion_.back().t>=.2) motion_.push_back({now,x,y});
    while(motion_.size()>2 && now-motion_[1].t>=timeout) motion_.pop_front();
    if(now-progress_at_>=timeout) return true;
    if(motion_.size()<2 || now-motion_.front().t<timeout) return false;
    double travelled=0., extent=0.;
    for(size_t i=1;i<motion_.size();++i) {
      travelled+=std::hypot(motion_[i].x-motion_[i-1].x,motion_[i].y-motion_[i-1].y);
      extent=std::max(extent,std::hypot(motion_[i].x-motion_[0].x,motion_[i].y-motion_[0].y));
    }
    const double net=std::hypot(x-motion_[0].x,y-motion_[0].y);
    return travelled>=.4 && extent<=.6 && net<travelled*.2 && net<.20;
  }
  void defer(double now,double x,double y) {
    // Local immobility is not evidence that a distant frontier is unreachable.
    visits_.erase(std::remove_if(visits_.begin(),visits_.end(),
      [now](const Visit &v){return v.until<=now;}),visits_.end());
    visits_.push_back({x,y,now+60.});
  }
  void suppress(double now,double x,double y) {
    for (auto &v:failures_) {
      if (std::hypot(x-v.x,y-v.y)<=.65) {
        // One cancellation may be reported by both explorer and supervisor.
        if (now < v.until) return;
        ++v.count; v.until=now+180.; return;
      }
    }
    failures_.push_back({x,y,now+180.,1});
  }
  bool exhausted(double x,double y) const {
    for (const auto &v:failures_)
      if (v.count>=2 && std::hypot(x-v.x,y-v.y)<=.65) return true;
    return false;
  }
  bool had_failures() const { return !failures_.empty(); }
  void clearFailures() { failures_.clear(); }
  void retryNow(double now,double x,double y) {
    // Only when no alternative frontier exists. Finite attempt counts remain.
    for (auto &v:failures_)
      if (v.count<2 && std::hypot(x-v.x,y-v.y)<=.65) v.until=now;
    for (auto &v:visits_)
      if (std::hypot(x-v.x,y-v.y)<=.35) v.until=now;
  }
  bool visited(double x,double y) const {
    for (const auto &v:reached_)
      if (v.count>=2 && std::hypot(x-v.x,y-v.y)<=.35) return true;
    return false;
  }
  bool temporary(double now,double x,double y) const {
    return !exhausted(x,y) && !visited(x,y) && cooling(now,x,y);
  }

  void succeeded(double now,double x,double y) {
    bool matched=false;
    for (auto &v:reached_) {
      if (std::hypot(x-v.x,y-v.y)<=.35) { ++v.count; matched=true; break; }
    }
    if (!matched) reached_.push_back({x,y,0.,1});
    visits_.erase(std::remove_if(visits_.begin(),visits_.end(),
      [now](const Visit &v){return v.until<=now;}),visits_.end());
    visits_.push_back({x,y,now+60.});
  }
  bool cooling(double now,double x,double y) const {
    if (visited(x,y)) return true;
    for(const auto &v:failures_)
      if ((v.count>=2 || now<v.until) && std::hypot(x-v.x,y-v.y)<=.65) return true;
    for(const auto &v:visits_)
      if(now<v.until && std::hypot(x-v.x,y-v.y)<=.35) return true;
    return false;
  }
};
}
