#pragma once
#include <algorithm>
#include <array>
#include <utility>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace explore {
// Failure evidence belongs to a physical position, not to one action goal.
class PlanningFailureGuard {
  bool initialized_{false};
  double x_{0}, y_{0}, since_{0}, last_{-1};
  unsigned failures_{0};
public:
  void reset(double now,double x,double y) {
    initialized_=true;x_=x;y_=y;since_=now;last_=-1;failures_=0;
  }
  void observe(double now,double x,double y) {
    if(!initialized_ || now<since_ || std::hypot(x-x_,y-y_)>=.25) reset(now,x,y);
  }
  bool failed(double now,double x,double y) {
    observe(now,x,y);
    if(last_<0 || now-last_>=1.) { if(failures_==0) since_=now; ++failures_; last_=now; }
    return failures_>=3 && now-since_>=10.;
  }
};

// Occupancy-grid projection of the actual Nav2 global costmap. 99 is the
// inscribed/inflated collision cost; unknown cells are not approach goals.
class ReachableGrid {
  unsigned w_,h_;double r_,ox_,oy_,yaw_;
  std::vector<int8_t> costs_;
  std::vector<bool> reached_;
  double extent_{0.};
  bool root_free_{false};
public:
  ReachableGrid(unsigned w,unsigned h,double r,double ox,double oy,double yaw,
      const std::vector<int8_t>& costs,double x,double y,bool departure_only=false):
      w_(w),h_(h),r_(r),ox_(ox),oy_(oy),yaw_(yaw),costs_(costs),reached_(costs.size(),false) {
    if(!w||!h||!std::isfinite(r)||r<=0||costs.size()!=size_t(w)*h)return;
    int ix,iy;if(!cell(x,y,ix,iy))return;
    const auto root=size_t(iy)*w+ix;
    if(costs_[root]<0 || costs_[root]>=99)return;
    root_free_=true;
    std::deque<size_t> queue{root};reached_[root]=true;
    while(!queue.empty()) {
      auto p=queue.front();queue.pop_front();int cx=p%w,cy=p/w;
      extent_=std::max(extent_,std::hypot(cx-ix,cy-iy)*r_);
      if(departure_only && extent_>=.60) break; // heartbeat need not flood the entire map
      for(auto d:std::array<std::pair<int,int>,4>{{{-1,0},{1,0},{0,-1},{0,1}}}) {
        int nx=cx+d.first,ny=cy+d.second;
        if(nx<0||ny<0||nx>=int(w)||ny>=int(h))continue;
        size_t z=size_t(ny)*w+nx;
        if(!reached_[z]&&costs_[z]>=0&&costs_[z]<99) {reached_[z]=true;queue.push_back(z);}
      }
    }
  }
  bool hasManeuverSpace() const { return root_free_ && extent_>=.60; }
  bool cell(double x,double y,int& ix,int& iy) const {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(r_)||r_<=0||
       !std::isfinite(ox_)||!std::isfinite(oy_)||!std::isfinite(yaw_)||
       costs_.size()!=size_t(w_)*h_)return false;
    double dx=x-ox_,dy=y-oy_;
    ix=std::floor((dx*std::cos(yaw_)+dy*std::sin(yaw_))/r_);
    iy=std::floor((-dx*std::sin(yaw_)+dy*std::cos(yaw_))/r_);
    return ix>=0&&iy>=0&&ix<int(w_)&&iy<int(h_);
  }
  bool approach(double x,double y,double robot_x,double robot_y,double& out_x,double& out_y) const {
    int ix,iy;if(!cell(x,y,ix,iy))return false;
    int radius=std::ceil(.35/r_);double best=std::numeric_limits<double>::infinity();
    for(int u=std::max(0,ix-radius);u<=std::min(int(w_)-1,ix+radius);++u)
      for(int v=std::max(0,iy-radius);v<=std::min(int(h_)-1,iy+radius);++v) {
        const size_t k=size_t(v)*w_+u;if(!reached_[k])continue;
        double lx=(u+.5)*r_,ly=(v+.5)*r_;
        double wx=ox_+lx*std::cos(yaw_)-ly*std::sin(yaw_),wy=oy_+lx*std::sin(yaw_)+ly*std::cos(yaw_);
        double distance=std::hypot(wx-x,wy-y);
        if(distance>.35||std::hypot(wx-robot_x,wy-robot_y)<.15)continue;
        double score=distance+.002*costs_[k];
        if(score<best) {best=score;out_x=wx;out_y=wy;}
      }
    return std::isfinite(best);
  }
};
// Fixed region anchors tolerate centroid jitter without sliding indefinitely.
// Every sample must have fresh inputs and an untrapped robot; any interruption
// starts a new continuous confirmation window. No permanent wall is invented.
class CompletionGuard {
public:
  using Regions=std::vector<std::pair<double,double>>;
private:
  Regions anchors_;
  double since_{-1.}, last_{-1.};
  unsigned samples_{0};
  static bool covered(const Regions& a,const Regions& b) {
    return std::all_of(a.begin(),a.end(),[&](const auto& p) {
      return std::any_of(b.begin(),b.end(),[&](const auto& q) {
        return std::hypot(p.first-q.first,p.second-q.second)<=.65;
      });
    });
  }
public:
  void reset() { since_=last_=-1.; samples_=0; anchors_.clear(); }
  bool observe(double now,bool eligible,const Regions& regions,double max_gap=7.5) {
    if(!eligible || !std::isfinite(now)) { reset(); return false; }
    if(since_<0. || now<last_ || now-last_>max_gap ||
       !covered(regions,anchors_) || !covered(anchors_,regions)) {
      anchors_=regions; since_=last_=now; samples_=1; return false;
    }
    if(now-last_>=1.) { ++samples_; last_=now; }
    return samples_>=4 && now-since_>=30.;
  }
};
}  // namespace explore
