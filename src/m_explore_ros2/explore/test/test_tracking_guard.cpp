#include <cassert>
#include <cmath>
#include "explore/tracking_guard.h"
#include "explore/planning_guard.h"
int main() {
  explore::DepartureFailureGuard departure;
  assert(!departure.failed(0,0,0));
  assert(!departure.failed(3,.01,0));
  assert(departure.failed(6,0,.01)); // different goals, same blocked place
  assert(departure.failed(9,0,0)); // pause/heading changes do not refill budget
  assert(!departure.failed(10,.3,0)); // real displacement starts a new episode
  assert(!departure.failed(0,.3,0)); // backwards clock starts a new window

  for(int mode=0;mode<4;++mode) {
    std::vector<int8_t> grid(10000,0);
    if(mode==1)grid[5050]=100;
    if(mode==2)grid.assign(10000,-1);
    if(mode==3) {
      grid.assign(10000,100);
      for(int y=46;y<=54;++y)for(int x=46;x<=54;++x)grid[y*100+x]=0;
    }
    explore::ReachableGrid full(100,100,.05,-2.5,-2.5,0,grid,0,0);
    explore::ReachableGrid quick(100,100,.05,-2.5,-2.5,0,grid,0,0,true);
    assert(full.hasManeuverSpace()==quick.hasManeuverSpace());
    assert(quick.hasManeuverSpace()==(mode==0));
  }
  explore::TrackingGuard g;
  g.path({{0,0},{3,0}});
  for(int i=0;i<16;++i) assert(!g.stalled(i*.5,0,(i%2)*.15));
  assert(g.stalled(8.,0,0)); // lateral wobble is not route progress
  g.path({{0,0},{3,0}});
  for(int i=0;i<60;++i) assert(!g.stalled(i*.5,i*.015,0)); // normal slow travel
  g.path({{0,0},{3,0}});
  for(int i=0;i<16;++i) assert(!g.stalled(i*.5,(i%2)*.06,0));
  assert(g.stalled(8.,0,0)); // back/forth cannot renew progress
  g.path({{0,0},{1,0},{1,1}});
  for(int i=0;i<=50;++i) assert(!g.stalled(i*.2,std::min(1.,i*.04),std::max(0.,i*.04-1.)));
  g.clear();assert(!g.stalled(100,0,0)); // pause/cancel clears old evidence
  explore::RouteRetryGuard retry;
  assert(retry.allow(0,0,3,0));
  assert(!retry.allow(.01,0,3.1,0)); // jitter/centroid drift cannot refill budget
  assert(!retry.allow(0,0,3,0)); // health pause does not refill budget
  assert(retry.allow(.7,0,3,0)); // physically left the previous blocked place
  assert(retry.allow(.7,0,4,1)); // distinct target
}
