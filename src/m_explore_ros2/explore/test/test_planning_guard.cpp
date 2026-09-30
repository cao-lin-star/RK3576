#include <explore/planning_guard.h>
#include <cassert>
#include <iostream>
using explore::PlanningFailureGuard;
using explore::ReachableGrid;
int main() {
  PlanningFailureGuard g;
  assert(!g.failed(100,0,0)); assert(!g.failed(105,.02,0));
  assert(g.failed(110,0,0)); // Different goals at the same physical position.
  g.reset(111,0,0); assert(!g.failed(111,0,0));
  assert(!g.failed(116,.1,0)); assert(g.failed(121,0,0)); // Small oscillation.
  g.observe(122,.3,0); assert(!g.failed(123,.3,0));
  assert(!g.failed(128,.3,0)); assert(g.failed(133,.3,0));
  g.reset(140,0,0);assert(!g.failed(140,0,0));assert(!g.failed(140.1,0,0));
  assert(!g.failed(151,0,0));assert(g.failed(152,0,0)); // Duplicate callbacks do not count.
  assert(!g.failed(1,0,0)); // Clock rollback.
  double x=0,y=0;
  std::vector<int8_t> cells(400,0);
  cells[10*20+10]=-1;
  ReachableGrid unknown(20,20,.1,0,0,0,cells,.25,1.05);
  assert(unknown.approach(1.05,1.05,.25,1.05,x,y));
  assert(std::hypot(x-1.05,y-1.05)>.01 && std::hypot(x-1.05,y-1.05)<=.35);
  cells[10*20+10]=100;
  ReachableGrid occupied(20,20,.1,0,0,0,cells,.25,1.05);
  assert(occupied.approach(1.05,1.05,.25,1.05,x,y));
  assert(std::hypot(x-1.05,y-1.05)>.01);
  for(int row=0;row<20;++row)cells[row*20+8]=99;
  ReachableGrid split(20,20,.1,0,0,0,cells,.25,1.05);
  assert(!split.approach(1.55,1.05,.25,1.05,x,y)); // Disconnected pocket.
  assert(split.approach(.55,1.05,.25,1.05,x,y));
  cells[10*20+2]=99;
  ReachableGrid blocked(20,20,.1,0,0,0,cells,.25,1.05);
  assert(!blocked.approach(.55,1.05,.25,1.05,x,y));
  std::vector<int8_t> free(400,0);
  ReachableGrid rotated(20,20,.1,5,5,1.5707963267948966,free,4.75,5.25);
  assert(rotated.approach(4.45,5.55,4.75,5.25,x,y));
  assert(std::hypot(x-4.45,y-5.55)<.01);
  ReachableGrid tiny(1,1,.1,0,0,0,std::vector<int8_t>{0},.05,.05);
  assert(!tiny.approach(.05,.05,.05,.05,x,y));
  ReachableGrid invalid(20,20,.1,0,0,0,std::vector<int8_t>{},.05,.05);
  assert(!invalid.approach(.55,.55,.05,.05,x,y));
  assert(split.hasManeuverSpace());
  assert(!blocked.hasManeuverSpace());
  assert(!tiny.hasManeuverSpace());
  assert(!invalid.hasManeuverSpace());
  explore::CompletionGuard finish;
  using Regions=explore::CompletionGuard::Regions;
  Regions glass{{2.,2.},{4.,2.}};
  for(double t=0;t<30;t+=5) assert(!finish.observe(t,true,glass));
  assert(finish.observe(30,true,{{2.2,2.},{4.1,2.}})); // centroid jitter
  assert(!finish.observe(31,false,glass)); // sensor/TF fault revokes completion
  assert(!finish.observe(40,true,glass));
  assert(!finish.observe(45,true,{{2.,2.},{5.,2.}})); // new frontier restarts
  for(double t=50;t<75;t+=5) assert(!finish.observe(t,true,{{2.,2.},{5.,2.}}));
  assert(finish.observe(75,true,{{2.,2.},{5.,2.}}));
  assert(!finish.observe(100,true,glass)); // missed checks do not count as health
  assert(!finish.observe(90,true,glass)); // clock rollback
  finish.reset();
  for(int i=0;i<100;++i) assert(!finish.observe(100,true,glass)); // duplicate callbacks
  finish.reset();
  for(double t=0;t<30;t+=5) assert(!finish.observe(t,true,{}));
  assert(finish.observe(30,true,{})); // raw empty map gets same stable window
  assert(!finish.observe(31,true,glass)); // newly exposed frontier revokes empty
  std::cout << "Planning failure guard and reachable grid scenarios passed\n";
}
