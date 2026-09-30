#include <cassert>
#include <cmath>
#include "rk3576_footbath_navigation/alignment_guard.hpp"
int main() {
  footbath::AlignmentGuard g;
  assert(g.update(0.,3.13)>0.); assert(g.update(.1,-3.13)>0.);
  assert(g.update(.2,3.10)>0.);
  bool failed=false;
  try {g.update(3.2,3.13);} catch (...) {failed=true;}
  assert(failed);
  g.reset();
  for(int i=0;i<=100;++i) assert(std::abs(g.update(i*.1,-3.+i*.03))<=3.01);
  g.reset(); g.update(0.,3.);
  for(int i=1;i<=20;++i) g.update(i,3.-i*.04);
  failed=false; try {g.update(20.1,2.1);} catch (...) {failed=true;}
  assert(failed);
  g.reset();g.update(0.,.2);assert(g.update(.1,-.02)<0.);
}
