#include <cassert>
#include "explore/progress_guard.h"
int main() {
  explore::ProgressGuard g;
  g.reset(0,0,0);
  g.succeeded(1,.2,.1);
  assert(g.cooling(2,.2,.1));
  assert(g.cooling(2,.4,.1));
  assert(!g.cooling(2,1.,1.));
  assert(!g.cooling(61,.2,.1));
  // Success cannot reset real-motion timeout, nor can small localization noise.
  assert(!g.stalled(10,.001,.001,20));
  g.succeeded(19,.21,.1);
  assert(g.stalled(20,.002,.002,20));
  assert(!g.stalled(21,.06,0,20));
  assert(!g.stalled(30,.06,0,20));
  assert(g.stalled(41,.06,0,20));
}
