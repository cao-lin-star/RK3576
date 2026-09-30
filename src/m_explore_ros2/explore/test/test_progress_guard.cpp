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
  // Oscillation exceeds the old 5 cm reset threshold, but is not useful progress.
  explore::ProgressGuard oscillation;
  oscillation.reset(0,0,0);
  bool detected=false;
  for(int i=1;i<=60;++i) detected |= oscillation.stalled(i,i%2 ? .08 : 0.,0.,30.);
  assert(detected);
  explore::ProgressGuard slow;
  slow.reset(0,0,0);
  for(int i=1;i<=100;++i) assert(!slow.stalled(i,i*.01,0.,30.));
  slow.suppress(100,2.,2.);
  assert(slow.cooling(200,2.1,2.));
  assert(!slow.cooling(281,2.1,2.));
  explore::ProgressGuard retries;
  retries.suppress(0,1,1);
  retries.suppress(.1,1,1); // duplicate cancellation must not spend two attempts
  assert(!retries.exhausted(1,1));
  assert(retries.temporary(100,1,1));
  assert(!retries.cooling(181,1,1));
  retries.suppress(181,1.05,1);
  assert(retries.exhausted(1,1));
  assert(retries.cooling(10000,1,1));
  assert(!retries.temporary(10000,1,1));
  assert(!retries.cooling(182,3,3)); // another frontier remains eligible
  explore::ProgressGuard completed;
  completed.succeeded(1,1,1);
  assert(completed.temporary(2,1,1));
  assert(!completed.cooling(62,1,1));
  completed.succeeded(62,1,1);
  assert(completed.visited(1,1));
  assert(!completed.temporary(200,1,1));
  assert(completed.cooling(200,1,1)); // persistent stale frontier cannot loop forever
  assert(!explore::blockedExecution(false,20,0)); // no path: select another
  assert(!explore::blockedExecution(true,2,0));   // no premature retreat
  assert(!explore::blockedExecution(true,20,.5));// progressing: try another target
  assert(explore::blockedExecution(true,10,.03));// trapped: supervised safe exit
  explore::ProgressGuard last_remaining;
  last_remaining.suppress(0,2,2);
  last_remaining.retryNow(3,2,2);
  assert(!last_remaining.cooling(3,2,2));
  last_remaining.suppress(3,2,2);
  assert(last_remaining.exhausted(2,2));
  assert(last_remaining.exhausted(2.5,2)); // centroid movement shares the attempt cap
  assert(!last_remaining.exhausted(2.8,2)); // distinct region remains available
  last_remaining.retryNow(4,2,2);
  assert(last_remaining.cooling(4,2,2)); // explicit retry never bypasses attempt cap
  last_remaining.clearFailures(); // obstacle edit can reopen a route
  assert(!last_remaining.cooling(5,2,2));
}
