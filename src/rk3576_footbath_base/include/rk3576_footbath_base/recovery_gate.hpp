#pragma once
#include <cstdint>
namespace footbath {
// Only zero commands cross a stop barrier. Old motion is never replayed.
class RecoveryGate {
  double heartbeat_{-1}, clear_since_{-1};
  bool neutral_{false};
public:
  void arm() { clear_since_=-1; neutral_=false; }
  void heartbeat(double now, uint16_t fault) {
    if (heartbeat_<0 || now-heartbeat_>1.5 || now<heartbeat_) arm();
    heartbeat_=now;
    if (fault) arm();
    else if (clear_since_<0) clear_since_=now;
  }
  void neutral_sent() { if(clear_since_>=0) neutral_=true; }
  bool blocked(double now) {
    if(heartbeat_<0 || now-heartbeat_>1.5 || now<heartbeat_) arm();
    return clear_since_<0 || !neutral_ || now-clear_since_<1.;
  }
};
}
