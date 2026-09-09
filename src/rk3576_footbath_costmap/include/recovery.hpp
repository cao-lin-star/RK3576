#pragma once
namespace rk3576_footbath_costmap {
inline bool recovered_current(bool previous, bool enabled, unsigned processed) {
  return previous || (enabled && processed > 0);
}
}
