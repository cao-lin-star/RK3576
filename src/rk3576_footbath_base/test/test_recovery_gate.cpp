#include <gtest/gtest.h>
#include "rk3576_footbath_base/recovery_gate.hpp"
TEST(RecoveryGate, RequiresNeutralAndHealthyWindow) {
  footbath::RecoveryGate g;
  EXPECT_TRUE(g.blocked(0));g.heartbeat(0,0);
  g.heartbeat(1,0); EXPECT_TRUE(g.blocked(1));
  g.neutral_sent(); EXPECT_FALSE(g.blocked(1));
  g.heartbeat(1.1,1024);EXPECT_TRUE(g.blocked(1.1));
  g.neutral_sent();g.heartbeat(2,0);EXPECT_TRUE(g.blocked(2));
  g.neutral_sent();g.heartbeat(2.9,0);EXPECT_TRUE(g.blocked(2.9));
  g.heartbeat(3,0);EXPECT_FALSE(g.blocked(3));
}
TEST(RecoveryGate, StaleHeartbeatAndNewStopRearm) {
  footbath::RecoveryGate g;
  g.heartbeat(0,0);g.neutral_sent();g.heartbeat(1,0);
  EXPECT_FALSE(g.blocked(1));EXPECT_TRUE(g.blocked(3));
  g.heartbeat(3,0);g.neutral_sent();EXPECT_TRUE(g.blocked(3));
  g.heartbeat(4,0);EXPECT_FALSE(g.blocked(4));
  g.arm();EXPECT_TRUE(g.blocked(4));
  g.heartbeat(4.1,0);g.neutral_sent();g.heartbeat(5.2,0);
  EXPECT_FALSE(g.blocked(5.2));
  g.heartbeat(5.3,256);EXPECT_TRUE(g.blocked(5.3));
}
