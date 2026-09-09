#include <gtest/gtest.h>
#include "recovery.hpp"
using rk3576_footbath_costmap::recovered_current;
TEST(RangeRecovery, TimeoutStaysFalseWithoutAcceptedReading) {
  EXPECT_FALSE(recovered_current(false,true,0));
}
TEST(RangeRecovery, FreshAcceptedReadingRecovers) {
  EXPECT_TRUE(recovered_current(false,true,1));
}
TEST(RangeRecovery, DisabledLayerDoesNotInventReading) {
  EXPECT_FALSE(recovered_current(false,false,1));
}
TEST(RangeRecovery, DoesNotDiscardExistingHealthyState) {
  EXPECT_TRUE(recovered_current(true,true,0));
}
