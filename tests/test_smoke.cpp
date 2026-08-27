#include <gtest/gtest.h>

// Confirms the build and test harness work before any real code exists.
TEST(Smoke, BuildSystemWorks) {
  EXPECT_EQ(1 + 1, 2);
}
