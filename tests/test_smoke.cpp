#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

// Confirms the build and test harness work before any real code exists.
TEST(Smoke, BuildSystemWorks) {
  EXPECT_EQ(1 + 1, 2);
}

// Confirms Eigen is linked and usable. A 2x2 inverse exercises enough of the
// library that a broken include path or version mismatch would fail here.
TEST(Smoke, EigenIsLinked) {
  Eigen::Matrix2d m;
  m << 2.0, 0.0, 0.0, 4.0;

  const Eigen::Matrix2d inv = m.inverse();

  EXPECT_NEAR(inv(0, 0), 0.5, 1e-12);
  EXPECT_NEAR(inv(1, 1), 0.25, 1e-12);
  EXPECT_NEAR(inv(0, 1), 0.0, 1e-12);
}

// Pins the Eigen version. If this fails, the FetchContent pin was bypassed
// and a system Eigen is being used instead — the exact local/CI skew the pin
// exists to prevent.
TEST(Smoke, EigenVersionIsPinned) {
  EXPECT_EQ(EIGEN_WORLD_VERSION, 3);
  EXPECT_EQ(EIGEN_MAJOR_VERSION, 4);
  EXPECT_EQ(EIGEN_MINOR_VERSION, 0);
}

// Confirms nlohmann/json is linked and can parse. The detection file is
// ~345 MB of JSON, so this dependency is load-bearing for the whole project.
TEST(Smoke, JsonIsLinked) {
  const auto parsed = nlohmann::json::parse(R"({"translation": [1.0, 2.0, 3.0]})");

  ASSERT_TRUE(parsed.contains("translation"));
  EXPECT_DOUBLE_EQ(parsed["translation"][1].get<double>(), 2.0);
}