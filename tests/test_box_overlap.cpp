#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/geometry/box_overlap.h"

namespace mot3d {
namespace {

const double kPi = std::acos(-1.0);
constexpr double kTol = 1e-9;

OrientedBox MakeBox(double x, double y, double z, double width, double length, double height,
                    double yaw) {
  OrientedBox box;
  box.center = Eigen::Vector3d(x, y, z);
  box.size = Eigen::Vector3d(width, length, height);
  box.yaw = yaw;
  return box;
}

// A car-sized box at the origin: 2 m wide, 4 m long, 2 m tall, facing east.
// Its volume is 2 x 4 x 2 = 16.
OrientedBox Car() {
  return MakeBox(0.0, 0.0, 0.0, 2.0, 4.0, 2.0, 0.0);
}

TEST(BoxOverlap, IdenticalBoxesGiveOne) {
  EXPECT_NEAR(BoxIou(Car(), Car()), 1.0, kTol);
}

TEST(BoxOverlap, FarApartGiveZero) {
  EXPECT_NEAR(BoxIou(Car(), MakeBox(10.0, 0.0, 0.0, 2.0, 4.0, 2.0, 0.0)), 0.0, kTol);
}

TEST(BoxOverlap, ShiftedHalfwayAlongLength) {
  // Shifted 2 m east: the footprints share a 2 x 2 square, so the overlap is
  // 2 x 2 x 2 = 8. Union = 16 + 16 - 8 = 24. IoU = 8 / 24 = 1/3.
  EXPECT_NEAR(BoxIou(Car(), MakeBox(2.0, 0.0, 0.0, 2.0, 4.0, 2.0, 0.0)), 1.0 / 3.0, kTol);
}

TEST(BoxOverlap, TurnedNinetyDegreesCrossesItself) {
  // Turned to face north, it crosses the original like a plus sign. The shared
  // footprint is again a 2 x 2 square, so IoU = 8 / 24 = 1/3.
  EXPECT_NEAR(BoxIou(Car(), MakeBox(0.0, 0.0, 0.0, 2.0, 4.0, 2.0, kPi / 2.0)), 1.0 / 3.0, kTol);
}

TEST(BoxOverlap, OffsetInHeight) {
  // Raised 1.5 m: the height ranges [-1, 1] and [0.5, 2.5] share 0.5 m.
  // Overlap = 2 x 4 x 0.5 = 4. Union = 16 + 16 - 4 = 28. IoU = 4 / 28 = 1/7.
  EXPECT_NEAR(BoxIou(Car(), MakeBox(0.0, 0.0, 1.5, 2.0, 4.0, 2.0, 0.0)), 1.0 / 7.0, kTol);
}

TEST(BoxOverlap, FacingTheOtherWayIsTheSameBox) {
  // A box looks identical from both ends, so turning it 180 degrees changes nothing.
  EXPECT_NEAR(BoxIou(Car(), MakeBox(0.0, 0.0, 0.0, 2.0, 4.0, 2.0, kPi)), 1.0, kTol);
}

TEST(BoxOverlap, SquareTurnedFortyFiveDegrees) {
  // Two 2 x 2 x 2 cubes, one turned 45 degrees. Their shared footprint is a
  // regular octagon, and the IoU works out to exactly 1 / sqrt(2).
  const OrientedBox square = MakeBox(0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 0.0);
  const OrientedBox turned = MakeBox(0.0, 0.0, 0.0, 2.0, 2.0, 2.0, kPi / 4.0);
  EXPECT_NEAR(BoxIou(square, turned), 1.0 / std::sqrt(2.0), kTol);
}

TEST(BoxOverlap, OrderDoesNotMatter) {
  const OrientedBox a = MakeBox(1.0, 2.0, 0.3, 1.9, 4.6, 1.7, 0.4);
  const OrientedBox b = MakeBox(1.5, 2.4, 0.1, 2.0, 4.2, 1.5, -0.3);
  EXPECT_NEAR(BoxIou(a, b), BoxIou(b, a), kTol);
}

}  // namespace
}  // namespace mot3d