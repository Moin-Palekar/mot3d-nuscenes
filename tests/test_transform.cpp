#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/geometry/transform.h"

namespace mot3d {
namespace {

const double kPi = std::acos(-1.0);
constexpr double kTol = 1e-9;

void ExpectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected) {
  EXPECT_NEAR(actual.x(), expected.x(), kTol);
  EXPECT_NEAR(actual.y(), expected.y(), kTol);
  EXPECT_NEAR(actual.z(), expected.z(), kTol);
}

// A pose with a turn that isn't only about the vertical axis, so the tests
// exercise full 3D rotation, not just yaw.
Pose MakeTiltedPose() {
  const Eigen::Quaterniond q = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()) *
                               Eigen::Quaterniond::Identity();
  return MakePose({q.w(), q.x(), q.y(), q.z()}, {12.0, -4.0, 1.5});
}

TEST(Transform, YawSurvivesRoundTrip) {
  for (const double yaw : {0.0, 0.5, -2.0, kPi / 2.0, -kPi / 2.0, kPi - 1e-6}) {
    EXPECT_NEAR(YawFromQuaternion(QuaternionFromYaw(yaw)), yaw, kTol) << "yaw " << yaw;
  }
}

TEST(Transform, NegatedQuaternionGivesSameYaw) {
  const QuaternionWxyz q = QuaternionFromYaw(1.2);
  const QuaternionWxyz neg = {-q[0], -q[1], -q[2], -q[3]};
  EXPECT_NEAR(YawFromQuaternion(neg), 1.2, kTol);
}

TEST(Transform, MakePoseReadsWxyzOrder) {
  const double half = 0.3;
  const Pose pose = MakePose({std::cos(half), 0.0, 0.0, std::sin(half)}, {1.0, 2.0, 3.0});
  EXPECT_NEAR(pose.rotation.w(), std::cos(half), kTol);
  EXPECT_NEAR(pose.rotation.z(), std::sin(half), kTol);
  ExpectVectorNear(pose.translation, {1.0, 2.0, 3.0});
}

TEST(Transform, MakePoseNormalizesRotation) {
  const Pose pose = MakePose({2.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  EXPECT_NEAR(pose.rotation.norm(), 1.0, kTol);
}

TEST(Transform, ApplyRotatesThenShifts) {
  // Turn left 90 degrees, then move 10 m east.
  const Pose pose = MakePose(QuaternionFromYaw(kPi / 2.0), {10.0, 0.0, 0.0});
  // A point 1 m ahead (east) ends up 1 m north of the shifted origin.
  ExpectVectorNear(Apply(pose, {1.0, 0.0, 0.0}), {10.0, 1.0, 0.0});
}

TEST(Transform, InverseUndoesApply) {
  const Pose pose = MakeTiltedPose();
  const Eigen::Vector3d point(3.0, -7.0, 0.5);
  ExpectVectorNear(Apply(Inverse(pose), Apply(pose, point)), point);
}

TEST(Transform, ComposeMatchesApplyingInTurn) {
  const Pose outer = MakeTiltedPose();
  const Pose inner = MakePose(QuaternionFromYaw(-0.4), {1.0, 2.0, 0.0});
  const Eigen::Vector3d point(0.5, 0.25, -1.0);
  ExpectVectorNear(Apply(Compose(outer, inner), point), Apply(outer, Apply(inner, point)));
}

}  // namespace
}  // namespace mot3d