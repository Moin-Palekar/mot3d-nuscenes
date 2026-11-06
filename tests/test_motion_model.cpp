#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/tracking/motion_model.h"

namespace mot3d {
namespace {

const double kPi = std::acos(-1.0);
constexpr double kTol = 1e-9;

State MakeState(double x, double y, double z, double yaw, double speed, double yaw_rate) {
  State s;
  s << x, y, z, yaw, speed, yaw_rate;
  return s;
}

// The Jacobian worked out by brute force: nudge each input a tiny bit up and
// down, and measure how much each predicted number moves.
StateJacobian NumericJacobian(const State& state, double dt) {
  constexpr double kStep = 1e-6;
  StateJacobian jac;
  for (int i = 0; i < kStateSize; ++i) {
    State plus = state;
    State minus = state;
    plus[i] += kStep;
    minus[i] -= kStep;
    jac.col(i) = (PredictCtrv(plus, dt) - PredictCtrv(minus, dt)) / (2.0 * kStep);
  }
  return jac;
}

void ExpectJacobianNear(const StateJacobian& actual, const StateJacobian& expected) {
  for (int row = 0; row < kStateSize; ++row) {
    for (int col = 0; col < kStateSize; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), 1e-6) << "row " << row << " col " << col;
    }
  }
}

TEST(MotionModel, StraightLineMovesAlongHeading) {
  // Facing north at 10 m/s, not turning, for half a second: 5 m north.
  const State next = PredictCtrv(MakeState(1.0, 2.0, 0.5, kPi / 2.0, 10.0, 0.0), 0.5);
  EXPECT_NEAR(next[kX], 1.0, kTol);
  EXPECT_NEAR(next[kY], 7.0, kTol);
  EXPECT_NEAR(next[kYaw], kPi / 2.0, kTol);
}

TEST(MotionModel, QuarterTurnEndsOnCircle) {
  // Speed and turn rate chosen so the circle has radius 1 m and the car turns
  // 90 degrees in 1 s. Starting at the origin facing east, a left quarter
  // circle ends at (1, 1) facing north.
  const State next = PredictCtrv(MakeState(0.0, 0.0, 0.0, 0.0, kPi / 2.0, kPi / 2.0), 1.0);
  EXPECT_NEAR(next[kX], 1.0, kTol);
  EXPECT_NEAR(next[kY], 1.0, kTol);
  EXPECT_NEAR(next[kYaw], kPi / 2.0, kTol);
}

TEST(MotionModel, KeepsHeightSpeedAndYawRate) {
  const State start = MakeState(3.0, -2.0, 1.2, 0.4, 8.0, 0.3);
  const State next = PredictCtrv(start, 0.5);
  EXPECT_NEAR(next[kZ], start[kZ], kTol);
  EXPECT_NEAR(next[kSpeed], start[kSpeed], kTol);
  EXPECT_NEAR(next[kYawRate], start[kYawRate], kTol);
}

TEST(MotionModel, YawWrapsIntoRange) {
  // 3.0 + 1.0 * 0.5 = 3.5 rad, past pi, so it must wrap to 3.5 - 2 pi.
  const State next = PredictCtrv(MakeState(0.0, 0.0, 0.0, 3.0, 5.0, 1.0), 0.5);
  EXPECT_NEAR(next[kYaw], 3.5 - 2.0 * kPi, kTol);
}

TEST(MotionModel, BranchesAgreeAtThreshold) {
  // Just above and just below the turn-rate threshold (1e-4 rad/s), the
  // circle formula and the straight-line formula must give the same answer.
  const State turning = PredictCtrv(MakeState(0.0, 0.0, 0.0, 0.3, 10.0, 1.000001e-4), 1.0);
  const State straight = PredictCtrv(MakeState(0.0, 0.0, 0.0, 0.3, 10.0, 0.999999e-4), 1.0);
  EXPECT_NEAR(turning[kX], straight[kX], 1e-7);
  EXPECT_NEAR(turning[kY], straight[kY], 1e-7);
}

TEST(MotionModel, JacobianMatchesNumericWhileTurning) {
  const State state = MakeState(3.0, -2.0, 1.0, 0.4, 8.0, 0.3);
  ExpectJacobianNear(CtrvJacobian(state, 0.5), NumericJacobian(state, 0.5));
}

TEST(MotionModel, JacobianMatchesNumericWhenStraight) {
  const State state = MakeState(3.0, -2.0, 1.0, 0.4, 8.0, 0.0);
  ExpectJacobianNear(CtrvJacobian(state, 0.5), NumericJacobian(state, 0.5));
}

TEST(MotionModel, CvMovesStraightAndDropsYawRate) {
  State state;
  state << 0.0, 0.0, 1.0, 0.0, 10.0, 0.5;  // facing east, 10 m/s, turning left
  const State next = PredictCv(state, 0.5);
  EXPECT_NEAR(next[kX], 5.0, 1e-9);
  EXPECT_NEAR(next[kY], 0.0, 1e-9);
  EXPECT_NEAR(next[kZ], 1.0, 1e-9);
  EXPECT_NEAR(next[kYaw], 0.0, 1e-9);
  EXPECT_NEAR(next[kSpeed], 10.0, 1e-9);
  EXPECT_NEAR(next[kYawRate], 0.0, 1e-9);
}

TEST(MotionModel, CvMatchesCtrvWhenNotTurning) {
  State state;
  state << 3.0, -2.0, 1.0, 0.7, 8.0, 0.0;  // yaw_rate 0: driving straight
  const State cv = PredictCv(state, 0.5);
  const State ctrv = PredictCtrv(state, 0.5);
  for (int i = 0; i < kStateSize; ++i) {
    EXPECT_NEAR(cv[i], ctrv[i], 1e-9) << "index " << i;
  }
}

TEST(MotionModel, CvJacobianMatchesNumeric) {
  State state;
  state << 3.0, -2.0, 1.0, 0.7, 8.0, 0.3;
  const double dt = 0.5;
  const double eps = 1e-6;
  const StateJacobian analytic = CvJacobian(state, dt);
  for (int col = 0; col < kStateSize; ++col) {
    State up = state;
    State down = state;
    up[col] += eps;
    down[col] -= eps;
    const State slope = (PredictCv(up, dt) - PredictCv(down, dt)) / (2.0 * eps);
    for (int row = 0; row < kStateSize; ++row) {
      EXPECT_NEAR(analytic(row, col), slope[row], 1e-6) << "row " << row << " col " << col;
    }
  }
}

}  // namespace
}  // namespace mot3d