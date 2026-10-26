#include "mot3d/tracking/motion_model.h"

#include <cmath>

namespace mot3d {
namespace {

// Below this turn rate (rad/s) we use the straight-line formulas. The turning
// formulas divide by the turn rate, which blows up as it approaches zero.
constexpr double kMinYawRate = 1e-4;

// Wraps an angle into [-pi, pi], so the same heading always has one number.
const double kPi = std::acos(-1.0);
double NormalizeAngle(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

}  // namespace

State PredictCtrv(const State& state, double dt) {
  const double yaw = state[kYaw];
  const double v = state[kSpeed];
  const double w = state[kYawRate];

  State next = state;  // z, speed and yaw rate stay the same.
  if (std::abs(w) > kMinYawRate) {
    // Moving along a circle of radius v / w.
    next[kX] += v / w * (std::sin(yaw + w * dt) - std::sin(yaw));
    next[kY] += v / w * (std::cos(yaw) - std::cos(yaw + w * dt));
  } else {
    // Practically not turning: a straight line, plus the first small effect
    // of the turn rate (the leading terms of the circle formulas above).
    const double c0 = std::cos(yaw);
    const double s0 = std::sin(yaw);
    next[kX] += v * c0 * dt - 0.5 * v * s0 * w * dt * dt;
    next[kY] += v * s0 * dt + 0.5 * v * c0 * w * dt * dt;
  }
  next[kYaw] = NormalizeAngle(yaw + w * dt);
  return next;
}

StateJacobian CtrvJacobian(const State& state, double dt) {
  const double yaw = state[kYaw];
  const double v = state[kSpeed];
  const double w = state[kYawRate];
  const double s0 = std::sin(yaw);
  const double c0 = std::cos(yaw);

  // Start from "every output depends only on its own input", then fill in
  // how x, y and yaw depend on the other inputs.
  StateJacobian jac = StateJacobian::Identity();
  if (std::abs(w) > kMinYawRate) {
    const double s1 = std::sin(yaw + w * dt);
    const double c1 = std::cos(yaw + w * dt);
    jac(kX, kYaw) = v / w * (c1 - c0);
    jac(kX, kSpeed) = (s1 - s0) / w;
    jac(kX, kYawRate) = -v / (w * w) * (s1 - s0) + v / w * c1 * dt;
    jac(kY, kYaw) = v / w * (s1 - s0);
    jac(kY, kSpeed) = (c0 - c1) / w;
    jac(kY, kYawRate) = -v / (w * w) * (c0 - c1) + v / w * s1 * dt;
  } else {
    jac(kX, kYaw) = -v * s0 * dt - 0.5 * v * c0 * w * dt * dt;
    jac(kX, kSpeed) = c0 * dt - 0.5 * s0 * w * dt * dt;
    jac(kX, kYawRate) = -0.5 * v * s0 * dt * dt;
    jac(kY, kYaw) = v * c0 * dt - 0.5 * v * s0 * w * dt * dt;
    jac(kY, kSpeed) = s0 * dt + 0.5 * c0 * w * dt * dt;
    jac(kY, kYawRate) = 0.5 * v * c0 * dt * dt;
  }
  jac(kYaw, kYawRate) = dt;
  return jac;
}

}  // namespace mot3d