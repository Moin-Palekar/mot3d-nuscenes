#pragma once

#include <Eigen/Dense>

namespace mot3d {

// Where each quantity lives inside the state vector.
enum StateIndex : int {
  kX = 0,        // east position, metres (global frame)
  kY = 1,        // north position, metres
  kZ = 2,        // height, metres
  kYaw = 3,      // heading, radians, 0 = facing east, counterclockwise positive
  kSpeed = 4,    // speed along the heading, metres per second
  kYawRate = 5,  // turning rate, radians per second
};

constexpr int kStateSize = 6;

using State = Eigen::Matrix<double, kStateSize, 1>;
using StateJacobian = Eigen::Matrix<double, kStateSize, kStateSize>;

// Wraps an angle into [-pi, pi], so the same heading always has one number.
double NormalizeAngle(double angle);

// CTRV: constant turn rate and velocity.
// Moves a state forward by dt seconds, assuming speed and turn rate stay fixed.
State PredictCtrv(const State& state, double dt);

// For each predicted number, how much it changes when each input number
// changes a tiny bit. The Kalman filter uses this to grow uncertainty correctly.
StateJacobian CtrvJacobian(const State& state, double dt);

}  // namespace mot3d