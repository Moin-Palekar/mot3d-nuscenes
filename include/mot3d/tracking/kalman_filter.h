#pragma once

#include <Eigen/Dense>

#include "mot3d/tracking/motion_model.h"

namespace mot3d {

// A detection gives us 4 numbers about an object: x, y, z and yaw.
constexpr int kMeasurementSize = 4;

using Measurement = Eigen::Matrix<double, kMeasurementSize, 1>;
using StateCovariance = Eigen::Matrix<double, kStateSize, kStateSize>;
using MeasurementCovariance = Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

// Every noise setting in one place, so M3 can tune them without touching code.
struct NoiseParams {
  // Process noise: how much an object's motion can change unexpectedly.
  double accel_std = 3.0;      // m/s^2, random change in speed
  double yaw_accel_std = 1.0;  // rad/s^2, random change in turn rate
  double z_drift_std = 0.5;    // m per sqrt(second), random change in height

  // Measurement noise: how far off a single detection can be.
  double pos_std = 0.5;  // m, for x and y
  double z_std = 0.5;    // m
  double yaw_std = 0.2;  // rad

  // Starting uncertainty for what one detection can't tell us.
  double init_speed_std = 5.0;     // m/s
  double init_yaw_rate_std = 0.5;  // rad/s
};

// The filter's belief about one object: a best guess (mean) and how unsure
// it is about every number in that guess (covariance).
struct FilterState {
  State mean = State::Zero();
  StateCovariance cov = StateCovariance::Identity();
};

// How far a detection is from the filter's prediction, and how uncertain that
// gap is. Used by the update step, and later by the Mahalanobis cost.
struct Innovation {
  Measurement residual = Measurement::Zero();
  MeasurementCovariance cov = MeasurementCovariance::Identity();
};

// Starts a filter from a first detection. `speed` is the detector's own speed
// estimate for the object.
FilterState InitFilter(const Measurement& z, double speed, const NoiseParams& noise);

// Moves the belief forward by dt seconds. Uncertainty grows.
void PredictFilter(double dt, const NoiseParams& noise, FilterState* filter);

// Compares a detection with the current belief.
Innovation ComputeInnovation(const FilterState& filter, const Measurement& z,
                             const NoiseParams& noise);

// Blends a detection into the belief. Uncertainty shrinks.
void UpdateFilter(const Measurement& z, const NoiseParams& noise, FilterState* filter);

}  // namespace mot3d