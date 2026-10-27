#include "mot3d/tracking/kalman_filter.h"

#include <cmath>

namespace mot3d {
namespace {

using MeasurementMatrix = Eigen::Matrix<double, kMeasurementSize, kStateSize>;
using KalmanGain = Eigen::Matrix<double, kStateSize, kMeasurementSize>;

// Which state numbers a detection measures: x, y, z and yaw, directly.
MeasurementMatrix MeasurementModel() {
  MeasurementMatrix h = MeasurementMatrix::Zero();
  h(0, kX) = 1.0;
  h(1, kY) = 1.0;
  h(2, kZ) = 1.0;
  h(3, kYaw) = 1.0;
  return h;
}

// How noisy a detection is, as a covariance.
MeasurementCovariance MeasurementNoise(const NoiseParams& noise) {
  Measurement variances;
  variances << noise.pos_std * noise.pos_std, noise.pos_std * noise.pos_std,
      noise.z_std * noise.z_std, noise.yaw_std * noise.yaw_std;
  return variances.asDiagonal();
}

// How much uncertainty dt seconds of unknown acceleration and turning adds.
StateCovariance ProcessNoise(const State& mean, double dt, const NoiseParams& noise) {
  // How a sudden push in acceleration (column 0) or turning (column 1)
  // spreads into each state number over dt seconds.
  const double half_dt2 = 0.5 * dt * dt;
  Eigen::Matrix<double, kStateSize, 2> g = Eigen::Matrix<double, kStateSize, 2>::Zero();
  g(kX, 0) = half_dt2 * std::cos(mean[kYaw]);
  g(kY, 0) = half_dt2 * std::sin(mean[kYaw]);
  g(kSpeed, 0) = dt;
  g(kYaw, 1) = half_dt2;
  g(kYawRate, 1) = dt;

  Eigen::Matrix2d push = Eigen::Matrix2d::Zero();
  push(0, 0) = noise.accel_std * noise.accel_std;
  push(1, 1) = noise.yaw_accel_std * noise.yaw_accel_std;

  StateCovariance q = g * push * g.transpose();
  q(kZ, kZ) += noise.z_drift_std * noise.z_drift_std * dt;
  return q;
}

}  // namespace

FilterState InitFilter(const Measurement& z, double speed, const NoiseParams& noise) {
  FilterState filter;
  filter.mean << z[0], z[1], z[2], NormalizeAngle(z[3]), speed, 0.0;

  State variances;
  variances << noise.pos_std * noise.pos_std, noise.pos_std * noise.pos_std,
      noise.z_std * noise.z_std, noise.yaw_std * noise.yaw_std,
      noise.init_speed_std * noise.init_speed_std,
      noise.init_yaw_rate_std * noise.init_yaw_rate_std;
  filter.cov = variances.asDiagonal();
  return filter;
}

void PredictFilter(double dt, const NoiseParams& noise, FilterState* filter) {
  // The Jacobian and process noise are worked out at the current belief,
  // so they must be computed before the mean moves.
  const StateJacobian f = CtrvJacobian(filter->mean, dt);
  filter->cov = f * filter->cov * f.transpose() + ProcessNoise(filter->mean, dt, noise);
  filter->mean = PredictCtrv(filter->mean, dt);
}

Innovation ComputeInnovation(const FilterState& filter, const Measurement& z,
                             const NoiseParams& noise) {
  const MeasurementMatrix h = MeasurementModel();
  Innovation innovation;
  innovation.residual = z - h * filter.mean;
  // Headings of 3.1 and -3.1 differ by about 0.08 rad, not 6.2.
  innovation.residual[3] = NormalizeAngle(innovation.residual[3]);
  innovation.cov = h * filter.cov * h.transpose() + MeasurementNoise(noise);
  return innovation;
}

void UpdateFilter(const Measurement& z, const NoiseParams& noise, FilterState* filter) {
  const MeasurementMatrix h = MeasurementModel();
  const Innovation innovation = ComputeInnovation(*filter, z, noise);
  const KalmanGain gain = filter->cov * h.transpose() * innovation.cov.inverse();

  filter->mean += gain * innovation.residual;
  filter->mean[kYaw] = NormalizeAngle(filter->mean[kYaw]);

  // Joseph form: the same result as the textbook (I - KH) P, but it stays
  // symmetric and positive under rounding errors over thousands of updates.
  const StateCovariance i_kh = StateCovariance::Identity() - gain * h;
  filter->cov =
      i_kh * filter->cov * i_kh.transpose() + gain * MeasurementNoise(noise) * gain.transpose();
}

}  // namespace mot3d