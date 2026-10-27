#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/tracking/kalman_filter.h"

namespace mot3d {
namespace {

const double kPi = std::acos(-1.0);
constexpr double kTol = 1e-9;

Measurement MakeMeasurement(double x, double y, double z, double yaw) {
  Measurement m;
  m << x, y, z, yaw;
  return m;
}

TEST(KalmanFilter, InitPutsDetectionInState) {
  const NoiseParams noise;
  const FilterState f = InitFilter(MakeMeasurement(1.0, 2.0, 0.5, 0.3), 7.0, noise);
  EXPECT_NEAR(f.mean[kX], 1.0, kTol);
  EXPECT_NEAR(f.mean[kY], 2.0, kTol);
  EXPECT_NEAR(f.mean[kZ], 0.5, kTol);
  EXPECT_NEAR(f.mean[kYaw], 0.3, kTol);
  EXPECT_NEAR(f.mean[kSpeed], 7.0, kTol);
  EXPECT_NEAR(f.mean[kYawRate], 0.0, kTol);
  EXPECT_NEAR(f.cov(kX, kX), noise.pos_std * noise.pos_std, kTol);
  EXPECT_NEAR(f.cov(kSpeed, kSpeed), noise.init_speed_std * noise.init_speed_std, kTol);
  EXPECT_NEAR(f.cov(kX, kY), 0.0, kTol);
}

TEST(KalmanFilter, PredictMovesMeanWithMotionModel) {
  const NoiseParams noise;
  FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 0.5), 10.0, noise);
  f.mean[kYawRate] = 0.2;
  const State expected = PredictCtrv(f.mean, 0.5);
  PredictFilter(0.5, noise, &f);
  for (int i = 0; i < kStateSize; ++i) {
    EXPECT_NEAR(f.mean[i], expected[i], kTol) << "state index " << i;
  }
}

TEST(KalmanFilter, PredictGrowsUncertainty) {
  const NoiseParams noise;
  FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 0.0), 10.0, noise);
  const double x_before = f.cov(kX, kX);
  const double speed_before = f.cov(kSpeed, kSpeed);
  PredictFilter(0.5, noise, &f);
  EXPECT_GT(f.cov(kX, kX), x_before);
  EXPECT_GT(f.cov(kSpeed, kSpeed), speed_before);
}

TEST(KalmanFilter, UpdatePullsTowardDetectionAndShrinksUncertainty) {
  const NoiseParams noise;
  FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 0.0), 0.0, noise);
  PredictFilter(0.5, noise, &f);
  const double x_before = f.cov(kX, kX);
  UpdateFilter(MakeMeasurement(1.0, 0.0, 0.0, 0.0), noise, &f);
  // The guess moves toward the detection, but not all the way.
  EXPECT_GT(f.mean[kX], 0.0);
  EXPECT_LT(f.mean[kX], 1.0);
  EXPECT_LT(f.cov(kX, kX), x_before);
}

TEST(KalmanFilter, YawResidualWrapsAcrossPi) {
  const NoiseParams noise;
  const FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 3.1), 0.0, noise);
  const Innovation innovation = ComputeInnovation(f, MakeMeasurement(0.0, 0.0, 0.0, -3.1), noise);
  // -3.1 - 3.1 = -6.2 rad, which is really a small turn of 2 pi - 6.2 = 0.083 rad.
  EXPECT_NEAR(innovation.residual[3], 2.0 * kPi - 6.2, kTol);
}

TEST(KalmanFilter, CovarianceStaysSymmetricAndPositive) {
  const NoiseParams noise;
  FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 0.2), 5.0, noise);
  f.mean[kYawRate] = 0.3;
  for (int step = 1; step <= 50; ++step) {
    PredictFilter(0.5, noise, &f);
    UpdateFilter(MakeMeasurement(1.0 * step, 0.5 * step, 0.0, 0.2), noise, &f);
  }
  EXPECT_NEAR((f.cov - f.cov.transpose()).cwiseAbs().maxCoeff(), 0.0, kTol);
  for (int i = 0; i < kStateSize; ++i) {
    EXPECT_GT(f.cov(i, i), 0.0) << "state index " << i;
  }
}

TEST(KalmanFilter, LearnsSpeedFromPositions) {
  // A car drives east at 10 m/s, detected exactly every 0.5 s. The filter
  // starts believing it is standing still and must learn the speed itself.
  const NoiseParams noise;
  FilterState f = InitFilter(MakeMeasurement(0.0, 0.0, 0.0, 0.0), 0.0, noise);
  for (int step = 1; step <= 20; ++step) {
    PredictFilter(0.5, noise, &f);
    UpdateFilter(MakeMeasurement(10.0 * 0.5 * step, 0.0, 0.0, 0.0), noise, &f);
  }
  EXPECT_NEAR(f.mean[kSpeed], 10.0, 0.5);
  EXPECT_NEAR(f.mean[kX], 100.0, 0.5);
}

}  // namespace
}  // namespace mot3d