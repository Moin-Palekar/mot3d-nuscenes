#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/tracking/cost.h"

namespace mot3d {
namespace {

constexpr double kTol = 1e-9;

Detection MakeDetection(double x, double y, double z, double yaw) {
  Detection det;
  det.center = Eigen::Vector3d(x, y, z);
  det.yaw = yaw;
  return det;
}

// A belief at the origin, facing east, whose x is unsure by `x_variance`.
// Every other number has a simple fixed uncertainty, so costs are easy to
// work out by hand.
FilterState MakeFilter(double x_variance) {
  FilterState filter;
  filter.mean = State::Zero();
  State variances;
  variances << x_variance, 1.0, 1.0, 0.04, 25.0, 0.25;
  filter.cov = variances.asDiagonal();
  return filter;
}

TEST(Cost, ToMeasurementCopiesCentreAndYaw) {
  const Measurement z = ToMeasurement(MakeDetection(1.0, 2.0, 3.0, 0.4));
  EXPECT_NEAR(z[0], 1.0, kTol);
  EXPECT_NEAR(z[1], 2.0, kTol);
  EXPECT_NEAR(z[2], 3.0, kTol);
  EXPECT_NEAR(z[3], 0.4, kTol);
}

TEST(Cost, EuclideanIsGroundDistanceIgnoringHeight) {
  FilterState filter = MakeFilter(1.0);
  filter.mean[kX] = 54.0;
  filter.mean[kY] = 20.0;
  // 0.6 m east and 0.8 m north: sqrt(0.36 + 0.64) = 1.0. The height differs
  // by 5 m, which must not count.
  EXPECT_NEAR(EuclideanCost(filter, MakeDetection(54.6, 20.8, 5.0, 0.0)), 1.0, kTol);
}

TEST(Cost, MahalanobisIsZeroForPerfectDetection) {
  const NoiseParams noise;
  EXPECT_NEAR(MahalanobisCost(MakeFilter(6.5), MakeDetection(0.0, 0.0, 0.0, 0.0), noise), 0.0,
              kTol);
}

TEST(Cost, MahalanobisMatchesHandCalculation) {
  // Expected x gap = filter x variance + detection x variance = 6.5 + 0.25 = 6.75.
  // A 1 m gap in x only gives sqrt(1 / 6.75) = 0.3849.
  const NoiseParams noise;
  const double cost = MahalanobisCost(MakeFilter(6.5), MakeDetection(1.0, 0.0, 0.0, 0.0), noise);
  EXPECT_NEAR(cost, std::sqrt(1.0 / 6.75), kTol);
}

TEST(Cost, MahalanobisScoresSameGapHigherForConfidentTrack) {
  const NoiseParams noise;
  const Detection one_metre_east = MakeDetection(1.0, 0.0, 0.0, 0.0);
  const double unsure = MahalanobisCost(MakeFilter(6.5), one_metre_east, noise);
  const double confident = MahalanobisCost(MakeFilter(0.05), one_metre_east, noise);
  EXPECT_GT(confident, unsure);
}

TEST(Cost, MahalanobisTreatsHeadingsAcrossPiAsClose) {
  const NoiseParams noise;
  FilterState filter = MakeFilter(1.0);
  filter.mean[kYaw] = 3.1;
  // -3.1 and 3.1 are about 0.083 rad apart. Unwrapped, the gap would look like
  // 6.2 rad and the cost would be over 20.
  EXPECT_LT(MahalanobisCost(filter, MakeDetection(0.0, 0.0, 0.0, -3.1), noise), 1.0);
}

TEST(Cost, MahalanobisPrefersGapAheadOfMovingCar) {
  // A car facing east at 8 m/s with an unsure speed. After predicting, its
  // position is much less certain along the road (x) than across it (y).
  const NoiseParams noise;
  Measurement first;
  first << 0.0, 0.0, 0.0, 0.0;
  FilterState filter = InitFilter(first, 8.0, noise);
  PredictFilter(0.5, noise, &filter);
  const double x = filter.mean[kX];
  const double y = filter.mean[kY];
  const double ahead = MahalanobisCost(filter, MakeDetection(x + 1.0, y, 0.0, 0.0), noise);
  const double sideways = MahalanobisCost(filter, MakeDetection(x, y + 1.0, 0.0, 0.0), noise);
  EXPECT_LT(ahead, sideways);
}

// A track whose filter believes it is at (x, y, 1), facing yaw, sized 2 x 4 x 1.5.
Track MakeTrackAt(double x, double y, double yaw) {
  Track track;
  track.filter.mean << x, y, 1.0, yaw, 0.0, 0.0;
  track.size = Eigen::Vector3d(2.0, 4.0, 1.5);
  return track;
}

// A detection box at (x, y, 1), facing yaw, sized 2 x 4 x 1.5.
Detection MakeBoxDetection(double x, double y, double yaw) {
  Detection det;
  det.center = Eigen::Vector3d(x, y, 1.0);
  det.size = Eigen::Vector3d(2.0, 4.0, 1.5);
  det.yaw = yaw;
  return det;
}

TEST(Cost, IouCostIsZeroForIdenticalBoxes) {
  EXPECT_NEAR(IouCost(MakeTrackAt(5.0, 2.0, 0.3), MakeBoxDetection(5.0, 2.0, 0.3)), 0.0, 1e-9);
}

TEST(Cost, IouCostIsOneWhenBoxesDoNotTouch) {
  EXPECT_NEAR(IouCost(MakeTrackAt(0.0, 0.0, 0.0), MakeBoxDetection(50.0, 0.0, 0.0)), 1.0, 1e-9);
}

TEST(Cost, IouCostUsesTrackPosition) {
  // Shifted half a length along x: overlap 2 m, union 6 m, IoU 1/3.
  EXPECT_NEAR(IouCost(MakeTrackAt(0.0, 0.0, 0.0), MakeBoxDetection(2.0, 0.0, 0.0)), 2.0 / 3.0,
              1e-9);
}

TEST(Cost, IouCostUsesTrackSize) {
  // Same centre, detection half as long: it sits inside the track's box, IoU 1/2.
  Detection det = MakeBoxDetection(0.0, 0.0, 0.0);
  det.size = Eigen::Vector3d(2.0, 2.0, 1.5);
  EXPECT_NEAR(IouCost(MakeTrackAt(0.0, 0.0, 0.0), det), 0.5, 1e-9);
}

}  // namespace
}  // namespace mot3d