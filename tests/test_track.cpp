#include <cmath>

#include <gtest/gtest.h>

#include "mot3d/tracking/track.h"

namespace mot3d {
namespace {

const double kPi = std::acos(-1.0);

// A truck with no velocity, at height 1.
Detection MakeDetection(double x, double y, double yaw) {
  Detection det;
  det.sample_token = "frame_a";
  det.center = Eigen::Vector3d(x, y, 1.0);
  det.size = Eigen::Vector3d(1.9, 4.6, 1.7);
  det.yaw = yaw;
  det.score = 0.8;
  det.cls = ClassId::kTruck;
  return det;
}

TEST(Track, StartTrackCopiesDetection) {
  const Detection det = MakeDetection(10.0, -3.0, 0.4);
  const Track track = StartTrack(det, 7, NoiseParams{});
  EXPECT_EQ(track.id, 7);
  EXPECT_EQ(track.cls, ClassId::kTruck);
  EXPECT_NEAR(track.filter.mean[kX], 10.0, 1e-12);
  EXPECT_NEAR(track.filter.mean[kY], -3.0, 1e-12);
  EXPECT_NEAR(track.filter.mean[kZ], 1.0, 1e-12);
  EXPECT_NEAR(track.filter.mean[kYaw], 0.4, 1e-12);
  EXPECT_TRUE(track.size.isApprox(det.size));
  EXPECT_DOUBLE_EQ(track.score, 0.8);
  EXPECT_EQ(track.hits, 1);
  EXPECT_EQ(track.misses, 0);
}

TEST(Track, StartTrackTakesSpeedAlongHeading) {
  Detection det = MakeDetection(0.0, 0.0, 0.0);  // facing east
  det.velocity = Eigen::Vector2d(3.0, 4.0);
  det.velocity_valid = true;
  const Track track = StartTrack(det, 0, NoiseParams{});
  EXPECT_NEAR(track.filter.mean[kSpeed], 3.0, 1e-12);
}

TEST(Track, StartTrackUsesZeroSpeedWhenVelocityInvalid) {
  Detection det = MakeDetection(0.0, 0.0, 0.0);
  det.velocity = Eigen::Vector2d(3.0, 4.0);
  det.velocity_valid = false;
  const Track track = StartTrack(det, 0, NoiseParams{});
  EXPECT_NEAR(track.filter.mean[kSpeed], 0.0, 1e-12);
}

TEST(Track, PredictMovesBeliefButNotCounters) {
  Detection det = MakeDetection(0.0, 0.0, 0.0);
  det.velocity = Eigen::Vector2d(10.0, 0.0);
  det.velocity_valid = true;
  Track track = StartTrack(det, 0, NoiseParams{});
  PredictTrack(0.5, NoiseParams{}, MotionModel::kCv, &track);
  EXPECT_NEAR(track.filter.mean[kX], 5.0, 1e-9);
  EXPECT_EQ(track.hits, 1);
  EXPECT_EQ(track.misses, 0);
}

TEST(Track, UpdateCountsHitAndEndsMissStreak) {
  Track track = StartTrack(MakeDetection(0.0, 0.0, 0.0), 0, NoiseParams{});
  MarkMissed(&track);
  MarkMissed(&track);
  EXPECT_EQ(track.misses, 2);

  Detection later = MakeDetection(0.5, 0.0, 0.0);
  later.size = Eigen::Vector3d(2.0, 5.0, 1.8);
  later.score = 0.6;
  UpdateTrack(later, NoiseParams{}, &track);
  EXPECT_EQ(track.hits, 2);
  EXPECT_EQ(track.misses, 0);
  EXPECT_TRUE(track.size.isApprox(later.size));
  EXPECT_DOUBLE_EQ(track.score, 0.6);
}

TEST(Track, ReportsOnlyAfterMinHitsAndWhenMatched) {
  const TrackParams params;  // min_hits 2
  Track track = StartTrack(MakeDetection(0.0, 0.0, 0.0), 0, NoiseParams{});
  EXPECT_FALSE(ShouldReport(track, params));  // 1 hit: could be a one-frame false detection

  UpdateTrack(MakeDetection(0.1, 0.0, 0.0), NoiseParams{}, &track);
  EXPECT_TRUE(ShouldReport(track, params));  // 2 hits and matched this frame

  MarkMissed(&track);
  EXPECT_FALSE(ShouldReport(track, params));  // no detection behind it this frame
}

TEST(Track, DeletesOnlyAfterTooManyMisses) {
  const TrackParams params;  // max_misses 2
  Track track = StartTrack(MakeDetection(0.0, 0.0, 0.0), 0, NoiseParams{});
  MarkMissed(&track);
  MarkMissed(&track);
  EXPECT_FALSE(ShouldDelete(track, params));  // 2 misses still allowed
  MarkMissed(&track);
  EXPECT_TRUE(ShouldDelete(track, params));  // 3rd miss: gone
}

TEST(Track, ToTrackedBoxUsesFilteredBelief) {
  Track track = StartTrack(MakeDetection(0.0, 0.0, 0.0), 12, NoiseParams{});
  // Pretend the filter now believes: at (4, 5, 1), facing north, 5 m/s.
  track.filter.mean << 4.0, 5.0, 1.0, kPi / 2.0, 5.0, 0.0;

  const TrackedBox out = ToTrackedBox(track, "frame_b");
  EXPECT_EQ(out.track_id, 12);
  EXPECT_EQ(out.box.sample_token, "frame_b");
  EXPECT_NEAR(out.box.center.x(), 4.0, 1e-12);
  EXPECT_NEAR(out.box.center.y(), 5.0, 1e-12);
  EXPECT_NEAR(out.box.center.z(), 1.0, 1e-12);
  EXPECT_NEAR(out.box.yaw, kPi / 2.0, 1e-12);
  EXPECT_NEAR(out.box.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(out.box.velocity.y(), 5.0, 1e-9);
  EXPECT_TRUE(out.box.velocity_valid);
  EXPECT_TRUE(out.box.size.isApprox(track.size));
  EXPECT_DOUBLE_EQ(out.box.score, 0.8);
  EXPECT_EQ(out.box.cls, ClassId::kTruck);
}

}  // namespace
}  // namespace mot3d