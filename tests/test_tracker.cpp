#include <vector>

#include <gtest/gtest.h>

#include "mot3d/tracking/tracker.h"

namespace mot3d {
namespace {

// A car at (x, y, 1), facing east, driving at 10 m/s.
Detection MakeCar(double x, double y) {
  Detection det;
  det.center = Eigen::Vector3d(x, y, 1.0);
  det.size = Eigen::Vector3d(1.9, 4.6, 1.7);
  det.yaw = 0.0;
  det.velocity = Eigen::Vector2d(10.0, 0.0);
  det.velocity_valid = true;
  det.score = 0.9;
  det.cls = ClassId::kCar;
  return det;
}

TEST(Tracker, FirstFrameStartsTracksButReportsNone) {
  Tracker tracker{TrackerParams{}};
  const auto out = tracker.Step({MakeCar(0.0, 0.0), MakeCar(0.0, 20.0)}, 0.0, "f0");
  EXPECT_TRUE(out.empty());  // 1 hit each: not confirmed yet
  EXPECT_EQ(tracker.tracks().size(), 2u);
}

TEST(Tracker, ConstantlySeenObjectKeepsItsId) {
  Tracker tracker{TrackerParams{}};
  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "f0");

  const auto second = tracker.Step({MakeCar(5.0, 0.0)}, 0.5, "f1");
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second[0].track_id, 0);
  EXPECT_EQ(second[0].box.sample_token, "f1");

  const auto third = tracker.Step({MakeCar(10.0, 0.0)}, 0.5, "f2");
  ASSERT_EQ(third.size(), 1u);
  EXPECT_EQ(third[0].track_id, 0);
  EXPECT_EQ(tracker.tracks().size(), 1u);
}

TEST(Tracker, TwoObjectsKeepSeparateIds) {
  Tracker tracker{TrackerParams{}};
  tracker.Step({MakeCar(0.0, 0.0), MakeCar(0.0, 20.0)}, 0.0, "f0");  // ids 0 and 1
  tracker.Step({MakeCar(5.0, 0.0), MakeCar(5.0, 20.0)}, 0.5, "f1");

  // Same cars, listed in the opposite order: ids must follow the cars, not the list.
  const auto out = tracker.Step({MakeCar(10.0, 20.0), MakeCar(10.0, 0.0)}, 0.5, "f2");
  ASSERT_EQ(out.size(), 2u);
  for (const TrackedBox& tracked : out) {
    const int expected_id = tracked.box.center.y() < 10.0 ? 0 : 1;
    EXPECT_EQ(tracked.track_id, expected_id);
  }
}

TEST(Tracker, DifferentClassesNeverMatch) {
  Tracker tracker{TrackerParams{}};
  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "f0");

  // A pedestrian exactly where the car is predicted to be.
  Detection pedestrian = MakeCar(5.0, 0.0);
  pedestrian.cls = ClassId::kPedestrian;
  tracker.Step({pedestrian}, 0.5, "f1");

  ASSERT_EQ(tracker.tracks().size(), 2u);
  EXPECT_EQ(tracker.tracks()[0].cls, ClassId::kCar);
  EXPECT_EQ(tracker.tracks()[0].misses, 1);
  EXPECT_EQ(tracker.tracks()[1].cls, ClassId::kPedestrian);
  EXPECT_EQ(tracker.tracks()[1].hits, 1);
}

TEST(Tracker, FarDetectionStartsNewTrack) {
  Tracker tracker{TrackerParams{}};
  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "f0");
  const auto out = tracker.Step({MakeCar(100.0, 0.0)}, 0.5, "f1");
  EXPECT_TRUE(out.empty());
  EXPECT_EQ(tracker.tracks().size(), 2u);  // the old car (missed) and a new one
}

TEST(Tracker, MissedTrackIsDeletedAfterMaxMisses) {
  Tracker tracker{TrackerParams{}};  // max_misses 2
  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "f0");
  tracker.Step({}, 0.5, "f1");
  tracker.Step({}, 0.5, "f2");
  EXPECT_EQ(tracker.tracks().size(), 1u);  // 2 misses: still kept
  tracker.Step({}, 0.5, "f3");
  EXPECT_TRUE(tracker.tracks().empty());  // 3rd miss: deleted
}

TEST(Tracker, ResetClearsTracksButIdsKeepCounting) {
  Tracker tracker{TrackerParams{}};
  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "f0");  // id 0
  tracker.Reset();
  EXPECT_TRUE(tracker.tracks().empty());

  tracker.Step({MakeCar(0.0, 0.0)}, 0.0, "g0");
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks()[0].id, 1);  // never reuses id 0
}

}  // namespace
}  // namespace mot3d