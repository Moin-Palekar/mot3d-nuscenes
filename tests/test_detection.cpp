#include <gtest/gtest.h>

#include <string>

#include "mot3d/io/detection.h"

namespace mot3d {

// The enum and the name table in detection.cpp are two hand-maintained lists
// that must stay in lockstep. Nothing in the language enforces that, so a
// mismatch would silently mislabel every detection of the affected class.
// Round-tripping every value catches it.
TEST(ClassId, RoundTripsThroughString) {
  for (std::size_t i = 0; i < kNumClasses; ++i) {
    const auto original = static_cast<ClassId>(i);

    ClassId parsed = ClassId::kCar;
    ASSERT_TRUE(ClassFromString(ToString(original), &parsed))
        << "failed to parse name for class index " << i;

    EXPECT_EQ(parsed, original) << "round trip changed class index " << i;
  }
}

// Pins the exact strings nuScenes expects. These appear in both the detection
// file we read and the submission file we write, so they are not ours to
// change — a typo here would make the scorer reject the submission.
TEST(ClassId, NamesMatchNuScenesVocabulary) {
  EXPECT_STREQ(ToString(ClassId::kBicycle), "bicycle");
  EXPECT_STREQ(ToString(ClassId::kBus), "bus");
  EXPECT_STREQ(ToString(ClassId::kCar), "car");
  EXPECT_STREQ(ToString(ClassId::kMotorcycle), "motorcycle");
  EXPECT_STREQ(ToString(ClassId::kPedestrian), "pedestrian");
  EXPECT_STREQ(ToString(ClassId::kTrailer), "trailer");
  EXPECT_STREQ(ToString(ClassId::kTruck), "truck");
}

// The detector emits 10 classes; nuScenes evaluates 7 for tracking. The three
// extras must be rejected so the loader skips them — a submission containing
// them is rejected outright by the official scorer.
TEST(ClassId, RejectsNonTrackingClasses) {
  ClassId cls = ClassId::kCar;

  EXPECT_FALSE(ClassFromString("barrier", &cls));
  EXPECT_FALSE(ClassFromString("traffic_cone", &cls));
  EXPECT_FALSE(ClassFromString("construction_vehicle", &cls));
  EXPECT_FALSE(ClassFromString("", &cls));
  EXPECT_FALSE(ClassFromString("Car", &cls));  // case-sensitive on purpose
}

}  // namespace mot3d