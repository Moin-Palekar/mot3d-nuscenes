#include <cmath>
#include <filesystem>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>

#include "mot3d/io/detection_loader.h"

namespace mot3d {
namespace {

// Absolute paths baked in by tests/CMakeLists.txt. ctest runs the binary from
// inside build/, so a relative path like "tests/data/..." would not resolve.
const std::string kFixture = std::string(MOT3D_TEST_DATA_DIR) + "/tiny_detections.json";
const std::string kRealFile =
    std::string(MOT3D_REPO_DIR) + "/data/detections/infos_val_10sweeps_withvelo_filter_True.json";

constexpr double kPi = 3.14159265358979323846;

// Returns the first detection of a class in a frame, or nullptr. Looking up by
// class instead of by index keeps the tests independent of box order.
const Detection* FindFirst(const Frame& frame, ClassId cls) {
  for (const auto& det : frame.detections) {
    if (det.cls == cls) return &det;
  }
  return nullptr;
}

// ---- Fixture tests: run everywhere, including CI ----

TEST(DetectionLoader, ReportsMissingFile) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  EXPECT_FALSE(LoadDetections("/no/such/file.json", LoadOptions{}, &frames, &error));
  EXPECT_FALSE(error.empty());
}

// frame_c holds only a traffic cone, which is dropped. The frame itself must
// survive with an empty list, or the tracker would skip a time step there.
TEST(DetectionLoader, KeepsFramesThatEndUpEmpty) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  ASSERT_TRUE(LoadDetections(kFixture, LoadOptions{}, &frames, &error)) << error;

  EXPECT_EQ(frames.size(), 3u);
  ASSERT_EQ(frames.count("frame_c"), 1u);
  EXPECT_TRUE(frames.at("frame_c").detections.empty());
}

// frame_a has car, barrier, pedestrian. The barrier must go at any score.
TEST(DetectionLoader, DropsNonTrackingClasses) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  ASSERT_TRUE(LoadDetections(kFixture, LoadOptions{}, &frames, &error)) << error;

  const Frame& a = frames.at("frame_a");
  EXPECT_EQ(a.detections.size(), 2u);
  EXPECT_NE(FindFirst(a, ClassId::kCar), nullptr);
  EXPECT_NE(FindFirst(a, ClassId::kPedestrian), nullptr);
}

// The pedestrian scores 0.05: kept at the default threshold, dropped at 0.1.
TEST(DetectionLoader, ScoreThresholdDropsLowConfidenceBoxes) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  LoadOptions options;
  options.min_score = 0.1;
  ASSERT_TRUE(LoadDetections(kFixture, options, &frames, &error)) << error;

  const Frame& a = frames.at("frame_a");
  ASSERT_EQ(a.detections.size(), 1u);
  EXPECT_EQ(a.detections[0].cls, ClassId::kCar);
}

// Pins every field of one box, including the [width, length, height] order.
TEST(DetectionLoader, ReadsCarFieldsInNuScenesOrder) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  ASSERT_TRUE(LoadDetections(kFixture, LoadOptions{}, &frames, &error)) << error;

  const Detection* car = FindFirst(frames.at("frame_a"), ClassId::kCar);
  ASSERT_NE(car, nullptr);

  EXPECT_EQ(car->sample_token, "frame_a");
  EXPECT_DOUBLE_EQ(car->center.x(), 10.0);
  EXPECT_DOUBLE_EQ(car->center.y(), 20.0);
  EXPECT_DOUBLE_EQ(car->center.z(), 1.0);

  EXPECT_DOUBLE_EQ(car->size[0], 1.9);  // width
  EXPECT_DOUBLE_EQ(car->size[1], 4.6);  // length
  EXPECT_DOUBLE_EQ(car->size[2], 1.7);  // height

  EXPECT_NEAR(car->yaw, kPi / 2, 1e-9);

  EXPECT_TRUE(car->velocity_valid);
  EXPECT_DOUBLE_EQ(car->velocity.x(), 1.0);
  EXPECT_DOUBLE_EQ(car->velocity.y(), 2.0);

  EXPECT_DOUBLE_EQ(car->score, 0.9);
}

// frame_b's quaternion is frame_a's with every component negated (w < 0).
// q and -q are the same rotation, so the yaw must match.
TEST(DetectionLoader, NegatedQuaternionGivesSameYaw) {
  std::unordered_map<std::string, Frame> frames;
  std::string error;
  ASSERT_TRUE(LoadDetections(kFixture, LoadOptions{}, &frames, &error)) << error;

  const Detection* car = FindFirst(frames.at("frame_b"), ClassId::kCar);
  ASSERT_NE(car, nullptr);
  EXPECT_NEAR(car->yaw, kPi / 2, 1e-9);
}

// ---- Real-data test: runs only where the 345 MB file exists ----

TEST(DetectionLoaderRealData, LoadsFullValSplit) {
  if (!std::filesystem::exists(kRealFile)) {
    GTEST_SKIP() << "real detection file not present: " << kRealFile;
  }

  std::unordered_map<std::string, Frame> frames;
  std::string error;
  ASSERT_TRUE(LoadDetections(kRealFile, LoadOptions{}, &frames, &error)) << error;

  EXPECT_EQ(frames.size(), 6019u);

  std::size_t total = 0;
  for (const auto& [token, frame] : frames) {
    for (const auto& det : frame.detections) {
      ++total;
      ASSERT_LT(static_cast<std::size_t>(det.cls), kNumClasses);
      ASSERT_GE(det.yaw, -kPi);
      ASSERT_LE(det.yaw, kPi);
      // This file contains no NaN velocities (checked with grep). If this
      // fails, a different detection file is on disk.
      ASSERT_TRUE(det.velocity_valid);
    }
  }
  // Roughly 772k boxes minus about 265k non-tracking ones.
  EXPECT_GT(total, 400000u);
  EXPECT_LT(total, 600000u);
}

}  // namespace
}  // namespace mot3d