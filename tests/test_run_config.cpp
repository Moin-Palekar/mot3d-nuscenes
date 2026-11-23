#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "mot3d/io/run_config.h"

namespace mot3d {
namespace {

// Writes `text` to a file in the system's temporary folder and returns its path.
std::string WriteTemp(const std::string& name, const std::string& text) {
  const std::string path = (std::filesystem::temp_directory_path() / name).string();
  std::ofstream(path) << text;
  return path;
}

TEST(RunConfig, MissingKeysKeepDefaults) {
  RunConfig config;
  std::string error;
  ASSERT_TRUE(LoadRunConfig(WriteTemp("rc_empty.json", "{}"), &config, &error)) << error;
  const RunConfig defaults;
  EXPECT_EQ(config.load.min_score, defaults.load.min_score);
  EXPECT_EQ(config.tracker.motion_model, defaults.tracker.motion_model);
  EXPECT_EQ(config.tracker.cost_type, defaults.tracker.cost_type);
  EXPECT_EQ(config.tracker.max_cost, defaults.tracker.max_cost);
  EXPECT_EQ(config.tracker.track.min_hits, defaults.tracker.track.min_hits);
  EXPECT_EQ(config.tracker.track.max_misses, defaults.tracker.track.max_misses);
  EXPECT_EQ(config.tracker.noise.pos_std, defaults.tracker.noise.pos_std);
}

TEST(RunConfig, OverridesOnlyListedKeys) {
  const std::string path = WriteTemp("rc_override.json", R"({
    "min_score": 0.2, "motion_model": "cv", "cost": "iou", "max_cost": 0.9,
    "min_hits": 1, "max_misses": 3, "noise": {"pos_std": 0.3}
  })");
  RunConfig config;
  std::string error;
  ASSERT_TRUE(LoadRunConfig(path, &config, &error)) << error;
  EXPECT_EQ(config.load.min_score, 0.2);
  EXPECT_EQ(config.tracker.motion_model, MotionModel::kCv);
  EXPECT_EQ(config.tracker.cost_type, CostType::kIou);
  EXPECT_EQ(config.tracker.max_cost, 0.9);
  EXPECT_EQ(config.tracker.track.min_hits, 1);
  EXPECT_EQ(config.tracker.track.max_misses, 3);
  EXPECT_EQ(config.tracker.noise.pos_std, 0.3);
  EXPECT_EQ(config.tracker.noise.accel_std, NoiseParams{}.accel_std);  // not listed: default
}

TEST(RunConfig, RejectsUnknownKey) {
  RunConfig config;
  std::string error;
  EXPECT_FALSE(LoadRunConfig(WriteTemp("rc_typo.json", R"({"min_hit": 1})"), &config, &error));
  EXPECT_NE(error.find("min_hit"), std::string::npos) << error;
}

TEST(RunConfig, RejectsUnknownNoiseKey) {
  RunConfig config;
  std::string error;
  EXPECT_FALSE(LoadRunConfig(WriteTemp("rc_noise_typo.json", R"({"noise": {"pos": 0.3}})"), &config,
                             &error));
  EXPECT_NE(error.find("pos"), std::string::npos) << error;
}

TEST(RunConfig, RejectsUnknownCostName) {
  RunConfig config;
  std::string error;
  EXPECT_FALSE(
      LoadRunConfig(WriteTemp("rc_cost.json", R"({"cost": "manhattan"})"), &config, &error));
}

TEST(RunConfig, RejectsWrongType) {
  RunConfig config;
  std::string error;
  EXPECT_FALSE(LoadRunConfig(WriteTemp("rc_type.json", R"({"min_hits": "two"})"), &config, &error));
}

TEST(RunConfig, RejectsMissingFile) {
  RunConfig config;
  std::string error;
  EXPECT_FALSE(LoadRunConfig("/no/such/dir/config.json", &config, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RunConfig, FailureLeavesOutputUntouched) {
  RunConfig config;
  config.tracker.track.min_hits = 7;
  std::string error;
  EXPECT_FALSE(
      LoadRunConfig(WriteTemp("rc_bad.json", R"({"min_hits": 1, "oops": 0})"), &config, &error));
  EXPECT_EQ(config.tracker.track.min_hits, 7);
}

}  // namespace
}  // namespace mot3d