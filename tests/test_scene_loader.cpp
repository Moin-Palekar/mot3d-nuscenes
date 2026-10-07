#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mot3d/io/scene_loader.h"

namespace mot3d {
namespace {

// Absolute paths baked in by tests/CMakeLists.txt, as in test_detection_loader.cpp.
const std::string kTinyScenes = std::string(MOT3D_TEST_DATA_DIR) + "/tiny_scenes";
const std::string kBrokenChain = std::string(MOT3D_TEST_DATA_DIR) + "/broken_chain";
const std::string kRealTables = std::string(MOT3D_REPO_DIR) + "/data/v1.0-trainval";

// Returns the scene with this name, or nullptr. Looking up by name keeps the
// tests independent of the order scenes come back in.
const Scene* FindScene(const std::vector<Scene>& scenes, const std::string& name) {
  for (const auto& scene : scenes) {
    if (scene.name == name) return &scene;
  }
  return nullptr;
}

// ---- Fixture tests: run everywhere, including CI ----

TEST(SceneLoader, ReportsMissingFolder) {
  std::vector<Scene> scenes;
  std::string error;
  EXPECT_FALSE(LoadScenes("/no/such/folder", &scenes, &error));
  EXPECT_FALSE(error.empty());
}

// tiny_scenes has two scenes: scene-a with 3 frames, scene-b with 2.
TEST(SceneLoader, LoadsEverySceneWithItsFrameCount) {
  std::vector<Scene> scenes;
  std::string error;
  ASSERT_TRUE(LoadScenes(kTinyScenes, &scenes, &error)) << error;

  EXPECT_EQ(scenes.size(), 2u);

  const Scene* a = FindScene(scenes, "scene-a");
  const Scene* b = FindScene(scenes, "scene-b");
  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(a->frames.size(), 3u);
  EXPECT_EQ(b->frames.size(), 2u);
}

// sample.json lists the cards in the order a3, b2, a1, b1, a2. Following the
// `next` links must still give a1, a2, a3. A loader that trusted file order
// would fail here.
TEST(SceneLoader, OrdersFramesByChainNotFileOrder) {
  std::vector<Scene> scenes;
  std::string error;
  ASSERT_TRUE(LoadScenes(kTinyScenes, &scenes, &error)) << error;

  const Scene* a = FindScene(scenes, "scene-a");
  ASSERT_NE(a, nullptr);
  ASSERT_EQ(a->frames.size(), 3u);
  EXPECT_EQ(a->frames[0].sample_token, "a1");
  EXPECT_EQ(a->frames[1].sample_token, "a2");
  EXPECT_EQ(a->frames[2].sample_token, "a3");

  const Scene* b = FindScene(scenes, "scene-b");
  ASSERT_NE(b, nullptr);
  ASSERT_EQ(b->frames.size(), 2u);
  EXPECT_EQ(b->frames[0].sample_token, "b1");
  EXPECT_EQ(b->frames[1].sample_token, "b2");
}

// Timestamps arrive in microseconds and must come out in seconds, each frame
// keeping its own time. scene-a's gaps are deliberately uneven: 0.5 s, 0.4 s.
TEST(SceneLoader, ConvertsTimestampsToSeconds) {
  std::vector<Scene> scenes;
  std::string error;
  ASSERT_TRUE(LoadScenes(kTinyScenes, &scenes, &error)) << error;

  const Scene* a = FindScene(scenes, "scene-a");
  ASSERT_NE(a, nullptr);
  ASSERT_EQ(a->frames.size(), 3u);

  EXPECT_DOUBLE_EQ(a->frames[0].timestamp, 1000000000.0);
  EXPECT_NEAR(a->frames[1].timestamp - a->frames[0].timestamp, 0.5, 1e-6);
  EXPECT_NEAR(a->frames[2].timestamp - a->frames[1].timestamp, 0.4, 1e-6);
}

// broken_chain: x2's `next` points to x9, which does not exist. The loader
// must refuse, and its message must name the scene and the missing token.
TEST(SceneLoader, RejectsDanglingNextLink) {
  std::vector<Scene> scenes;
  std::string error;
  EXPECT_FALSE(LoadScenes(kBrokenChain, &scenes, &error));
  EXPECT_NE(error.find("scene-x"), std::string::npos) << error;
  EXPECT_NE(error.find("x9"), std::string::npos) << error;
}

// ---- Real-data test: runs only where the nuScenes tables exist ----

TEST(SceneLoaderRealData, LoadsAllTrainvalScenes) {
  if (!std::filesystem::exists(kRealTables + "/scene.json")) {
    GTEST_SKIP() << "nuScenes tables not present: " << kRealTables;
  }

  std::vector<Scene> scenes;
  std::string error;
  ASSERT_TRUE(LoadScenes(kRealTables, &scenes, &error)) << error;

  EXPECT_EQ(scenes.size(), 850u);

  std::size_t total_frames = 0;
  for (const auto& scene : scenes) {
    total_frames += scene.frames.size();
  }
  EXPECT_EQ(total_frames, 34149u);
}

}  // namespace
}  // namespace mot3d