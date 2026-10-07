#pragma once

#include <string>
#include <vector>

namespace mot3d {

// One keyframe inside a scene.
struct SceneFrame {
  std::string sample_token;

  // Seconds since the Unix epoch. The tables store microseconds; this is the
  // one place that conversion happens. Gaps between frames vary from about
  // 0.40 s to 0.65 s, so consumers must take differences, never assume 0.5.
  double timestamp = 0.0;
};

// A continuous ~20 s recording. Tracks must never carry across scenes: the
// next scene is an unrelated clip, even when it comes from the same drive.
struct Scene {
  std::string token;
  std::string name;                // e.g. "scene-0001", for logs
  std::vector<SceneFrame> frames;  // in time order, first to last
};

// Reads scene.json and sample.json from a nuScenes table directory such as
// data/v1.0-trainval, and returns every scene with its frames in order, found
// by walking each scene's chain of `next` links from its first sample.
//
// Returns false and sets *error if a table is missing or malformed, or if a
// chain is broken: a `next` token that does not exist, a chain whose length
// disagrees with the scene's nbr_samples, or timestamps that do not increase.
bool LoadScenes(const std::string& table_dir, std::vector<Scene>* out, std::string* error);

}  // namespace mot3d