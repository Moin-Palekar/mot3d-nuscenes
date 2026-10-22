#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mot3d/io/detection.h"

namespace mot3d {

// One box the tracker outputs: a detection plus the identity the tracker gave
// it. The same real object keeps the same track_id in every frame.
struct TrackedBox {
  Detection box;
  int track_id = -1;
};

// Writes a nuScenes tracking submission file.
//
// `results` maps each frame's sample token to that frame's boxes. It must
// contain every frame being scored, including frames with no boxes: the
// official scorer rejects a file whose frames do not exactly match the split.
//
// Each box is converted to the scorer's format: yaw becomes a quaternion
// [w, x, y, z], size stays [width, length, height], an invalid velocity is
// written as [0, 0], and track_id becomes a string.
//
// Returns false and sets *error if the file cannot be written.
bool WriteSubmission(const std::string& path,
                     const std::unordered_map<std::string, std::vector<TrackedBox>>& results,
                     std::string* error);

}  // namespace mot3d