#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mot3d/io/detection.h"

namespace mot3d {

// All detections for one frame, plus the frame's identity.
struct Frame {
  std::string sample_token;
  std::vector<Detection> detections;
};

// Controls what the loader keeps. Boxes that fail these checks never enter
// the pipeline, which is both a correctness requirement (the scorer rejects
// non-tracking classes) and a large speedup (a third of the file is dropped).
struct LoadOptions {
  // Minimum detection score to keep. The file contains everything above 0.10,
  // and the median car score is 0.23, so most boxes are low-confidence noise.
  double min_score = 0.0;
};

// Reads a nuScenes detection result file into memory.
//
// The file is a single JSON object: {"meta": {...}, "results": {token: [box]}}.
// Boxes arrive in the global frame, so no transform is applied here. The
// quaternion is collapsed to a yaw angle and the class name to a ClassId.
//
// Returns false and sets *error on failure. Frames are returned keyed by
// sample token; a frame with no surviving detections is still present with an
// empty list, since the tracker must still step time forward for it.
bool LoadDetections(const std::string& path, const LoadOptions& options,
                    std::unordered_map<std::string, Frame>* out, std::string* error);

}  // namespace mot3d