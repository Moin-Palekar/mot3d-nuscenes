#pragma once

#include <string>
#include <vector>

#include "mot3d/io/detection.h"
#include "mot3d/io/submission_writer.h"
#include "mot3d/tracking/kalman_filter.h"
#include "mot3d/tracking/motion_model.h"
#include "mot3d/tracking/track.h"

namespace mot3d {

// Which cost scores a (track, detection) pair.
enum class CostType { kMahalanobis, kEuclidean, kIou };

// Every setting of the tracker in one place.
struct TrackerParams {
  MotionModel motion_model = MotionModel::kCtrv;
  CostType cost_type = CostType::kMahalanobis;

  // Pairs costing more than this are never matched. 3.64 lets about 99% of true
  // matches through for Mahalanobis with 4 measured numbers. Euclidean and IoU
  // need their own values (set in M3).
  double max_cost = 3.64;

  NoiseParams noise;
  TrackParams track;
};

// Follows objects across the frames of one scene.
class Tracker {
 public:
  explicit Tracker(const TrackerParams& params);

  // Processes one frame and returns the boxes to report for it.
  // dt is the time in seconds since the previous frame of the same scene.
  std::vector<TrackedBox> Step(const std::vector<Detection>& detections, double dt,
                               const std::string& sample_token);

  // Forgets every track. Call at the start of each scene.
  // Ids keep counting up, so no id is ever reused.
  void Reset();

  const std::vector<Track>& tracks() const {
    return tracks_;
  }

 private:
  TrackerParams params_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};

}  // namespace mot3d