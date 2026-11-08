#pragma once

#include <string>

#include <Eigen/Core>

#include "mot3d/io/detection.h"
#include "mot3d/io/submission_writer.h"
#include "mot3d/tracking/kalman_filter.h"
#include "mot3d/tracking/motion_model.h"

namespace mot3d {

// The rules for when a track is reported and when it is deleted.
// Starting values; tuned in M3.
struct TrackParams {
  int min_hits = 2;    // matched frames needed before the track is reported
  int max_misses = 2;  // unmatched frames in a row allowed before deletion
};

// One object being followed over time.
struct Track {
  int id = -1;
  ClassId cls = ClassId::kCar;
  FilterState filter;  // where it is and how it moves, with uncertainty

  // Not part of the filter state, so kept beside it.
  // Copied from the latest matched detection.
  Eigen::Vector3d size = Eigen::Vector3d::Zero();  // [width, length, height]
  double score = 0.0;

  int hits = 0;    // total frames this track was matched (birth counts as one)
  int misses = 0;  // frames in a row with no match
};

// Birth: a new track from an unmatched detection.
Track StartTrack(const Detection& det, int id, const NoiseParams& noise);

// Moves the track's belief forward by dt seconds.
void PredictTrack(double dt, const NoiseParams& noise, MotionModel model, Track* track);

// Blends a matched detection into the track.
void UpdateTrack(const Detection& det, const NoiseParams& noise, Track* track);

// Records a frame where no detection matched this track.
void MarkMissed(Track* track);

// True if the track should appear in this frame's output.
bool ShouldReport(const Track& track, const TrackParams& params);

// True if the track has been missing too long and should be removed.
bool ShouldDelete(const Track& track, const TrackParams& params);

// The track's current belief in the submission writer's format.
TrackedBox ToTrackedBox(const Track& track, const std::string& sample_token);

}  // namespace mot3d