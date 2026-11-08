#include "mot3d/tracking/track.h"

#include <cmath>

#include "mot3d/tracking/cost.h"

namespace mot3d {

Track StartTrack(const Detection& det, int id, const NoiseParams& noise) {
  // Keep only the part of the detector's velocity that points along the heading.
  double speed = 0.0;
  if (det.velocity_valid) {
    speed = det.velocity.x() * std::cos(det.yaw) + det.velocity.y() * std::sin(det.yaw);
  }

  Track track;
  track.id = id;
  track.cls = det.cls;
  track.filter = InitFilter(ToMeasurement(det), speed, noise);
  track.size = det.size;
  track.score = det.score;
  track.hits = 1;
  track.misses = 0;
  return track;
}

void PredictTrack(double dt, const NoiseParams& noise, MotionModel model, Track* track) {
  PredictFilter(dt, noise, &track->filter, model);
}

void UpdateTrack(const Detection& det, const NoiseParams& noise, Track* track) {
  UpdateFilter(ToMeasurement(det), noise, &track->filter);
  track->size = det.size;
  track->score = det.score;
  ++track->hits;
  track->misses = 0;  // a match ends the missed-in-a-row streak
}

void MarkMissed(Track* track) {
  ++track->misses;
}

bool ShouldReport(const Track& track, const TrackParams& params) {
  return track.hits >= params.min_hits && track.misses == 0;
}

bool ShouldDelete(const Track& track, const TrackParams& params) {
  return track.misses > params.max_misses;
}

TrackedBox ToTrackedBox(const Track& track, const std::string& sample_token) {
  const State& mean = track.filter.mean;
  const double yaw = mean[kYaw];

  TrackedBox out;
  out.track_id = track.id;
  out.box.sample_token = sample_token;
  out.box.center = Eigen::Vector3d(mean[kX], mean[kY], mean[kZ]);
  out.box.size = track.size;
  out.box.yaw = yaw;
  out.box.velocity = mean[kSpeed] * Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
  out.box.velocity_valid = true;
  out.box.score = track.score;
  out.box.cls = track.cls;
  return out;
}

}  // namespace mot3d