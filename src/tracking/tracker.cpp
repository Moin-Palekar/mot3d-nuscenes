#include "mot3d/tracking/tracker.h"

#include <algorithm>
#include <limits>

#include "mot3d/tracking/cost.h"
#include "mot3d/tracking/hungarian.h"

namespace mot3d {
namespace {

// How badly a track fits a detection. Lower is better.
// Different classes never match: a car track must not take a pedestrian box.
double PairCost(const Track& track, const Detection& det, const TrackerParams& params) {
  if (track.cls != det.cls) {
    return std::numeric_limits<double>::infinity();
  }
  switch (params.cost_type) {
    case CostType::kEuclidean:
      return EuclideanCost(track.filter, det);
    case CostType::kIou:
      return IouCost(track, det);
    case CostType::kMahalanobis:
      break;
  }
  return MahalanobisCost(track.filter, det, params.noise);
}

}  // namespace

Tracker::Tracker(const TrackerParams& params) : params_(params) {}

std::vector<TrackedBox> Tracker::Step(const std::vector<Detection>& detections, double dt,
                                      const std::string& sample_token) {
  // 1. Predict: move every track to where it should be now.
  for (Track& track : tracks_) {
    PredictTrack(dt, params_.noise, params_.motion_model, &track);
  }

  // 2 and 3. Score every pair, then match.
  std::vector<int> new_detections;
  if (tracks_.empty()) {
    // With no tracks the table has no rows, so the solver cannot see the
    // detections. Every detection starts a new track.
    for (std::size_t d = 0; d < detections.size(); ++d) {
      new_detections.push_back(static_cast<int>(d));
    }
  } else {
    CostMatrix costs(tracks_.size(), std::vector<double>(detections.size()));
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      for (std::size_t d = 0; d < detections.size(); ++d) {
        costs[t][d] = PairCost(tracks_[t], detections[d], params_);
      }
    }
    const Assignment assignment = SolveAssignment(costs, params_.max_cost);

    // 4. Apply the result.
    for (const auto& [t, d] : assignment.matches) {
      UpdateTrack(detections[d], params_.noise, &tracks_[t]);
    }
    for (int t : assignment.unmatched_tracks) {
      MarkMissed(&tracks_[t]);
    }
    new_detections = assignment.unmatched_detections;
  }
  for (int d : new_detections) {
    tracks_.push_back(StartTrack(detections[d], next_id_++, params_.noise));
  }

  // 5. Delete tracks that have been missing too long.
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [this](const Track& track) { return ShouldDelete(track, params_.track); }),
      tracks_.end());

  // 6. Report the tracks that are confirmed and were seen this frame.
  std::vector<TrackedBox> reported;
  for (const Track& track : tracks_) {
    if (ShouldReport(track, params_.track)) {
      reported.push_back(ToTrackedBox(track, sample_token));
    }
  }
  return reported;
}

void Tracker::Reset() {
  tracks_.clear();
}

}  // namespace mot3d