#pragma once

#include <utility>
#include <vector>

namespace mot3d {

// costs[t][d] is the cost of pairing track t with detection d. Lower is better.
// Every row has the same length: one entry per detection.
using CostMatrix = std::vector<std::vector<double>>;

// The result of matching tracks to detections.
struct Assignment {
  std::vector<std::pair<int, int>> matches;  // (track index, detection index)
  std::vector<int> unmatched_tracks;         // tracks that got no detection
  std::vector<int> unmatched_detections;     // detections that got no track
};

// With zero tracks the table has no rows, so the detection count is unknown
// and unmatched_detections comes back empty. Callers must treat every
// detection as unmatched in that case.
Assignment SolveAssignment(const CostMatrix& costs, double max_cost);

}  // namespace mot3d