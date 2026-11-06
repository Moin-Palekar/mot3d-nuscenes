#include <limits>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mot3d/tracking/hungarian.h"

namespace mot3d {
namespace {

using Pairs = std::vector<std::pair<int, int>>;

constexpr double kNoGate = std::numeric_limits<double>::max();

double TotalCost(const CostMatrix& costs, const Pairs& pairs) {
  double total = 0.0;
  for (const auto& [t, d] : pairs) {
    total += costs[t][d];
  }
  return total;
}

// The best total cost found by trying every possible one-to-one pairing.
// Slow, but obviously correct, which is what a reference needs to be.
double BruteForceBest(const CostMatrix& costs, int t, std::vector<bool>& detection_used,
                      int pairs_left) {
  if (pairs_left == 0) {
    return 0.0;
  }
  const int num_tracks = static_cast<int>(costs.size());
  const int num_detections = static_cast<int>(costs[0].size());
  if (t == num_tracks) {
    return std::numeric_limits<double>::infinity();
  }
  // Option 1: track t takes no detection (only useful when tracks outnumber detections).
  double best = BruteForceBest(costs, t + 1, detection_used, pairs_left);
  // Option 2: track t takes any detection still free.
  for (int d = 0; d < num_detections; ++d) {
    if (!detection_used[d]) {
      detection_used[d] = true;
      best = std::min(best,
                      costs[t][d] + BruteForceBest(costs, t + 1, detection_used, pairs_left - 1));
      detection_used[d] = false;
    }
  }
  return best;
}

TEST(Hungarian, NoDetectionsLeavesEveryTrackUnmatched) {
  const CostMatrix costs = {{}, {}};  // 2 tracks, 0 detections
  const Assignment result = SolveAssignment(costs, 5.0);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_EQ(result.unmatched_tracks, (std::vector<int>{0, 1}));
  EXPECT_TRUE(result.unmatched_detections.empty());
}

TEST(Hungarian, PicksObviousPairsAndLeavesExtraDetection) {
  const CostMatrix costs = {{1.0, 2.6, 4.7},   // track 0
                            {2.4, 0.9, 4.2}};  // track 1
  const Assignment result = SolveAssignment(costs, 3.0);
  EXPECT_EQ(result.matches, (Pairs{{0, 0}, {1, 1}}));
  EXPECT_TRUE(result.unmatched_tracks.empty());
  EXPECT_EQ(result.unmatched_detections, (std::vector<int>{2}));
}

TEST(Hungarian, BeatsGreedyMatching) {
  // Greedy takes the cheapest pair first (0,0) = 1.0, which forces (1,1) = 10.0,
  // a total of 11.0. The best is (0,1) + (1,0) = 1.1 + 1.1 = 2.2.
  const CostMatrix costs = {{1.0, 1.1}, {1.1, 10.0}};
  const Assignment result = SolveAssignment(costs, kNoGate);
  EXPECT_EQ(result.matches, (Pairs{{0, 1}, {1, 0}}));
  EXPECT_NEAR(TotalCost(costs, result.matches), 2.2, 1e-9);
}

TEST(Hungarian, RefusesPairsOverMaxCost) {
  // Only (0,0) is within max_cost = 3. Track 1 and detection 1 stay unmatched
  // rather than being forced together at cost 9.
  const CostMatrix costs = {{1.0, 9.0}, {9.0, 9.0}};
  const Assignment result = SolveAssignment(costs, 3.0);
  EXPECT_EQ(result.matches, (Pairs{{0, 0}}));
  EXPECT_EQ(result.unmatched_tracks, (std::vector<int>{1}));
  EXPECT_EQ(result.unmatched_detections, (std::vector<int>{1}));
}

TEST(Hungarian, MoreTracksThanDetections) {
  // 3 tracks, 1 detection: the detection goes to the track it fits best.
  const CostMatrix costs = {{5.0}, {1.0}, {3.0}};
  const Assignment result = SolveAssignment(costs, kNoGate);
  EXPECT_EQ(result.matches, (Pairs{{1, 0}}));
  EXPECT_EQ(result.unmatched_tracks, (std::vector<int>{0, 2}));
  EXPECT_TRUE(result.unmatched_detections.empty());
}

TEST(Hungarian, MatchesBruteForceOnRandomTables) {
  std::mt19937 rng(42);  // fixed seed: the same "random" tables every run
  std::uniform_int_distribution<int> size(1, 5);
  std::uniform_real_distribution<double> cost(0.0, 10.0);
  for (int trial = 0; trial < 200; ++trial) {
    const int num_tracks = size(rng);
    const int num_detections = size(rng);
    CostMatrix costs(num_tracks, std::vector<double>(num_detections));
    for (auto& row : costs) {
      for (double& c : row) {
        c = cost(rng);
      }
    }
    const Assignment result = SolveAssignment(costs, kNoGate);
    std::vector<bool> used(num_detections, false);
    const double best = BruteForceBest(costs, 0, used, std::min(num_tracks, num_detections));
    EXPECT_NEAR(TotalCost(costs, result.matches), best, 1e-9) << "trial " << trial;
  }
}

}  // namespace
}  // namespace mot3d