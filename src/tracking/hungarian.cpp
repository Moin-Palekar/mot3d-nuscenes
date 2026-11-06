#include "mot3d/tracking/hungarian.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace mot3d {
namespace {

// Stands in for "this pair is not allowed": far bigger than any real cost, so
// the algorithm only uses it when nothing better exists.
constexpr double kForbidden = 1e9;

}  // namespace

Assignment SolveAssignment(const CostMatrix& costs, double max_cost) {
  Assignment result;
  const int num_tracks = static_cast<int>(costs.size());
  const int num_detections = num_tracks == 0 ? 0 : static_cast<int>(costs[0].size());

  // Nothing to match: everything is unmatched.
  if (num_tracks == 0 || num_detections == 0) {
    for (int t = 0; t < num_tracks; ++t) {
      result.unmatched_tracks.push_back(t);
    }
    for (int d = 0; d < num_detections; ++d) {
      result.unmatched_detections.push_back(d);
    }
    return result;
  }

  // A square table, numbered from 1 (row 0 and column 0 are helpers the
  // algorithm needs). Padding and pairs over max_cost are forbidden.
  const int n = std::max(num_tracks, num_detections);
  std::vector<std::vector<double>> a(n + 1, std::vector<double>(n + 1, kForbidden));
  for (int t = 0; t < num_tracks; ++t) {
    for (int d = 0; d < num_detections; ++d) {
      const double c = costs[t][d];
      a[t + 1][d + 1] = c <= max_cost ? c : kForbidden;
    }
  }

  // The Hungarian algorithm. u and v are "prices" on rows and columns; p[j] is
  // the row currently assigned to column j; way[j] remembers how column j was
  // reached, so a better chain of assignments can be traced back and swapped in.
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0);
  std::vector<int> way(n + 1, 0);
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, kInf);
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = kInf;
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = a[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  // Read out the real pairs that are within max_cost.
  std::vector<bool> track_matched(num_tracks, false);
  std::vector<bool> detection_matched(num_detections, false);
  for (int j = 1; j <= num_detections; ++j) {
    const int i = p[j];
    if (i >= 1 && i <= num_tracks && costs[i - 1][j - 1] <= max_cost) {
      result.matches.emplace_back(i - 1, j - 1);
      track_matched[i - 1] = true;
      detection_matched[j - 1] = true;
    }
  }
  std::sort(result.matches.begin(), result.matches.end());
  for (int t = 0; t < num_tracks; ++t) {
    if (!track_matched[t]) {
      result.unmatched_tracks.push_back(t);
    }
  }
  for (int d = 0; d < num_detections; ++d) {
    if (!detection_matched[d]) {
      result.unmatched_detections.push_back(d);
    }
  }
  return result;
}

}  // namespace mot3d