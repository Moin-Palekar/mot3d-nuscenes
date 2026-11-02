#include "mot3d/tracking/cost.h"

#include <cmath>

namespace mot3d {

Measurement ToMeasurement(const Detection& det) {
  Measurement z;
  z << det.center.x(), det.center.y(), det.center.z(), det.yaw;
  return z;
}

double EuclideanCost(const FilterState& filter, const Detection& det) {
  const double dx = det.center.x() - filter.mean[kX];
  const double dy = det.center.y() - filter.mean[kY];
  return std::hypot(dx, dy);
}

double MahalanobisCost(const FilterState& filter, const Detection& det, const NoiseParams& noise) {
  const Innovation innovation = ComputeInnovation(filter, ToMeasurement(det), noise);
  // residualᵀ · innovation.cov⁻¹ · residual: the squared gap, scaled by how
  // big a gap was expected in each direction.
  const double squared = innovation.residual.dot(innovation.cov.inverse() * innovation.residual);
  return std::sqrt(squared);
}

}  // namespace mot3d