#pragma once

#include "mot3d/io/detection.h"
#include "mot3d/tracking/kalman_filter.h"

namespace mot3d {

// A detection's 4 measured numbers (x, y, z, yaw), in the order the Kalman
// filter expects.
Measurement ToMeasurement(const Detection& det);

// Straight-line distance on the ground (east, north) between the track's
// predicted centre and the detection's centre, in metres.
double EuclideanCost(const FilterState& filter, const Detection& det);

// How far the detection is from the prediction, counted in "typical gaps":
// the gap divided by how big a gap the filter expected. 1.0 means an ordinary
// gap; 3.0 means three times bigger than ordinary, which is very surprising.
double MahalanobisCost(const FilterState& filter, const Detection& det, const NoiseParams& noise);

}  // namespace mot3d