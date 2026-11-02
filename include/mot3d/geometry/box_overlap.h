#pragma once

#include <Eigen/Core>

namespace mot3d {

// A 3D box that can turn around the vertical axis only, like every nuScenes box.
struct OrientedBox {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // metres, global frame
  Eigen::Vector3d size = Eigen::Vector3d::Zero();    // [width, length, height], nuScenes order
  double yaw = 0.0;                                  // heading; length runs along it
};

// How much two boxes overlap, from 0 (not touching) to 1 (identical):
// overlap volume / (volume of a + volume of b - overlap volume).
double BoxIou(const OrientedBox& a, const OrientedBox& b);

}  // namespace mot3d