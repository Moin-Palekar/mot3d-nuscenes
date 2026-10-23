#pragma once

#include <array>

#include <Eigen/Geometry>

namespace mot3d {

// A quaternion in nuScenes order: [w, x, y, z].
using QuaternionWxyz = std::array<double, 4>;

// Heading around the vertical axis, in radians, range (-pi, pi].
// 0 = facing +x (east in the global frame), positive = turning left.
double YawFromQuaternion(const QuaternionWxyz& q);

// The rotation by `yaw` around the vertical axis, as [w, x, y, z].
QuaternionWxyz QuaternionFromYaw(double yaw);

// A rigid transform: rotate, then shift. Maps a point from a "child" frame
// (for example the car) into a "parent" frame (for example the world).
struct Pose {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

// Builds a Pose from nuScenes fields: rotation [w, x, y, z], translation [x, y, z].
Pose MakePose(const QuaternionWxyz& rotation, const std::array<double, 3>& translation);

// Moves a point from the pose's child frame into its parent frame.
Eigen::Vector3d Apply(const Pose& pose, const Eigen::Vector3d& point);

// Chains two poses: apply `inner` first, then `outer`.
Pose Compose(const Pose& outer, const Pose& inner);

// The reverse pose: maps parent-frame points back into the child frame.
Pose Inverse(const Pose& pose);

}  // namespace mot3d