#include "mot3d/geometry/transform.h"

#include <cmath>

namespace mot3d {

double YawFromQuaternion(const QuaternionWxyz& q) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

QuaternionWxyz QuaternionFromYaw(double yaw) {
  const double half_yaw = yaw / 2.0;
  return {std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)};
}

Pose MakePose(const QuaternionWxyz& rotation, const std::array<double, 3>& translation) {
  Pose pose;
  pose.rotation =
      Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]).normalized();
  pose.translation = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  return pose;
}

Eigen::Vector3d Apply(const Pose& pose, const Eigen::Vector3d& point) {
  return pose.rotation * point + pose.translation;
}

Pose Compose(const Pose& outer, const Pose& inner) {
  Pose result;
  result.rotation = outer.rotation * inner.rotation;
  result.translation = outer.rotation * inner.translation + outer.translation;
  return result;
}

Pose Inverse(const Pose& pose) {
  Pose result;
  result.rotation = pose.rotation.conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

}  // namespace mot3d