#pragma once

#include <cstdint>
#include <string>

#include <Eigen/Core>

namespace mot3d {

// The 7 classes nuScenes evaluates for tracking. The detection file also
// contains barrier, traffic_cone and construction_vehicle; those are dropped
// at load time because the official scorer rejects submissions containing them.
enum class ClassId : std::uint8_t {
  kBicycle = 0,
  kBus,
  kCar,
  kMotorcycle,
  kPedestrian,
  kTrailer,
  kTruck,
  kCount,
};

// One detector output: a 3D box in a single frame, with no identity.
// Positions and orientations are in the nuScenes GLOBAL frame, which is what
// the detection file already provides.
struct Detection {
  // Which frame this came from. nuScenes calls these sample tokens.
  std::string sample_token;

  // Box centre in metres, global frame.
  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  // Box extents in metres, in nuScenes order: [width, length, height].
  // NOT [length, width, height] — mixing these up silently corrupts 3D IoU.
  Eigen::Vector3d size = Eigen::Vector3d::Zero();

  // Heading in radians, normalised to (-pi, pi]. Extracted from the file's
  // quaternion at load time: nuScenes boxes are gravity-aligned, so rotation
  // about the vertical axis is the only meaningful component.
  double yaw = 0.0;

  // Detector-estimated ground-plane velocity in m/s, global frame.
  // The file may contain NaN here; velocity_valid says whether it does.
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  bool velocity_valid = false;

  // Detector confidence, in [0, 1].
  double score = 0.0;

  ClassId cls = ClassId::kCar;
};

// Number of tracking classes. Used to size per-class parameter arrays.
inline constexpr std::size_t kNumClasses = static_cast<std::size_t>(ClassId::kCount);

// Name as it appears in the nuScenes detection and submission formats.
const char* ToString(ClassId cls);

// Parses a nuScenes class name. Returns false for the three detection-only
// classes and for anything unrecognised, so callers can skip those boxes.
bool ClassFromString(const std::string& name, ClassId* out);

}  // namespace mot3d