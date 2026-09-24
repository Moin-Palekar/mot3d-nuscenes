#include "mot3d/io/detection.h"

#include <array>

namespace mot3d {
namespace {

// Indexed by ClassId. Order must match the enum exactly. These strings are
// what nuScenes uses in both the detection file and the submission format,
// so they are not free to change.
constexpr std::array<const char*, kNumClasses> kClassNames = {
    "bicycle", "bus", "car", "motorcycle", "pedestrian", "trailer", "truck",
};

}  // namespace

const char* ToString(ClassId cls) {
  const auto index = static_cast<std::size_t>(cls);
  if (index >= kNumClasses) {
    return "unknown";
  }
  return kClassNames[index];
}

bool ClassFromString(const std::string& name, ClassId* out) {
  for (std::size_t i = 0; i < kNumClasses; ++i) {
    if (name == kClassNames[i]) {
      *out = static_cast<ClassId>(i);
      return true;
    }
  }
  // Reached for barrier, traffic_cone and construction_vehicle, which the
  // detector emits but nuScenes does not evaluate for tracking. Returning
  // false lets the loader skip the box rather than treating it as an error.
  return false;
}

}  // namespace mot3d