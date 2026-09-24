#include "mot3d/io/detection_loader.h"

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

namespace mot3d {
namespace {

using Json = nlohmann::json;

// nuScenes stores orientation as a quaternion, but boxes are gravity aligned,
// so the only meaningful component is rotation about the vertical axis. This
// extracts that angle.
//
// The quaternion in the file is [w, x, y, z]. Note that q and -q describe the
// same rotation, so a negative w is not an error and the formula handles it.
double YawFromQuaternion(double w, double x, double y, double z) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

bool LoadDetections(const std::string& path, const LoadOptions& options,
                    std::unordered_map<std::string, Frame>* out, std::string* error) {
  std::ifstream file(path);
  if (!file.is_open()) {
    *error = "could not open detection file: " + path;
    return false;
  }

  Json root;
  try {
    file >> root;
  } catch (const Json::parse_error& e) {
    *error = std::string("failed to parse detection JSON: ") + e.what();
    return false;
  }

  if (!root.contains("results")) {
    *error = "detection file has no 'results' key";
    return false;
  }

  out->clear();
  out->reserve(root["results"].size());

  for (const auto& [sample_token, boxes] : root["results"].items()) {
    Frame frame;
    frame.sample_token = sample_token;

    for (const auto& box : boxes) {
      ClassId cls;
      // Rejects barrier, traffic_cone and construction_vehicle, which the
      // detector emits but nuScenes does not evaluate for tracking.
      if (!ClassFromString(box["detection_name"].get<std::string>(), &cls)) {
        continue;
      }

      const double score = box["detection_score"].get<double>();
      if (score < options.min_score) {
        continue;
      }

      Detection det;
      det.sample_token = sample_token;
      det.cls = cls;
      det.score = score;

      const auto& t = box["translation"];
      det.center = Eigen::Vector3d(t[0].get<double>(), t[1].get<double>(), t[2].get<double>());

      // nuScenes order is [width, length, height], not [length, width, height].
      const auto& s = box["size"];
      det.size = Eigen::Vector3d(s[0].get<double>(), s[1].get<double>(), s[2].get<double>());

      const auto& r = box["rotation"];
      det.yaw = YawFromQuaternion(r[0].get<double>(), r[1].get<double>(), r[2].get<double>(),
                                  r[3].get<double>());

      // Velocity may be NaN in the file. NaN propagates through every
      // arithmetic operation it touches, so a single one entering the filter
      // would corrupt the whole track state.
      const double vx = box["velocity"][0].get<double>();
      const double vy = box["velocity"][1].get<double>();
      if (std::isfinite(vx) && std::isfinite(vy)) {
        det.velocity = Eigen::Vector2d(vx, vy);
        det.velocity_valid = true;
      }

      frame.detections.push_back(det);
    }

    // Frames are kept even when empty. The tracker still has to step time
    // forward so existing tracks age correctly.
    (*out)[sample_token] = std::move(frame);
  }

  return true;
}

}  // namespace mot3d