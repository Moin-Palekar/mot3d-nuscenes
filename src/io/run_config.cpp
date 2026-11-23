#include "mot3d/io/run_config.h"

#include <fstream>
#include <set>
#include <string>

#include <nlohmann/json.hpp>

namespace mot3d {
namespace {

using Json = nlohmann::json;

// Fails if `object` has a key that is not in `allowed`.
bool CheckKeys(const Json& object, const std::set<std::string>& allowed, const std::string& where,
               std::string* error) {
  for (const auto& item : object.items()) {
    if (allowed.count(item.key()) == 0) {
      *error = "unknown key \"" + item.key() + "\" in " + where;
      return false;
    }
  }
  return true;
}

bool ParseMotionModel(const std::string& name, MotionModel* out) {
  if (name == "ctrv") {
    *out = MotionModel::kCtrv;
    return true;
  }
  if (name == "cv") {
    *out = MotionModel::kCv;
    return true;
  }
  return false;
}

bool ParseCostType(const std::string& name, CostType* out) {
  if (name == "mahalanobis") {
    *out = CostType::kMahalanobis;
    return true;
  }
  if (name == "euclidean") {
    *out = CostType::kEuclidean;
    return true;
  }
  if (name == "iou") {
    *out = CostType::kIou;
    return true;
  }
  return false;
}

}  // namespace

bool LoadRunConfig(const std::string& path, RunConfig* out, std::string* error) {
  std::ifstream file(path);
  if (!file) {
    *error = "cannot open " + path;
    return false;
  }
  try {
    const Json root = Json::parse(file);
    if (!root.is_object()) {
      *error = path + ": the top level must be a JSON object";
      return false;
    }
    if (!CheckKeys(root,
                   {"min_score", "motion_model", "cost", "max_cost", "min_hits", "max_misses",
                    "noise", "use_velocity"},
                   path, error)) {
      return false;
    }

    RunConfig config;  // starts from every default; the file only overrides
    config.load.min_score = root.value("min_score", config.load.min_score);
    if (root.contains("motion_model") &&
        !ParseMotionModel(root.at("motion_model").get<std::string>(),
                          &config.tracker.motion_model)) {
      *error = path + ": motion_model must be \"ctrv\" or \"cv\"";
      return false;
    }
    if (root.contains("cost") &&
        !ParseCostType(root.at("cost").get<std::string>(), &config.tracker.cost_type)) {
      *error = path + ": cost must be \"mahalanobis\", \"euclidean\" or \"iou\"";
      return false;
    }
    config.tracker.max_cost = root.value("max_cost", config.tracker.max_cost);
    config.tracker.track.min_hits = root.value("min_hits", config.tracker.track.min_hits);
    config.tracker.track.max_misses = root.value("max_misses", config.tracker.track.max_misses);
    config.tracker.use_velocity = root.value("use_velocity", config.tracker.use_velocity);

    if (root.contains("noise")) {
      const Json& noise = root.at("noise");
      if (!noise.is_object()) {
        *error = path + ": noise must be a JSON object";
        return false;
      }
      if (!CheckKeys(noise,
                     {"accel_std", "yaw_accel_std", "z_drift_std", "pos_std", "z_std", "yaw_std",
                      "init_speed_std", "init_yaw_rate_std", "speed_std"},
                     path + " (noise)", error)) {
        return false;
      }
      NoiseParams& params = config.tracker.noise;
      params.accel_std = noise.value("accel_std", params.accel_std);
      params.yaw_accel_std = noise.value("yaw_accel_std", params.yaw_accel_std);
      params.z_drift_std = noise.value("z_drift_std", params.z_drift_std);
      params.pos_std = noise.value("pos_std", params.pos_std);
      params.z_std = noise.value("z_std", params.z_std);
      params.yaw_std = noise.value("yaw_std", params.yaw_std);
      params.init_speed_std = noise.value("init_speed_std", params.init_speed_std);
      params.init_yaw_rate_std = noise.value("init_yaw_rate_std", params.init_yaw_rate_std);
      params.speed_std = noise.value("speed_std", params.speed_std);
    }

    *out = config;
    return true;
  } catch (const Json::exception& e) {
    // Bad JSON syntax, or a value of the wrong type such as "min_hits": "two".
    *error = path + ": " + e.what();
    return false;
  }
}

}  // namespace mot3d