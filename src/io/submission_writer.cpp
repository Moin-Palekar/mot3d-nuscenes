#include "mot3d/io/submission_writer.h"

#include <cmath>
#include <fstream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace mot3d {
namespace {

using Json = nlohmann::json;

// Converts one tracked box into one box in the scorer's JSON format.
Json BoxToJson(const std::string& sample_token, const TrackedBox& tracked) {
  const Detection& d = tracked.box;

  // nuScenes boxes only turn about the vertical axis. For a heading of `yaw`,
  // that rotation as a quaternion [w, x, y, z] is [cos(yaw/2), 0, 0, sin(yaw/2)].
  const double half_yaw = d.yaw / 2.0;

  Json box;
  box["sample_token"] = sample_token;
  box["translation"] = {d.center.x(), d.center.y(), d.center.z()};
  box["size"] = {d.size[0], d.size[1], d.size[2]};  // [width, length, height]
  box["rotation"] = {std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)};
  if (d.velocity_valid) {
    box["velocity"] = {d.velocity.x(), d.velocity.y()};
  } else {
    box["velocity"] = {0.0, 0.0};
  }
  box["tracking_id"] = std::to_string(tracked.track_id);
  box["tracking_name"] = ToString(d.cls);
  box["tracking_score"] = d.score;
  return box;
}

}  // namespace

bool WriteSubmission(const std::string& path,
                     const std::unordered_map<std::string, std::vector<TrackedBox>>& results,
                     std::string* error) {
  // Every frame gets an entry, even with no boxes. Starting each list as an
  // explicit empty array makes an empty frame come out as [] in the file.
  Json results_json = Json::object();
  for (const auto& [sample_token, boxes] : results) {
    Json box_list = Json::array();
    for (const auto& tracked : boxes) {
      box_list.push_back(BoxToJson(sample_token, tracked));
    }
    results_json[sample_token] = std::move(box_list);
  }

  Json root;
  root["meta"] = {
      {"use_camera", false}, {"use_lidar", true},     {"use_radar", false},
      {"use_map", false},    {"use_external", false},
  };
  root["results"] = std::move(results_json);

  std::ofstream file(path);
  if (!file.is_open()) {
    *error = "could not open " + path + " for writing";
    return false;
  }
  file << root.dump();
  if (!file) {
    *error = "failed while writing " + path;
    return false;
  }
  return true;
}

}  // namespace mot3d