#include "mot3d/io/scene_loader.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

namespace mot3d {
namespace {

using Json = nlohmann::json;

// sample.json stores timestamps in microseconds; everything downstream uses
// seconds. This is the only place the conversion happens.
constexpr double kMicrosecondsPerSecond = 1e6;

// The two fields of a sample record the chain walk needs.
struct SampleRecord {
  std::int64_t timestamp_us = 0;
  std::string next;
};

// Reads one JSON file into *out. Returns false and sets *error on failure.
bool ReadJson(const std::string& path, Json* out, std::string* error) {
  std::ifstream file(path);
  if (!file.is_open()) {
    *error = "could not open " + path;
    return false;
  }
  try {
    file >> *out;
  } catch (const Json::parse_error& e) {
    *error = "failed to parse " + path + ": " + e.what();
    return false;
  }
  return true;
}

}  // namespace

bool LoadScenes(const std::string& table_dir, std::vector<Scene>* out, std::string* error) {
  Json scene_table;
  Json sample_table;
  if (!ReadJson(table_dir + "/scene.json", &scene_table, error)) return false;
  if (!ReadJson(table_dir + "/sample.json", &sample_table, error)) return false;

  try {
    // Index every sample by its token, so each step of the chain walk is a
    // direct lookup instead of a search through 34k records.
    std::unordered_map<std::string, SampleRecord> samples;
    samples.reserve(sample_table.size());
    for (const auto& s : sample_table) {
      samples[s.at("token").get<std::string>()] =
          SampleRecord{s.at("timestamp").get<std::int64_t>(), s.at("next").get<std::string>()};
    }

    out->clear();
    out->reserve(scene_table.size());

    for (const auto& sc : scene_table) {
      Scene scene;
      scene.token = sc.at("token").get<std::string>();
      scene.name = sc.at("name").get<std::string>();
      const auto expected = sc.at("nbr_samples").get<std::size_t>();
      scene.frames.reserve(expected);

      // Walk the chain: start at the first sample, follow `next` until it is
      // empty. The file order of sample.json is never trusted.
      std::string token = sc.at("first_sample_token").get<std::string>();
      while (!token.empty()) {
        // Stops a chain that loops back on itself from running forever.
        if (scene.frames.size() == expected) {
          *error =
              scene.name + ": chain is longer than nbr_samples (" + std::to_string(expected) + ")";
          return false;
        }

        const auto it = samples.find(token);
        if (it == samples.end()) {
          *error = scene.name + ": sample " + token + " not found in sample.json";
          return false;
        }

        const double t = static_cast<double>(it->second.timestamp_us) / kMicrosecondsPerSecond;
        if (!scene.frames.empty() && t <= scene.frames.back().timestamp) {
          *error = scene.name + ": timestamps do not increase at sample " + token;
          return false;
        }

        scene.frames.push_back(SceneFrame{token, t});
        token = it->second.next;
      }

      if (scene.frames.size() != expected) {
        *error = scene.name + ": chain has " + std::to_string(scene.frames.size()) +
                 " samples but nbr_samples is " + std::to_string(expected);
        return false;
      }

      out->push_back(std::move(scene));
    }
  } catch (const Json::exception& e) {
    // A missing field or a wrong type anywhere in the tables lands here,
    // instead of crashing the program.
    *error = std::string("malformed nuScenes table: ") + e.what();
    return false;
  }

  return true;
}

}  // namespace mot3d