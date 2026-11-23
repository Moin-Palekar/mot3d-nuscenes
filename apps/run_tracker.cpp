// Runs the tracker over every nuScenes scene covered by a detection file and
// writes the result in the official nuScenes tracking submission format.
//
// Usage: run_tracker <detections.json> <table_dir> <submission.json> [config.json]
// Without a config file, every setting keeps its default value.

#include <cstddef>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mot3d/io/detection_loader.h"
#include "mot3d/io/run_config.h"
#include "mot3d/io/scene_loader.h"
#include "mot3d/io/submission_writer.h"
#include "mot3d/tracking/tracker.h"

int main(int argc, char** argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: " << argv[0]
              << " <detections.json> <table_dir> <submission.json> [config.json]\n";
    return 2;
  }
  const std::string detections_path = argv[1];
  const std::string table_dir = argv[2];
  const std::string output_path = argv[3];

  std::string error;
  mot3d::RunConfig config;  // defaults, unless a config file overrides them
  if (argc == 5 && !mot3d::LoadRunConfig(argv[4], &config, &error)) {
    std::cerr << "loading the config failed: " << error << "\n";
    return 1;
  }

  std::unordered_map<std::string, mot3d::Frame> frames;
  if (!mot3d::LoadDetections(detections_path, config.load, &frames, &error)) {
    std::cerr << "loading detections failed: " << error << "\n";
    return 1;
  }
  std::vector<mot3d::Scene> scenes;
  if (!mot3d::LoadScenes(table_dir, &scenes, &error)) {
    std::cerr << "loading scenes failed: " << error << "\n";
    return 1;
  }

  mot3d::Tracker tracker{config.tracker};
  std::unordered_map<std::string, std::vector<mot3d::TrackedBox>> results;
  std::size_t scenes_run = 0;
  std::size_t boxes_written = 0;

  for (const mot3d::Scene& scene : scenes) {
    // Only scenes covered by the detection file (the 150 val scenes) are tracked.
    if (scene.frames.empty() || frames.count(scene.frames.front().sample_token) == 0) {
      continue;
    }

    tracker.Reset();  // tracks never carry across scenes
    double previous_time = scene.frames.front().timestamp;
    for (const mot3d::SceneFrame& scene_frame : scene.frames) {
      const auto it = frames.find(scene_frame.sample_token);
      if (it == frames.end()) {
        std::cerr << scene.name << ": frame " << scene_frame.sample_token
                  << " is missing from the detection file\n";
        return 1;
      }
      const double dt = scene_frame.timestamp - previous_time;  // 0 on the first frame
      previous_time = scene_frame.timestamp;

      std::vector<mot3d::TrackedBox> boxes =
          tracker.Step(it->second.detections, dt, scene_frame.sample_token);
      boxes_written += boxes.size();
      results[scene_frame.sample_token] = std::move(boxes);
    }
    ++scenes_run;
  }

  // The scorer rejects a submission whose frames differ from the split, so
  // every frame in the detection file must have been tracked.
  if (results.size() != frames.size()) {
    std::cerr << "tracked " << results.size() << " frames, but the detection file has "
              << frames.size() << "\n";
    return 1;
  }

  if (!mot3d::WriteSubmission(output_path, results, &error)) {
    std::cerr << "writing the submission failed: " << error << "\n";
    return 1;
  }

  std::cout << "scenes: " << scenes_run << "\n"
            << "frames: " << results.size() << "\n"
            << "boxes:  " << boxes_written << "\n"
            << "wrote " << output_path << "\n";
  return 0;
}