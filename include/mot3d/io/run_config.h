#pragma once

#include <string>

#include "mot3d/io/detection_loader.h"
#include "mot3d/tracking/tracker.h"

namespace mot3d {

// Every setting one tracking run needs: what to load and how to track.
struct RunConfig {
  LoadOptions load;
  TrackerParams tracker;
};

// Reads a JSON config file. Keys that are missing keep their default values,
// so a config only needs to list what it changes. Unknown keys are an error:
// a misspelled key would otherwise be silently ignored and waste a whole run.
//
// Returns false and sets *error on failure.
bool LoadRunConfig(const std::string& path, RunConfig* out, std::string* error);

}  // namespace mot3d