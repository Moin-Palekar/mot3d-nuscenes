# Core library. Deliberately has NO ROS dependency — the ROS 2 node in
# ros2/ links against this, not the other way round.
add_library(mot3d_core
  io/detection.cpp
  io/detection_loader.cpp
  io/scene_loader.cpp
  io/submission_writer.cpp
  geometry/transform.cpp
  tracking/motion_model.cpp
  tracking/kalman_filter.cpp

)

target_include_directories(mot3d_core PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mot3d_core PUBLIC
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)