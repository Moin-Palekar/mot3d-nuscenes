# Core library. Deliberately has NO ROS dependency — the ROS 2 node in
# ros2/ links against this, not the other way round.
add_library(mot3d_core INTERFACE)

target_include_directories(mot3d_core INTERFACE
  ${CMAKE_SOURCE_DIR}/include
)
