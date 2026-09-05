cmake_minimum_required(VERSION 3.22)
project(mot3d_nuscenes VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Emit compile_commands.json so clangd knows how each file is built.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(MOT3D_BUILD_TESTS "Build unit tests" ON)

add_compile_options(-Wall -Wextra -Wpedantic)

add_subdirectory(src)

include(FetchContent)

# Pinned rather than using the system Eigen: Homebrew ships 5.x, Ubuntu 22.04
# ships 3.4. Fetching a fixed version keeps local, CI, and Docker identical.
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.11.3
  GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(nlohmann_json)

if(MOT3D_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
