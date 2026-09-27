include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(mot3d_tests
  test_smoke.cpp
  test_detection.cpp
  test_detection_loader.cpp
)

target_link_libraries(mot3d_tests PRIVATE
  mot3d_core
  GTest::gtest_main
)

# Absolute paths for tests that read files. ctest runs the binary from inside
# build/, so relative paths would not resolve.
target_compile_definitions(mot3d_tests PRIVATE
  MOT3D_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
  MOT3D_REPO_DIR="${PROJECT_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(mot3d_tests)
