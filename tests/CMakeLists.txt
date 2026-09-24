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
)

target_link_libraries(mot3d_tests PRIVATE
  mot3d_core
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(mot3d_tests)
