include(FetchContent)
# Remove unnecessary build files
# and steps from GTest's CMake
option(BUILD_GMOCK OFF)
option(INSTALL_GTEST OFF)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0
)
FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

# https://google.github.io/googletest/quickstart-cmake.html

add_executable(util_solution_test util_solution_test.cpp)
target_link_libraries(
  util_solution_test
  GTest::gtest_main
)

include(GoogleTest)

# Include tests
gtest_discover_tests(util_solution_test)

# add_executable(play playground.cpp)