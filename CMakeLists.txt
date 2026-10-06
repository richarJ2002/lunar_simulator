cmake_minimum_required(VERSION 3.16)
project(space_robotics_simulator LANGUAGES CXX)

# Default to an optimized build when the invoker (a bare `colcon build`,
# per this repository's own documented build command) does not request a
# build type. This project's OpenCV-based nodes were previously
# insensitive to this: OpenCV itself is a separately pre-built,
# pre-optimized library, so an unoptimized CMAKE_BUILD_TYPE only affected
# this project's own thin wrapper code around it. That stopped being true
# once `alpha_feature_tracking` (src/localisation/visual_odometry/
# feature_tracking/) replaced the OpenCV corner-detection/optical-flow
# calls with equivalent code compiled as part of this project: measured
# live, an unoptimized build of that engine cannot keep pace with the
# LocCam sensor's 10 Hz publish rate (per-frame processing regularly
# exceeded 500 ms, causing the fusing EKF to discard every visual
# odometry update as stale), while a RelWithDebInfo build comfortably
# does (regularly ~110-150 ms). RelWithDebInfo (not Release) is chosen so
# debug symbols remain available without a separate rebuild.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Build type (defaulted; pass -DCMAKE_BUILD_TYPE=... to override)"
      FORCE)
endif()

# ISO C++17 without compiler extensions, per CPP_CODING_STANDARD.md Section
# 3.1. Individual targets additionally request cxx_std_17 via
# target_compile_features(); these project-wide settings make the baseline
# explicit and prevent an extensions-enabled compiler default from silently
# widening the accepted language.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Full warning baseline from CPP_CODING_STANDARD.md Section 15.3. -Wall,
# -Wextra, -Wpedantic, -Wconversion and -Wshadow are clean across all
# project-owned code. Integer ROS parameters are declared as std::int64_t,
# matching ROS 2's underlying int64_t parameter storage, so no narrowing
# occurs inside rclcpp::Node::declare_parameter's template body; values are
# narrowed with an explicit static_cast only after range validation at
# int-API boundaries (OpenCV StereoBM, feature-tracking engines, sparse
# matcher). -Werror is intentionally not enabled here per Section 15.3
# ("Do not apply project warning-as-error flags to third-party headers").
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Shared support modules (wrapped console logging, diagnostics entries)
# build first because every other library links them. Localisation and
# control follow: both produce the static libraries the Alpha system's
# composed entry point (src/systems/alpha/alpha_node/main.cpp) links
# against, and CMake requires a target to already exist when a later
# add_subdirectory() links against it.
add_subdirectory(src/common)
add_subdirectory(src/localisation)
add_subdirectory(src/control)
add_subdirectory(src/systems/alpha)

install(
  FILES
    launch/alpha_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY 
        config 
    DESTINATION share/${PROJECT_NAME} 
    PATTERN "*.bak" EXCLUDE
)

install(
    DIRECTORY 
        parameters 
    DESTINATION share/${PROJECT_NAME} 
    PATTERN "*.bak" EXCLUDE
)

install(
    DIRECTORY 
        environment 
    DESTINATION share/${PROJECT_NAME} 
    PATTERN "*.bak" EXCLUDE
    PATTERN "*.blend" EXCLUDE
    PATTERN "*.blend1" EXCLUDE
)

install(
    DIRECTORY 
        src/systems/alpha/alpha_model/ 
    DESTINATION share/${PROJECT_NAME}/alpha_model 
    PATTERN "*.bak" EXCLUDE
)

install(
  FILES
    src/systems/alpha/alpha.rviz
  DESTINATION share/${PROJECT_NAME}/alpha
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
