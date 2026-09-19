cmake_minimum_required(VERSION 3.16)
project(lunar_simulator LANGUAGES CXX)

# Default to an optimized build when the invoker (a bare `colcon build`,
# per this repository's own documented build command) does not request a
# build type. This project's OpenCV-based nodes were previously
# insensitive to this: OpenCV itself is a separately pre-built,
# pre-optimized library, so an unoptimized CMAKE_BUILD_TYPE only affected
# this project's own thin wrapper code around it. That stopped being true
# once `alpha_feature_tracking` (src/localisation/visual_odometry/src/
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
# -Wextra, -Wpedantic and -Wshadow are clean across all project-owned code.
# -Wconversion additionally reports a `long int` -> `int` narrowing at every
# `declare_parameter<int>(...)` call site (path_maximum_poses, num_disparities,
# block_size, maximum_features, minimum_correspondences): the narrowing occurs
# inside rclcpp::Node::declare_parameter's own template body, which converts
# ROS 2's underlying int64_t parameter storage to the requested `int`, not in
# any project-owned expression. Per Section 15.3 ("Do not apply project
# warning-as-error flags to third-party headers"), this residual warning class
# is accepted rather than silenced with a cast; -Werror is intentionally not
# enabled here for that reason.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Localisation and control build first: both produce the static libraries
# the Alpha system's composed entry point (src/systems/alpha/
# alpha_node/main.cpp) links against, and CMake requires a target to
# already exist when a later add_subdirectory() links against it.
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
        worlds 
    DESTINATION share/${PROJECT_NAME} 
    PATTERN "*.bak" EXCLUDE
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
