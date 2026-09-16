cmake_minimum_required(VERSION 3.16)
project(lunar_simulator LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
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

# Add the Alpha system and its localisation stack.
add_subdirectory(src/systems/alpha_system)
add_subdirectory(src/localisation)
install(
  FILES
    launch/alpha_system_launch.py
    launch/kalman_filter_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} PATTERN "*.bak" EXCLUDE)
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME} PATTERN "*.bak" EXCLUDE)
install(DIRECTORY src/systems/alpha_system/alpha_model/ DESTINATION share/${PROJECT_NAME}/alpha_model PATTERN "*.bak" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
