cmake_minimum_required(VERSION 3.16)

add_executable(inertial_odometry_node src/inertial_odometry_node.cpp)
target_compile_features(inertial_odometry_node PUBLIC cxx_std_17)
ament_target_dependencies(inertial_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS inertial_odometry_node DESTINATION lib/${PROJECT_NAME})
