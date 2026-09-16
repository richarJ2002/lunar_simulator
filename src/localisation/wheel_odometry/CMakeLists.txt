cmake_minimum_required(VERSION 3.16)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_compile_features(wheel_odometry_node PUBLIC cxx_std_17)
target_link_libraries(wheel_odometry_node Eigen3::Eigen)
ament_target_dependencies(wheel_odometry_node
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
install(TARGETS wheel_odometry_node DESTINATION lib/${PROJECT_NAME})
