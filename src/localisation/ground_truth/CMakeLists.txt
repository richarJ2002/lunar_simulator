cmake_minimum_required(VERSION 3.16)

add_executable(ground_truth_node src/ground_truth_node.cpp)

target_compile_features(ground_truth_node PUBLIC cxx_std_17)

ament_target_dependencies(ground_truth_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS ground_truth_node DESTINATION lib/${PROJECT_NAME})
