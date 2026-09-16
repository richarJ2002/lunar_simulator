cmake_minimum_required(VERSION 3.16)

add_executable(visual_odometry_node src/visual_odometry_node.cpp)
target_compile_features(visual_odometry_node PUBLIC cxx_std_17)
target_include_directories(visual_odometry_node SYSTEM PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(visual_odometry_node
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(visual_odometry_node ${OpenCV_LIBRARIES})
install(TARGETS visual_odometry_node DESTINATION lib/${PROJECT_NAME})
