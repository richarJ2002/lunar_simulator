cmake_minimum_required(VERSION 3.16)

add_library(alpha_kalman_filter STATIC
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/correct.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/init.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/predictStep.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/predictTo.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/step.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/terminate.cc
  src/kalman_filter/methods/ContinuousExtendedKalmanFilter/wrapAngle.cc
)
target_compile_features(alpha_kalman_filter PUBLIC cxx_std_17)
target_include_directories(alpha_kalman_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(alpha_kalman_filter PUBLIC Eigen3::Eigen)

add_executable(kalman_filter_node src/kalman_filter_node.cpp)
target_compile_features(kalman_filter_node PUBLIC cxx_std_17)
target_link_libraries(kalman_filter_node alpha_kalman_filter Eigen3::Eigen)
ament_target_dependencies(kalman_filter_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS alpha_kalman_filter kalman_filter_node
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
