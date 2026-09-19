cmake_minimum_required(VERSION 3.16)

add_library(ground_truth STATIC
  src/methods/stampToNanoseconds.cc
  src/methods/isPoseValid.cc
  src/methods/handleOdometryCallBack.cc
  src/methods/appendPathPose.cc
  src/methods/publishTransform.cc
)

target_compile_features(ground_truth PUBLIC cxx_std_17)

target_include_directories(ground_truth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

ament_target_dependencies(ground_truth
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS ground_truth ARCHIVE DESTINATION lib)
