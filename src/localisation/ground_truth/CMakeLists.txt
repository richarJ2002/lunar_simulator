cmake_minimum_required(VERSION 3.16)

add_library(ground_truth STATIC
  methods/calculateFixedFromBody.cc
  methods/stampToNanoseconds.cc
  methods/isPoseValid.cc
  methods/handleOdometryCallBack.cc
  methods/appendPathPose.cc
  methods/publishTransform.cc
  methods/publishStartupFixedTransform.cc
)

target_compile_features(ground_truth PUBLIC cxx_std_17)

target_include_directories(ground_truth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ground_truth common_console)

ament_target_dependencies(ground_truth
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS ground_truth ARCHIVE DESTINATION lib)
