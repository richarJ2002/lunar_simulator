cmake_minimum_required(VERSION 3.16)

add_library(inertial_odometry STATIC
  src/methods/calculateGravitySpecificForceFixed.cc
  src/methods/calculateGravityFreeAccelerationFixed.cc
  src/methods/handleImuCallBack.cc
  src/methods/publishFilteredImu.cc
  src/methods/publishOdometry.cc
  src/methods/stampToSeconds.cc
  src/methods/toMessage.cc
)
target_compile_features(inertial_odometry PUBLIC cxx_std_17)
target_include_directories(inertial_odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(inertial_odometry
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS inertial_odometry ARCHIVE DESTINATION lib)
