cmake_minimum_required(VERSION 3.16)

add_library(wheel_odometry STATIC
  src/methods/loadSixValues.cc
  src/methods/stampToSeconds.cc
  src/methods/findJoint.cc
  src/methods/handleVisualOdometryCallBack.cc
  src/methods/publishSlipObservation.cc
  src/methods/publishSlipRatios.cc
  src/methods/handleWheelSlipEstimateCallBack.cc
  src/methods/handleKalmanFilterCallBack.cc
  src/methods/handleInitialPoseCallBack.cc
  src/methods/handleJointStateCallBack.cc
  src/methods/wrapAngle.cc
  src/methods/publishOdometry.cc
)
target_compile_features(wheel_odometry PUBLIC cxx_std_17)
target_include_directories(wheel_odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(wheel_odometry Eigen3::Eigen)
ament_target_dependencies(wheel_odometry
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS wheel_odometry ARCHIVE DESTINATION lib)
