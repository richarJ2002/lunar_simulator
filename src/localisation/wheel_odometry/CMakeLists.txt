cmake_minimum_required(VERSION 3.16)

add_library(wheel_odometry STATIC
  methods/loadSixValues.cc
  methods/stampToSeconds.cc
  methods/findJoint.cc
  methods/handleVisualOdometryCallBack.cc
  methods/publishSlipObservation.cc
  methods/publishSlipRatios.cc
  methods/handleWheelSlipEstimateCallBack.cc
  methods/handleJointStateCallBack.cc
  methods/wrapAngle.cc
  methods/publishDiagnosticsCallBack.cc
  methods/publishOdometry.cc
  public_functions/solveBodyTwist.cc
)
target_compile_features(wheel_odometry PUBLIC cxx_std_17)
target_include_directories(wheel_odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(wheel_odometry
  common_console
  common_diagnostics
  Eigen3::Eigen
)
ament_target_dependencies(wheel_odometry
  diagnostic_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS wheel_odometry ARCHIVE DESTINATION lib)
