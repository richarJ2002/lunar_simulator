cmake_minimum_required(VERSION 3.16)

add_library(inertial_odometry STATIC
  methods/calculateGravitySpecificForceFixed.cc
  methods/calculateGravityFreeAccelerationFixed.cc
  methods/handleImuCallBack.cc
  methods/publishDiagnosticsCallBack.cc
  methods/publishFilteredImu.cc
  methods/publishOdometry.cc
  methods/stampToSeconds.cc
  methods/toMessage.cc
)
target_compile_features(inertial_odometry PUBLIC cxx_std_17)
target_include_directories(inertial_odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(inertial_odometry common_console common_diagnostics)
ament_target_dependencies(inertial_odometry
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS inertial_odometry ARCHIVE DESTINATION lib)
