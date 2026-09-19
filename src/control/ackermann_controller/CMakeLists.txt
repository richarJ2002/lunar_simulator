cmake_minimum_required(VERSION 3.16)

add_library(ackermann_controller STATIC
  src/methods/loadSixValues.cc
  src/methods/handleVelocityCommandCallBack.cc
  src/methods/computeSteeringAngle.cc
  src/methods/computeWheelSpeedRadps.cc
)
target_compile_features(ackermann_controller PUBLIC cxx_std_17)
target_include_directories(ackermann_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(ackermann_controller
  actuator_msgs
  geometry_msgs
  rclcpp
)
install(TARGETS ackermann_controller ARCHIVE DESTINATION lib)
