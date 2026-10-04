cmake_minimum_required(VERSION 3.16)

add_library(ackermann_controller STATIC
  methods/loadSixValues.cc
  methods/handleVelocityCommandCallBack.cc
  methods/computeSteeringAngle.cc
  methods/computeWheelSpeed_radPs.cc
)
target_compile_features(ackermann_controller PUBLIC cxx_std_17)
target_include_directories(ackermann_controller
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ackermann_controller common_console)
ament_target_dependencies(ackermann_controller
  actuator_msgs
  geometry_msgs
  rclcpp
)
install(TARGETS ackermann_controller ARCHIVE DESTINATION lib)
