cmake_minimum_required(VERSION 3.16)

# Low-pass filter that smooths the fused estimate for controllers
# (/alpha/control/filtered_odometry); never fed back into the estimator.
# The filter core is ROS-free so it is unit-tested without a node.
# Consumers include "estimate_filter/...", so the include root is src/control.
add_library(estimate_filter STATIC
  methods/EstimateLowPassFilter/getOutput.cc
  methods/EstimateLowPassFilter/reset.cc
  methods/EstimateLowPassFilter/update.cc
  methods/EstimateLowPassFilterNode/declareConfiguration.cc
  methods/EstimateLowPassFilterNode/handleOdometryCallBack.cc
  public_functions/isConfigurationValid.cc
)
target_compile_features(estimate_filter PUBLIC cxx_std_17)
target_include_directories(estimate_filter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(estimate_filter common_console Eigen3::Eigen)
ament_target_dependencies(estimate_filter nav_msgs rclcpp)

install(TARGETS estimate_filter ARCHIVE DESTINATION lib)
