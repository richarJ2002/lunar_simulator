cmake_minimum_required(VERSION 3.16)

# Interfaces Alpha's stack with Gazebo in both directions: raw sensor reads
# and raw actuator writes both pass through this one package, with
# configurable Gaussian noise/bias applied on each.
add_library(alpha_drivers STATIC
  methods/addVarianceToCovariance.cc
  methods/configureImuInterface.cc
  methods/configureJointStateInterface.cc
  methods/configureWheelCommandInterface.cc
  methods/declareNonnegativeParameter.cc
  methods/declareTripletParameter.cc
  methods/perturbOrientation.cc
  methods/publishNoisyImuCallBack.cc
  methods/publishNoisyJointStateCallBack.cc
  methods/publishNoisyWheelCommandCallBack.cc
  methods/sampleGaussian.cc
)
target_compile_features(alpha_drivers PUBLIC cxx_std_17)
target_include_directories(alpha_drivers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(alpha_drivers
  actuator_msgs
  rclcpp
  sensor_msgs
  tf2
)

install(TARGETS alpha_drivers ARCHIVE DESTINATION lib)
