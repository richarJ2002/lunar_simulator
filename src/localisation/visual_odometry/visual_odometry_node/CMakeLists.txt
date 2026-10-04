cmake_minimum_required(VERSION 3.16)

add_library(visual_odometry STATIC
  methods/stampToSeconds.cc
  methods/multiply.cc
  methods/invertRigid.cc
  methods/configureCameraTransform.cc
  methods/calculateVisualPoseQuality.cc
  methods/handleResetCallBack.cc
  methods/handleStereoCallBack.cc
  methods/publishPipelineDiagnosticsCallBack.cc
  methods/publishFeatureImage.cc
  methods/updatePose.cc
  methods/publishPointCloud.cc
  methods/rotationToQuaternion.cc
  methods/publishOdometry.cc
  methods/storePrevious.cc
  methods/calculateSparseDisparity.cc
  methods/selectKeyframeAction.cc
)
target_compile_features(visual_odometry PUBLIC cxx_std_17)
target_include_directories(visual_odometry PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_include_directories(visual_odometry SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(visual_odometry
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(visual_odometry
  alpha_feature_tracking
  common_console
  common_diagnostics
  ${OpenCV_LIBRARIES}
)