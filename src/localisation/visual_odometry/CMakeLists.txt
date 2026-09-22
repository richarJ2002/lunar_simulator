cmake_minimum_required(VERSION 3.16)

# Reusable, dependency-minimal feature-detection/tracking engine. No
# third-party dependency at all (not even Eigen) -- see
# docs/compliance/feature_tracking/ for the applicable coding profile.
add_library(alpha_feature_tracking STATIC
  src/feature_tracking/methods/ShiTomasiCornerDetector/initialize.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/detect.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/computeGradients.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/computeStructureTensorAndResponse.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/collectCandidates.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/selectByMinimumDistance.cc
  src/feature_tracking/methods/ShiTomasiCornerDetector/terminate.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/initialize.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/track.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/buildPyramid.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/refineFeatureAtLevel.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/sampleBilinear.cc
  src/feature_tracking/methods/PyramidalLucasKanadeTracker/terminate.cc
)
target_compile_features(alpha_feature_tracking PUBLIC cxx_std_17)
target_include_directories(alpha_feature_tracking PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
# This target has no OpenCV/ROS header exposure, so it isn't subject to the
# -Wconversion-inside-declare_parameter reason the root CMakeLists.txt
# documents for omitting -Werror repo-wide; hold it to a stricter bar.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(alpha_feature_tracking PRIVATE -Werror)
endif()

add_library(visual_odometry STATIC
  src/visual_odometry_node/methods/stampToSeconds.cc
  src/visual_odometry_node/methods/multiply.cc
  src/visual_odometry_node/methods/invertRigid.cc
  src/visual_odometry_node/methods/configureCameraTransform.cc
  src/visual_odometry_node/methods/calculateVisualPoseQuality.cc
  src/visual_odometry_node/methods/handleResetCallBack.cc
  src/visual_odometry_node/methods/handleStereoCallBack.cc
  src/visual_odometry_node/methods/logPipelineDiagnostics.cc
  src/visual_odometry_node/methods/publishFeatureImage.cc
  src/visual_odometry_node/methods/updatePose.cc
  src/visual_odometry_node/methods/publishPointCloud.cc
  src/visual_odometry_node/methods/rotationToQuaternion.cc
  src/visual_odometry_node/methods/publishOdometry.cc
  src/visual_odometry_node/methods/storePrevious.cc
  src/visual_odometry_node/methods/selectKeyframeAction.cc
)
target_compile_features(visual_odometry PUBLIC cxx_std_17)
target_include_directories(visual_odometry PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(visual_odometry SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(visual_odometry
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(visual_odometry
  alpha_feature_tracking
  ${OpenCV_LIBRARIES}
)
install(TARGETS alpha_feature_tracking visual_odometry
  ARCHIVE DESTINATION lib
)
