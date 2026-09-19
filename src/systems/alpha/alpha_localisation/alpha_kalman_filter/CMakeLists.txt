cmake_minimum_required(VERSION 3.16)

# Alpha's own EKF model: process model, per-source observation matrices, and
# the eighteen-state pose/twist layout. Wraps the model-agnostic
# ekf_continuous_kalman_filter engine (src/localisation/kalman_filter/) and
# also exposes the fused estimate as a retained path and TF; see
# AlphaKalmanFilterNode.h.
add_library(alpha_kalman_filter STATIC
  methods/setNoiseTriplet.cc
  methods/computeProcessModel.cc
  methods/predictTo.cc
  methods/buildObservationMatrix.cc
  methods/odometryToState.cc
  methods/measurementVariances.cc
  methods/compensateMeasurementAge.cc
  methods/wrapAngle.cc
  methods/logStepFailure.cc
  methods/publishEstimate.cc
  methods/handleMeasurementCallBack.cc
  methods/handleInitialPoseCallBack.cc
  methods/isPoseValid.cc
  methods/stampToNanoseconds.cc
  methods/prepareOdometry.cc
  methods/appendPathPose.cc
  methods/publishTransform.cc
  methods/publishEstimatedPath.cc
  methods/handleSlipObservationCallBack.cc
  methods/publishWheelSlip.cc
)
target_compile_features(alpha_kalman_filter PUBLIC cxx_std_17)
target_include_directories(alpha_kalman_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(alpha_kalman_filter ekf_continuous_kalman_filter Eigen3::Eigen)
ament_target_dependencies(alpha_kalman_filter
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS alpha_kalman_filter ARCHIVE DESTINATION lib)
