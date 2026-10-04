cmake_minimum_required(VERSION 3.16)

# Reusable, model-agnostic continuous-discrete EKF math engine: takes
# caller-evaluated F/Q/H/R matrices for a runtime-sized state, performs only
# the linear algebra. See ContinuousExtendedKalmanFilter.h.
add_library(ekf_continuous_kalman_filter STATIC
  methods/initialize.cc
  methods/predict.cc
  methods/update.cc
  methods/terminate.cc
  methods/getState.cc
  methods/getCovariance.cc
  methods/setState.cc
  methods/applyCovarianceTransform.cc
  methods/restore.cc
  methods/isCovarianceValid.cc
)
target_compile_features(ekf_continuous_kalman_filter PUBLIC cxx_std_17)
target_include_directories(ekf_continuous_kalman_filter
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ekf_continuous_kalman_filter PUBLIC Eigen3::Eigen)

install(TARGETS ekf_continuous_kalman_filter ARCHIVE DESTINATION lib)
