cmake_minimum_required(VERSION 3.16)

# Reusable, model-agnostic continuous-discrete EKF math engine: takes
# caller-evaluated F/Q/H/R matrices for a runtime-sized state, performs only
# the linear algebra. See ContinuousExtendedKalmanFilter.h.
add_library(ekf_continuous_kalman_filter STATIC
  src/methods/initialize.cc
  src/methods/predict.cc
  src/methods/update.cc
  src/methods/terminate.cc
  src/methods/getState.cc
  src/methods/getCovariance.cc
  src/methods/setState.cc
  src/methods/applyCovarianceTransform.cc
  src/methods/restore.cc
  src/methods/isCovarianceValid.cc
)
target_compile_features(ekf_continuous_kalman_filter PUBLIC cxx_std_17)
target_include_directories(ekf_continuous_kalman_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ekf_continuous_kalman_filter PUBLIC Eigen3::Eigen)

install(TARGETS ekf_continuous_kalman_filter ARCHIVE DESTINATION lib)
