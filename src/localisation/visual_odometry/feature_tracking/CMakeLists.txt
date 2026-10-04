cmake_minimum_required(VERSION 3.16)

# Reusable, dependency-minimal feature-detection/tracking engine. No
# third-party dependency at all (not even Eigen) -- see
# docs/compliance/feature_tracking/ for the applicable coding profile.
add_library(alpha_feature_tracking STATIC
  methods/ShiTomasiCornerDetector/initialize.cc
  methods/ShiTomasiCornerDetector/detect.cc
  methods/ShiTomasiCornerDetector/computeGradients.cc
  methods/ShiTomasiCornerDetector/computeStructureTensorAndResponse.cc
  methods/ShiTomasiCornerDetector/collectCandidates.cc
  methods/ShiTomasiCornerDetector/selectByMinimumDistance.cc
  methods/ShiTomasiCornerDetector/terminate.cc
  methods/PyramidalLucasKanadeTracker/initialize.cc
  methods/PyramidalLucasKanadeTracker/track.cc
  methods/PyramidalLucasKanadeTracker/buildPyramid.cc
  methods/PyramidalLucasKanadeTracker/refineFeatureAtLevel.cc
  methods/PyramidalLucasKanadeTracker/sampleBilinear.cc
  methods/PyramidalLucasKanadeTracker/terminate.cc
)

target_compile_features(alpha_feature_tracking PUBLIC cxx_std_17)
target_include_directories(alpha_feature_tracking PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
# This target has no OpenCV/ROS header exposure, so it isn't subject to the
# -Wconversion-inside-declare_parameter reason the root CMakeLists.txt
# documents for omitting -Werror repo-wide; hold it to a stricter bar.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(alpha_feature_tracking PRIVATE -Werror)
endif()