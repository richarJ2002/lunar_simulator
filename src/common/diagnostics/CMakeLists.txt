cmake_minimum_required(VERSION 3.16)

# Builds the key/value entries every project node publishes on the recorded
# <system>/diagnostics topic (diagnostic_msgs/DiagnosticArray), so values
# are formatted identically everywhere post-processing parses them.
add_library(common_diagnostics STATIC
  public_functions/addCountValue.cc
  public_functions/addFlagValue.cc
  public_functions/addRealValue.cc
  public_functions/addTextValue.cc
)
target_compile_features(common_diagnostics PUBLIC cxx_std_17)
target_include_directories(common_diagnostics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
ament_target_dependencies(common_diagnostics diagnostic_msgs)

install(TARGETS common_diagnostics ARCHIVE DESTINATION lib)
