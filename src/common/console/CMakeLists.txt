cmake_minimum_required(VERSION 3.16)

# Wrapped console logging: every project node logs through the SRS_LOG_*
# macros in console.h, which format, wrap at 40 characters and emit one
# record per line. Consumers include "console/console.h", so the include
# root is this module's parent directory.
add_library(common_console STATIC
  methods/shouldEmit.cc
  private_functions/wrapParagraph.cc
  public_functions/formatText.cc
  public_functions/isSeverityEnabled.cc
  public_functions/logWrapped.cc
  public_functions/wrapText.cc
)
target_compile_features(common_console PUBLIC cxx_std_17)
target_include_directories(common_console PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
# rclcpp exports rcutils, whose logging API isSeverityEnabled.cc uses.
ament_target_dependencies(common_console rclcpp)

install(TARGETS common_console ARCHIVE DESTINATION lib)
