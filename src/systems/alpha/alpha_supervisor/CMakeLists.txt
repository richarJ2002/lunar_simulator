cmake_minimum_required(VERSION 3.16)

# Start-up window and command readiness: aggregates every component's
# self-reported readiness from the diagnostics topic into one latched
# system state, which alpha_drivers' command gate enforces. Consumers
# include "alpha_supervisor/...", so the include root is src/systems/alpha.
add_library(alpha_supervisor STATIC
  methods/handleDiagnosticsCallBack.cc
  methods/publishSystemStateCallBack.cc
  public_functions/evaluateSystemState.cc
  public_functions/parseSystemState.cc
  public_functions/systemStateName.cc
)
target_compile_features(alpha_supervisor PUBLIC cxx_std_17)
target_include_directories(alpha_supervisor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(alpha_supervisor common_console common_diagnostics)
ament_target_dependencies(alpha_supervisor diagnostic_msgs rclcpp)

install(TARGETS alpha_supervisor ARCHIVE DESTINATION lib)
