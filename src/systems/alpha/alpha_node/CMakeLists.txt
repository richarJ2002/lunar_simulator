cmake_minimum_required(VERSION 3.16)

# The single entry point for the Alpha system: constructs and spins every
# node in its stack in one process. Every library it links is built by an
# earlier add_subdirectory() in this package's own CMakeLists.txt or by
# src/localisation and src/control -- see the root CMakeLists.txt comment
# on why those two build before src/systems/alpha.
add_executable(alpha_node
  main.cpp
  methods/spin.cc
)
target_compile_features(alpha_node PUBLIC cxx_std_17)
target_include_directories(alpha_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(alpha_node rclcpp)
target_link_libraries(alpha_node
  alpha_drivers
  ground_truth
  inertial_odometry
  visual_odometry
  wheel_odometry
  alpha_kalman_filter
  ackermann_controller
)

install(TARGETS alpha_node DESTINATION lib/${PROJECT_NAME})
