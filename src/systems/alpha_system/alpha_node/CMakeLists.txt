cmake_minimum_required(VERSION 3.16)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(alpha_node src/alpha_node.cpp)
add_executable(alpha_sensor_node src/alpha_sensor_node.cpp)

target_compile_features(alpha_node PUBLIC cxx_std_17)
target_compile_features(alpha_sensor_node PUBLIC cxx_std_17)

ament_target_dependencies(alpha_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(alpha_sensor_node rclcpp sensor_msgs tf2)

install(TARGETS alpha_node alpha_sensor_node DESTINATION lib/${PROJECT_NAME})
