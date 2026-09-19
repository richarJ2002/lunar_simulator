cmake_minimum_required(VERSION 3.16)

# alpha_node (the single composed entry point) links every subpackage below,
# and CMake requires each target to already exist when a later
# add_subdirectory() links against it.
add_subdirectory(alpha_localisation)
add_subdirectory(alpha_drivers)
add_subdirectory(alpha_node)
