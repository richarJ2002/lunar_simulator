cmake_minimum_required(VERSION 3.16)

add_subdirectory(feature_tracking)
add_subdirectory(visual_odometry_node)

install(TARGETS alpha_feature_tracking visual_odometry
  ARCHIVE DESTINATION lib
)
