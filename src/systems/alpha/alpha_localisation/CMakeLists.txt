cmake_minimum_required(VERSION 3.16)

# Dispatcher for Alpha's localisation-consuming packages, mirroring
# src/localisation/kalman_filter/'s dispatcher pattern -- currently just
# alpha_kalman_filter (which owns both the EKF fusion and the estimated
# path/TF, with room for a future sibling fusion approach without
# cluttering alpha's own top level.
add_subdirectory(alpha_kalman_filter)
