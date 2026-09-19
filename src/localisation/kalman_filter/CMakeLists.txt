cmake_minimum_required(VERSION 3.16)

add_subdirectory(ekf_continuous_kalman_filter)
# future algorithm variants (e.g. ekf_discrete_kalman_filter,
# ukf_discrete_kalman_filter, ukf_continuous_kalman_filter) are added as
# additional self-contained sibling packages, each with their own
# add_subdirectory() line here.
