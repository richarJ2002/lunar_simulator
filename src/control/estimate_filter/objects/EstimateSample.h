/*!
 * @file            EstimateSample.h
 *
 * @brief           Declares one pose-and-twist estimate as the low-pass
 *                  filter consumes and produces it.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_SAMPLE_H
#define LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_SAMPLE_H

/* External Library Includes */
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace control::estimate_filter
{

/*!
 * @brief           Pose in a fixed frame and twist in the body frame at one
 *                  time, following nav_msgs/Odometry conventions.
 */
struct EstimateSample
{
  public:
    /*!
     * @brief           Time the estimate is valid at.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double stamp_s{0.0};

    /*!
     * @brief           Body position.
     *
     * @frame           fixed (the odometry header frame)
     * @units           metres
     */
    Eigen::Vector3d position_fixed_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Rotation from body to fixed frame.
     *
     * @frame           body to fixed
     * @units           unit quaternion
     */
    Eigen::Quaterniond orientation_bodyToFixed{Eigen::Quaterniond::Identity()};

    /*!
     * @brief           Linear velocity.
     *
     * @frame           body (the odometry child frame)
     * @units           metres per second
     */
    Eigen::Vector3d linearVelocity_body_mPerS{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Angular velocity.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d angularVelocity_body_radPerS{Eigen::Vector3d::Zero()};
};

} /* namespace control::estimate_filter */

#endif /* LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_SAMPLE_H */
