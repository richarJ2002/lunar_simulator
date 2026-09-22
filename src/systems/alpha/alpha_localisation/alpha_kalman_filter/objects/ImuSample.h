/**
 * @file            ImuSample.h
 *
 * @brief           Declares one filtered IMU sample used by Alpha's EKF.
 *
 * @date            20/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_H

/* C++ Standard Library Includes */
/* None */

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <Eigen/Dense>

/* Other Project Module Includes */
/* None */

/* Object Includes */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/**
 * @brief           Stores one bias-corrected, gravity-free IMU sample.
 */
struct ImuSample
{
  public:
    /**
     * @brief           ROS timestamp carried by the source IMU message.
     *
     * @frame           N/A
     * @units           seconds
     */
    double timestamp_s{0.0};

    /**
     * @brief           Gravity-free linear acceleration reported by the IMU.
     *
     * @frame           body
     * @units           metres per second squared
     */
    Eigen::Vector3d linearAcceleration_body_mPerS2{Eigen::Vector3d::Zero()};

    /**
     * @brief           Angular velocity reported by the IMU.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d angularVelocity_body_radPerS{Eigen::Vector3d::Zero()};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_H \
        */
