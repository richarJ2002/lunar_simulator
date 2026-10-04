/*!
 * @file            ImuSampleStruct.h
 *
 * @brief           Declares one filtered IMU sample used by Alpha's EKF.
 *
 * @date            20/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_STRUCT_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_STRUCT_H

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

/*!
 * @brief           Stores one raw IMU sample as the EKF propagates it.
 */
struct ImuSample
{
  public:
    /*!
     * @brief           ROS timestamp carried by the source IMU message.
     *
     * @frame           N/A
     * @units           seconds
     */
    double timestamp_s{0.0};

    /*!
     * @brief           Specific force applied over the interval ending at
     *                  timestamp_s.
     *
     *                  For a sample held as a contact shock this is the
     *                  previous sample's specific force, not the measured
     *                  one.
     *
     * @frame           body
     * @units           metres per second squared
     */
    Eigen::Vector3d linearAcceleration_body_mPs2{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Angular velocity reported by the IMU.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d angularVelocity_body_radPs{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Velocity change a held shock would have integrated:
     *                  the discarded specific-force change times the
     *                  sample interval.
     *
     *                  Zero for an ordinary sample. predictTo() adds its
     *                  outer product to the velocity covariance, because
     *                  the true change from a brief impulse lies somewhere
     *                  between zero and this value.
     *
     * @frame           body
     * @units           metres per second
     */
    Eigen::Vector3d discardedDeltaVelocity_body_mPs{Eigen::Vector3d::Zero()};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_SAMPLE_STRUCT_H \
        */
