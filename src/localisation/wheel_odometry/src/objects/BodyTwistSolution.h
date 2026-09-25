/*!
 * @file            BodyTwistSolution.h
 *
 * @brief           Declares the result of one six-wheel kinematic solve.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_BODY_TWIST_SOLUTION_H
#define LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_BODY_TWIST_SOLUTION_H

/* External Library Includes */
#include <Eigen/Dense>

namespace localisation::wheel_odometry
{

/*!
 * @brief           Planar body twist estimated from the six wheels, with
 *                  its covariance.
 */
struct BodyTwistSolution
{
  public:
    /*!
     * @brief           Whether the solve produced a finite twist and a
     *                  positive-definite covariance.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isValid{false};

    /*!
     * @brief           Body twist (vx, vy, wz).
     *
     * @frame           body
     * @units           metres per second, metres per second, radians per
     *                  second
     */
    Eigen::Vector3d twist_body{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Covariance of twist_body.
     *
     * @frame           body
     * @units           squared twist units
     */
    Eigen::Matrix3d covariance_body{Eigen::Matrix3d::Identity()};

    /*!
     * @brief           Condition number of the weighted normal matrix, a
     *                  health indicator of the wheel geometry.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double normalCondition{0.0};
};

} /* namespace localisation::wheel_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_BODY_TWIST_SOLUTION_H */
