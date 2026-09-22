/**
 * @file            calculateQuaternionError.cc
 *
 * @brief           Implements the logarithmic quaternion innovation.
 *
 * @date            20/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <Eigen/Geometry>

/* Other Project Module Includes */
/* None */

/* Object Includes */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

Eigen::Vector3d AlphaKalmanFilterNode::calculateQuaternionError(
    const Eigen::Quaterniond &measuredQuaternion_in,
    const Eigen::Quaterniond &predictedQuaternion_in)
{
    Eigen::Quaterniond measuredQuaternion = measuredQuaternion_in.normalized();
    const Eigen::Quaterniond predictedQuaternion =
        predictedQuaternion_in.normalized();

    /*
     * Compute the right-multiplicative attitude residual
     *
     *     errorQuaternion_body =
     *         predictedQuaternion_bodyToMap^-1
     *         * measuredQuaternion_bodyToMap
     *
     * Both operands are unit quaternions. The result rotates the predicted
     * body frame onto the measured body frame and is therefore represented in
     * the body tangent frame used by the error-state attitude components.
     */
    Eigen::Quaterniond errorQuaternion =
        predictedQuaternion.conjugate() * measuredQuaternion;

    /* q and -q encode the same rotation. Select the hemisphere with positive
     * scalar coefficient so the logarithm follows the shortest rotation. */
    if (errorQuaternion.w() < 0.0)
    {
        errorQuaternion.coeffs() *= -1.0;
    }
    errorQuaternion.normalize();

    const double vectorNorm = errorQuaternion.vec().norm();
    if (vectorNorm <= 1.0e-12)
    {
        /* For a near-identity quaternion, Log(q) approaches twice the vector
         * coefficient and avoids normalizing a near-zero rotation axis. */
        return 2.0 * errorQuaternion.vec();
    }

    const double angle_rad = 2.0 * std::atan2(vectorNorm, errorQuaternion.w());
    return errorQuaternion.vec() * (angle_rad / vectorNorm);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
