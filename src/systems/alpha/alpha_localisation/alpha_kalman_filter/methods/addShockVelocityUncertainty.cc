/*!
 * @file            addShockVelocityUncertainty.cc
 *
 * @brief           Implements velocity-covariance inflation for a held IMU
 *                  shock.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

/* External Library Includes */
#include <Eigen/Geometry>

/* Object Includes */
#include "objects/ErrorStateIndexEnum.h"
#include "objects/StateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus
    AlphaKalmanFilterNode::addShockVelocityUncertainty(
        const Eigen::Vector3d &deltaVelocityBody_mPs_in)
{
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);

    Eigen::Quaterniond quaternion_bodyToFixed(
        nominalState(quaternionIndex + 3),
        nominalState(quaternionIndex),
        nominalState(quaternionIndex + 1),
        nominalState(quaternionIndex + 2));
    quaternion_bodyToFixed.normalize();
    const Eigen::Vector3d deltaVelocity_fixed_mPs =
        quaternion_bodyToFixed * deltaVelocityBody_mPs_in;

    /* The velocity error-state is expressed in startup-fixed, like the
     * nominal velocity. */
    ErrorStateMatrix covariance = filter.getCovariance();
    covariance.block<3, 3>(errorVelocityIndex, errorVelocityIndex) +=
        deltaVelocity_fixed_mPs * deltaVelocity_fixed_mPs.transpose();
    return filter.restore(filter.getState(), covariance);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
