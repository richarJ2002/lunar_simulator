/**
 * @file            calculateWheelVelocityObservation.cc
 *
 * @brief           Calculates Alpha's wheel body-velocity observation model.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

/* Object Includes */
#include "objects/ErrorStateIndexEnum.h"
#include "objects/StateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::calculateWheelVelocityObservation(
    const NominalStateVector       &state_in,
    Eigen::Vector3d                &predictedVelocityBody_mPs_out,
    WheelVelocityObservationMatrix &observationMatrix_out)
{
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);

    Eigen::Quaterniond quaternion_bodyToFixed(state_in(quaternionIndex + 3),
                                              state_in(quaternionIndex),
                                              state_in(quaternionIndex + 1),
                                              state_in(quaternionIndex + 2));
    quaternion_bodyToFixed.normalize();
    const Eigen::Matrix3d rotation_fixedToBody =
        quaternion_bodyToFixed.toRotationMatrix().transpose();
    const Eigen::Vector3d velocity_body_mPs =
        rotation_fixedToBody * state_in.segment<3>(velocityIndex);
    predictedVelocityBody_mPs_out = velocity_body_mPs;

    Eigen::Matrix3d velocityCrossMatrix = Eigen::Matrix3d::Zero();
    velocityCrossMatrix << 0.0, -velocity_body_mPs.z(),
        velocity_body_mPs.y(), velocity_body_mPs.z(), 0.0,
        -velocity_body_mPs.x(), -velocity_body_mPs.y(),
        velocity_body_mPs.x(), 0.0;
    observationMatrix_out.setZero();
    observationMatrix_out.block<3, 3>(0, errorVelocityIndex) =
        rotation_fixedToBody;
    observationMatrix_out.block<3, 3>(0, errorAttitudeIndex) =
        velocityCrossMatrix;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
