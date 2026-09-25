/*!
 * @file            calculateVisualIncrementObservation.cc
 *
 * @brief           Implements the increment-mode visual measurement model.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::calculateVisualIncrementObservation(
    const NominalStateVector &state_in,
    double                    meanRawYawRateRadPerS_in,
    bool                      hasYawRate_in,
    Eigen::VectorXd          &predictedMeasurement_out,
    Eigen::MatrixXd          &observationMatrix_out)
{
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    Eigen::Quaterniond quaternion_bodyToFixed(state_in(quaternionIndex + 3),
                                              state_in(quaternionIndex),
                                              state_in(quaternionIndex + 1),
                                              state_in(quaternionIndex + 2));
    quaternion_bodyToFixed.normalize();
    const Eigen::Matrix3d rotation_fixedToBody =
        quaternion_bodyToFixed.toRotationMatrix().transpose();
    const Eigen::Vector3d velocity_body_mPerS =
        rotation_fixedToBody * state_in.segment<3>(velocityIndex);

    /* Right perturbation R = R_hat Exp(dtheta) gives
     * R^T v = R_hat^T v + [R_hat^T v]x dtheta to first order. */
    Eigen::Matrix3d velocityCrossMatrix;
    velocityCrossMatrix << 0.0, -velocity_body_mPerS.z(),
        velocity_body_mPerS.y(), velocity_body_mPerS.z(), 0.0,
        -velocity_body_mPerS.x(), -velocity_body_mPerS.y(),
        velocity_body_mPerS.x(), 0.0;

    const Eigen::Index rowCount = hasYawRate_in ? 4 : 3;
    predictedMeasurement_out    = Eigen::VectorXd::Zero(rowCount);
    observationMatrix_out = Eigen::MatrixXd::Zero(rowCount, ERROR_STATE_SIZE);
    predictedMeasurement_out.head<3>() = velocity_body_mPerS;
    observationMatrix_out.block<3, 3>(0, errorVelocityIndex) =
        rotation_fixedToBody;
    observationMatrix_out.block<3, 3>(0, errorAttitudeIndex) =
        velocityCrossMatrix;
    if (hasYawRate_in)
    {
        /* Bias-corrected body yaw rate averaged over the visual interval. */
        predictedMeasurement_out(3) =
            meanRawYawRateRadPerS_in - state_in(gyroscopeBiasIndex + 2);
        observationMatrix_out(3, errorGyroscopeBiasIndex + 2) = -1.0;
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
