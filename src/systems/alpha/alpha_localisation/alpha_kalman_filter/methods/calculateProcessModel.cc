/**
 * @file            calculateProcessModel.cc
 *
 * @brief           Implements Alpha's bias-aware ESKF process model.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>

/* External Library Includes */
#include <Eigen/Dense>
#include <Eigen/Geometry>

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::calculateProcessModel(
    const NominalStateVector &state_in,
    const Eigen::Vector3d    &specificForceBodyMps2_in,
    const Eigen::Vector3d    &angularVelocityBodyRadPerS_in,
    const Eigen::Vector3d    &gravityAccelerationFixedMps2_in,
    double                    timeStepS_in,
    NominalStateVector       &predictedState_out,
    ErrorStateMatrix         &processJacobian_out)
{
    const Eigen::Index positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);

    const Eigen::Vector3d position_fixed_m = state_in.segment<3>(positionIndex);
    const Eigen::Vector3d velocity_fixed_mPerS =
        state_in.segment<3>(velocityIndex);
    const Eigen::Vector3d accelerometerBias_body_mPerS2 =
        state_in.segment<3>(accelerometerBiasIndex);
    const Eigen::Vector3d gyroscopeBias_body_radPerS =
        state_in.segment<3>(gyroscopeBiasIndex);
    Eigen::Quaterniond quaternion_bodyToFixed(state_in(quaternionIndex + 3),
                                              state_in(quaternionIndex),
                                              state_in(quaternionIndex + 1),
                                              state_in(quaternionIndex + 2));
    quaternion_bodyToFixed.normalize();

    const Eigen::Vector3d correctedSpecificForce_body_mPerS2 =
        specificForceBodyMps2_in - accelerometerBias_body_mPerS2;
    const Eigen::Vector3d correctedAngularVelocity_body_radPerS =
        angularVelocityBodyRadPerS_in - gyroscopeBias_body_radPerS;
    const Eigen::Matrix3d rotation_bodyToFixed =
        quaternion_bodyToFixed.toRotationMatrix();
    const Eigen::Vector3d acceleration_fixed_mPerS2 =
        rotation_bodyToFixed * correctedSpecificForce_body_mPerS2 +
        gravityAccelerationFixedMps2_in;

    const Eigen::Vector3d rotationVector_body_rad =
        correctedAngularVelocity_body_radPerS * timeStepS_in;
    const double       rotationMagnitude_rad = rotationVector_body_rad.norm();
    Eigen::Quaterniond rotationIncrement_bodyToBody =
        Eigen::Quaterniond::Identity();
    if (rotationMagnitude_rad > 1.0e-12)
    {
        rotationIncrement_bodyToBody = Eigen::Quaterniond(
            Eigen::AngleAxisd(rotationMagnitude_rad,
                              rotationVector_body_rad / rotationMagnitude_rad));
    }

    Eigen::Quaterniond predictedQuaternion_bodyToFixed =
        quaternion_bodyToFixed * rotationIncrement_bodyToBody;
    predictedQuaternion_bodyToFixed.normalize();

    predictedState_out = state_in;
    predictedState_out.segment<3>(positionIndex) =
        position_fixed_m + velocity_fixed_mPerS * timeStepS_in +
        0.5 * acceleration_fixed_mPerS2 * timeStepS_in * timeStepS_in;
    predictedState_out.segment<3>(velocityIndex) =
        velocity_fixed_mPerS + acceleration_fixed_mPerS2 * timeStepS_in;
    predictedState_out(quaternionIndex) = predictedQuaternion_bodyToFixed.x();
    predictedState_out(quaternionIndex + 1) =
        predictedQuaternion_bodyToFixed.y();
    predictedState_out(quaternionIndex + 2) =
        predictedQuaternion_bodyToFixed.z();
    predictedState_out(quaternionIndex + 3) =
        predictedQuaternion_bodyToFixed.w();

    Eigen::Matrix3d specificForceCrossMatrix = Eigen::Matrix3d::Zero();
    specificForceCrossMatrix << 0.0, -correctedSpecificForce_body_mPerS2.z(),
        correctedSpecificForce_body_mPerS2.y(),
        correctedSpecificForce_body_mPerS2.z(), 0.0,
        -correctedSpecificForce_body_mPerS2.x(),
        -correctedSpecificForce_body_mPerS2.y(),
        correctedSpecificForce_body_mPerS2.x(), 0.0;
    Eigen::Matrix3d angularVelocityCrossMatrix = Eigen::Matrix3d::Zero();
    angularVelocityCrossMatrix << 0.0,
        -correctedAngularVelocity_body_radPerS.z(),
        correctedAngularVelocity_body_radPerS.y(),
        correctedAngularVelocity_body_radPerS.z(), 0.0,
        -correctedAngularVelocity_body_radPerS.x(),
        -correctedAngularVelocity_body_radPerS.y(),
        correctedAngularVelocity_body_radPerS.x(), 0.0;

    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorAccelerometerBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    processJacobian_out = ErrorStateMatrix::Zero();
    processJacobian_out.block<3, 3>(errorPositionIndex, errorVelocityIndex) =
        Eigen::Matrix3d::Identity();
    processJacobian_out.block<3, 3>(errorVelocityIndex, errorAttitudeIndex) =
        -rotation_bodyToFixed * specificForceCrossMatrix;
    processJacobian_out.block<3, 3>(errorVelocityIndex,
                                    errorAccelerometerBiasIndex) =
        -rotation_bodyToFixed;
    processJacobian_out.block<3, 3>(errorAttitudeIndex, errorAttitudeIndex) =
        -angularVelocityCrossMatrix;
    processJacobian_out.block<3, 3>(errorAttitudeIndex,
                                    errorGyroscopeBiasIndex) =
        -Eigen::Matrix3d::Identity();
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
