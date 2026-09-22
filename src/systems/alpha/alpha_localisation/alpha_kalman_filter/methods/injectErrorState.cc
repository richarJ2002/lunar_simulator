/**
 * @file            injectErrorState.cc
 *
 * @brief           Implements multiplicative error injection into nominal
 * state.
 *
 * @date            20/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
/* None */

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <Eigen/Geometry>

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus AlphaKalmanFilterNode::injectErrorState()
{
    const ErrorStateVector errorState = filter.getState();
    if (!errorState.allFinite())
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    const Eigen::Index nominalPositionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index nominalQuaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index nominalLinearVelocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index nominalAccelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index nominalGyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorLinearVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAccelerometerBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    /* Euclidean nominal components receive their posterior errors directly in
     * the same frames and units documented by the two index enumerations. */
    nominalState.segment<3>(nominalPositionIndex) +=
        errorState.segment<3>(errorPositionIndex);
    nominalState.segment<3>(nominalLinearVelocityIndex) +=
        errorState.segment<3>(errorLinearVelocityIndex);
    nominalState.segment<3>(nominalAccelerometerBiasIndex) +=
        errorState.segment<3>(errorAccelerometerBiasIndex);
    nominalState.segment<3>(nominalGyroscopeBiasIndex) +=
        errorState.segment<3>(errorGyroscopeBiasIndex);

    Eigen::Quaterniond quaternion_bodyToMap(
        nominalState(nominalQuaternionIndex + 3),
        nominalState(nominalQuaternionIndex),
        nominalState(nominalQuaternionIndex + 1),
        nominalState(nominalQuaternionIndex + 2));
    quaternion_bodyToMap.normalize();

    /*
     * Inject the right-multiplicative body-frame attitude error
     *
     *     quaternionCorrected_bodyToMap =
     *         quaternionNominal_bodyToMap * Exp(deltaAttitude_body)
     *
     * deltaAttitude_body is a 3-vector in radians. Right composition matches
     * calculateQuaternionError() and computeProcessModel().
     */
    const Eigen::Vector3d attitudeError_body_rad =
        errorState.segment<3>(errorAttitudeIndex);

    ErrorStateMatrix resetJacobian = ErrorStateMatrix::Identity();
    resetJacobian.block<3, 3>(errorAttitudeIndex, errorAttitudeIndex) =
        calculateAttitudeResetJacobian(attitudeError_body_rad);
    const FilterStatus covarianceResetStatus =
        filter.applyCovarianceTransform(resetJacobian);
    if (covarianceResetStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        return covarianceResetStatus;
    }

    const double attitudeErrorMagnitude_rad = attitudeError_body_rad.norm();
    Eigen::Quaterniond attitudeCorrection_bodyToBody =
        Eigen::Quaterniond::Identity();
    if (attitudeErrorMagnitude_rad > 1.0e-12)
    {
        attitudeCorrection_bodyToBody = Eigen::Quaterniond(Eigen::AngleAxisd(
            attitudeErrorMagnitude_rad,
            attitudeError_body_rad / attitudeErrorMagnitude_rad));
    }

    quaternion_bodyToMap = quaternion_bodyToMap * attitudeCorrection_bodyToBody;
    quaternion_bodyToMap.normalize();
    nominalState(nominalQuaternionIndex)     = quaternion_bodyToMap.x();
    nominalState(nominalQuaternionIndex + 1) = quaternion_bodyToMap.y();
    nominalState(nominalQuaternionIndex + 2) = quaternion_bodyToMap.z();
    nominalState(nominalQuaternionIndex + 3) = quaternion_bodyToMap.w();

    if (!nominalState.allFinite())
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    /* Error-state injection transfers the posterior mean into the nominal
     * state after covariance has moved to the corrected tangent coordinates. */
    const FilterStatus resetStatus = filter.setState(ErrorStateVector::Zero());
    if (resetStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        return resetStatus;
    }

    latestState      = nominalState;
    latestCovariance = filter.getCovariance();
    hasEstimate      = true;
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
