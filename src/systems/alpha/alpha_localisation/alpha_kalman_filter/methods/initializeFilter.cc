/**
 * @file            initializeFilter.cc
 *
 * @brief           Initializes Alpha's bias-aware ESKF from stationary IMU.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus
    AlphaKalmanFilterNode::initializeFilter(double timestampS_in)
{
    if (imuInitializationSampleCount < imuInitializationSampleTarget)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }

    const double initializationSampleCount =
        static_cast<double>(imuInitializationSampleCount);
    const Eigen::Vector3d meanSpecificForce_body_mPerS2 =
        initializationSpecificForceSum_body_mPerS2 / initializationSampleCount;
    const Eigen::Vector3d meanAngularVelocity_body_radPerS =
        initializationAngularVelocitySum_body_radPerS /
        initializationSampleCount;
    const double measuredSpecificForceMagnitude_mPerS2 =
        meanSpecificForce_body_mPerS2.norm();
    if (measuredSpecificForceMagnitude_mPerS2 <= 1.0e-12)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /* At rest, body-to-fixed is identity and specific force is the negative
     * of physical gravity. The known lunar magnitude separates a bounded
     * accelerometer-bias prior from the measured direction. */
    gravityAcceleration_fixed_mPerS2 = -gravityMagnitudeMps2 *
                                       meanSpecificForce_body_mPerS2 /
                                       measuredSpecificForceMagnitude_mPerS2;
    const Eigen::Vector3d initialAccelerometerBias_body_mPerS2 =
        meanSpecificForce_body_mPerS2 + gravityAcceleration_fixed_mPerS2;

    nominalState = NominalStateVector::Zero();
    const Eigen::Index quaternionWIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_W);
    const Eigen::Index accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    nominalState(quaternionWIndex) = 1.0;
    nominalState.segment<3>(accelerometerBiasIndex) =
        initialAccelerometerBias_body_mPerS2;
    nominalState.segment<3>(gyroscopeBiasIndex) =
        meanAngularVelocity_body_radPerS;

    ErrorStateMatrix   initialCovariance  = ErrorStateMatrix::Identity();
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAccelerometerBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);
    initialCovariance.block<3, 3>(errorPositionIndex, errorPositionIndex) *=
        1.0e-9;
    initialCovariance.block<3, 3>(errorAttitudeIndex, errorAttitudeIndex) *=
        1.0e-9;
    initialCovariance.block<3, 3>(errorVelocityIndex, errorVelocityIndex) *=
        0.01;
    initialCovariance.block<3, 3>(errorAccelerometerBiasIndex,
                                  errorAccelerometerBiasIndex) *=
        initialAccelerometerBiasVariance;
    initialCovariance.block<3, 3>(errorGyroscopeBiasIndex,
                                  errorGyroscopeBiasIndex) *=
        initialGyroscopeBiasVariance;

    const FilterStatus status = filter.initialize(ERROR_STATE_SIZE,
                                                  ErrorStateVector::Zero(),
                                                  initialCovariance);
    if (status != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        return status;
    }

    stateTimestamp_s                   = timestampS_in;
    latestState                        = nominalState;
    latestCovariance                   = filter.getCovariance();
    latestAngularVelocity_body_radPerS = Eigen::Vector3d::Zero();
    hasInitialState                    = true;
    hasEstimate                        = true;
    clearFilterCheckpoints();
    saveFilterCheckpoint();
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
