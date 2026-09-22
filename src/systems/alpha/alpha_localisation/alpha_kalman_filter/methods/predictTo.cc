/**
 * @file            predictTo.cc
 *
 * @brief           Implements timestamp-ordered raw-IMU ESKF prediction.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <algorithm>
#include <cstddef>
#include <optional>

/* Object Includes */
#include "objects/ImuSample.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus
    AlphaKalmanFilterNode::predictTo(double targetTimestampS_in,
                                     bool   shouldSaveCheckpoints_in)
{
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    constexpr double MAXIMUM_SUBSTEP_S     = 0.02;

    if (!hasInitialState)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }
    if (targetTimestampS_in < stateTimestamp_s - TIMESTAMP_TOLERANCE_S)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }
    if (targetTimestampS_in <= stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
    {
        return FilterStatus::FILTER_STATUS_SUCCESS;
    }

    const auto propagateInterval =
        [this, MAXIMUM_SUBSTEP_S](
            double                 intervalEndTimestampS_in,
            const Eigen::Vector3d &specificForceBodyMps2_in,
            const Eigen::Vector3d &angularVelocityBodyRadPerS_in)
        -> FilterStatus
    {
        double remainingTime_s = intervalEndTimestampS_in - stateTimestamp_s;
        while (remainingTime_s > 0.0)
        {
            const double timeStep_s =
                std::min(remainingTime_s, MAXIMUM_SUBSTEP_S);
            NominalStateVector predictedState;
            ErrorStateMatrix   processJacobian;
            computeProcessModel(nominalState,
                                specificForceBodyMps2_in,
                                angularVelocityBodyRadPerS_in,
                                timeStep_s,
                                predictedState,
                                processJacobian);
            const FilterStatus predictionStatus =
                filter.predict(timeStep_s,
                               ErrorStateVector::Zero(),
                               processJacobian,
                               processNoise);
            if (predictionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
            {
                return predictionStatus;
            }
            nominalState = predictedState;
            stateTimestamp_s += timeStep_s;
            remainingTime_s -= timeStep_s;
        }
        return FilterStatus::FILTER_STATUS_SUCCESS;
    };

    const std::size_t retainedSampleCount = imuBuffer.getSampleCount();
    for (std::size_t sampleIndex = 0U; sampleIndex < retainedSampleCount;
         ++sampleIndex)
    {
        const std::optional<ImuSample> sample =
            imuBuffer.getSample(sampleIndex);
        if (!sample.has_value())
        {
            return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
        }
        if (sample->timestamp_s <= stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
        {
            continue;
        }
        const double intervalEndTimestamp_s =
            std::min(sample->timestamp_s, targetTimestampS_in);
        const FilterStatus propagationStatus =
            propagateInterval(intervalEndTimestamp_s,
                              sample->linearAcceleration_body_mPerS2,
                              sample->angularVelocity_body_radPerS);
        if (propagationStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return propagationStatus;
        }
        if (shouldSaveCheckpoints_in)
        {
            saveFilterCheckpoint();
        }
        if (sample->timestamp_s > targetTimestampS_in + TIMESTAMP_TOLERANCE_S)
        {
            break;
        }
    }

    if (targetTimestampS_in > stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
    {
        const std::optional<ImuSample> latestSample =
            imuBuffer.getLatestSample();
        if (!latestSample.has_value() ||
            targetTimestampS_in - latestSample->timestamp_s >
                maximumImuMeasurementAgeS)
        {
            return FilterStatus::FILTER_STATUS_INVALID_INPUT;
        }
        const FilterStatus propagationStatus =
            propagateInterval(targetTimestampS_in,
                              latestSample->linearAcceleration_body_mPerS2,
                              latestSample->angularVelocity_body_radPerS);
        if (propagationStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return propagationStatus;
        }
    }

    const std::optional<ImuSample> latestSample = imuBuffer.getLatestSample();
    if (latestSample.has_value())
    {
        const Eigen::Index gyroscopeBiasIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
        latestAngularVelocity_body_radPerS =
            latestSample->angularVelocity_body_radPerS -
            nominalState.segment<3>(gyroscopeBiasIndex);
    }
    latestState      = nominalState;
    latestCovariance = filter.getCovariance();
    hasEstimate      = true;
    if (shouldSaveCheckpoints_in)
    {
        saveFilterCheckpoint();
    }
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
