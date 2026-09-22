/**
 * @file            handleImuCallBack.cc
 *
 * @brief           Implements stationary initialization and raw-IMU prediction.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <optional>

/* Object Includes */
#include "objects/ImuSample.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::handleImuCallBack(
    const sensor_msgs::msg::Imu &message_in)
{
    const double admissionTimestamp_s = now().seconds();
    ++imuDiagnostics.receivedCount;
    imuDiagnostics.callbackAdmissionTimestamp_s = admissionTimestamp_s;
    imuDiagnostics.processingStartTimestamp_s   = admissionTimestamp_s;
    const auto finishDiagnostics                = [this]()
    {
        imuDiagnostics.processingEndTimestamp_s = now().seconds();
        imuDiagnostics.estimatorTimestamp_s     = stateTimestamp_s;
    };

    ImuSample sample;
    sample.timestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    sample.linearAcceleration_body_mPerS2 =
        Eigen::Vector3d(message_in.linear_acceleration.x,
                        message_in.linear_acceleration.y,
                        message_in.linear_acceleration.z);
    sample.angularVelocity_body_radPerS =
        Eigen::Vector3d(message_in.angular_velocity.x,
                        message_in.angular_velocity.y,
                        message_in.angular_velocity.z);
    imuDiagnostics.publicationTimestamp_s = sample.timestamp_s;

    if (!std::isfinite(sample.timestamp_s) ||
        !sample.linearAcceleration_body_mPerS2.allFinite() ||
        !sample.angularVelocity_body_radPerS.allFinite())
    {
        ++imuDiagnostics.numericalRejectedCount;
        finishDiagnostics();
        return;
    }

    const std::optional<ImuSample> latestSample = imuBuffer.getLatestSample();
    if (latestSample.has_value() &&
        sample.timestamp_s < latestSample->timestamp_s)
    {
        imuBuffer.clear();
        clearFilterCheckpoints();
        if (hasInitialState)
        {
            const FilterStatus terminateStatus = filter.terminate();
            if (terminateStatus != FilterStatus::FILTER_STATUS_SUCCESS)
            {
                logStepFailure(terminateStatus);
            }
        }
        hasInitialState              = false;
        hasEstimate                  = false;
        imuInitializationSampleCount = 0U;
        initializationSpecificForceSum_body_mPerS2.setZero();
        initializationAngularVelocitySum_body_radPerS.setZero();
        ++imuDiagnostics.ageRejectedCount;
    }

    if (!imuBuffer.push(sample))
    {
        ++imuDiagnostics.numericalRejectedCount;
        finishDiagnostics();
        return;
    }
    ++imuDiagnostics.acceptedCount;

    if (!hasInitialState)
    {
        initializationSpecificForceSum_body_mPerS2 +=
            sample.linearAcceleration_body_mPerS2;
        initializationAngularVelocitySum_body_radPerS +=
            sample.angularVelocity_body_radPerS;
        ++imuInitializationSampleCount;
        if (imuInitializationSampleCount < imuInitializationSampleTarget)
        {
            finishDiagnostics();
            return;
        }

        const FilterStatus initializeStatus =
            initializeFilter(sample.timestamp_s);
        if (initializeStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            ++imuDiagnostics.numericalRejectedCount;
            logStepFailure(initializeStatus);
            finishDiagnostics();
            return;
        }
        ++imuDiagnostics.fusedCount;
        RCLCPP_INFO(get_logger(),
                    "Raw IMU stationary initialization complete (%zu samples)",
                    imuInitializationSampleCount);
        finishDiagnostics();
        return;
    }

    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    if (sample.timestamp_s <= stateTimestamp_s + TIMESTAMP_TOLERANCE_S)
    {
        const Eigen::Index gyroscopeBiasIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
        latestAngularVelocity_body_radPerS =
            sample.angularVelocity_body_radPerS -
            nominalState.segment<3>(gyroscopeBiasIndex);
        finishDiagnostics();
        return;
    }

    const FilterStatus predictionStatus = predictTo(sample.timestamp_s);
    if (predictionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        ++imuDiagnostics.numericalRejectedCount;
        logStepFailure(predictionStatus);
    }
    else
    {
        const Eigen::Index gyroscopeBiasIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
        latestAngularVelocity_body_radPerS =
            sample.angularVelocity_body_radPerS -
            nominalState.segment<3>(gyroscopeBiasIndex);
        ++imuDiagnostics.fusedCount;
        saveFilterCheckpoint();
    }
    finishDiagnostics();
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
