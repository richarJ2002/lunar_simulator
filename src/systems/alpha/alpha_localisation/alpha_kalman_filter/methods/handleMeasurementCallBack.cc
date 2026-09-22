/**
 * @file            handleMeasurementCallBack.cc
 *
 * @brief           Implements visual-pose and wheel-velocity corrections.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <optional>

/* External Library Includes */
#include <Eigen/Dense>
#include <Eigen/Geometry>

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/ImuSample.h"
#include "objects/MeasurementKind.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::handleMeasurementCallBack(
    const nav_msgs::msg::Odometry &message_in,
    MeasurementKind                kind_in)
{
    SourceDiagnostics &diagnostics =
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL ? visualDiagnostics
                                                            : wheelDiagnostics;
    const double admissionTimestamp_s = now().seconds();
    ++diagnostics.receivedCount;
    diagnostics.callbackAdmissionTimestamp_s = admissionTimestamp_s;
    diagnostics.processingStartTimestamp_s   = admissionTimestamp_s;
    diagnostics.publicationTimestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    const auto finishDiagnostics = [this, &diagnostics]()
    {
        diagnostics.processingEndTimestamp_s = now().seconds();
        diagnostics.estimatorTimestamp_s     = stateTimestamp_s;
    };

    const bool shouldFuseSource =
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL
            ? shouldFuseVisualPose
            : shouldFuseWheelTwist;
    if (!shouldFuseSource)
    {
        finishDiagnostics();
        return;
    }
    if (!hasInitialState)
    {
        ++diagnostics.ageRejectedCount;
        finishDiagnostics();
        return;
    }

    const double measurementTimestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    const double callbackAge_s = now().seconds() - measurementTimestamp_s;
    if (!std::isfinite(measurementTimestamp_s) ||
        !std::isfinite(callbackAge_s) || callbackAge_s < 0.0 ||
        callbackAge_s > maximumVisualMeasurementAgeS)
    {
        ++diagnostics.ageRejectedCount;
        finishDiagnostics();
        return;
    }

    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    const double     presentTimestamp_s    = stateTimestamp_s;
    const double     stateAgeFromMeasurement_s =
        stateTimestamp_s - measurementTimestamp_s;
    if (std::abs(stateAgeFromMeasurement_s) > maximumVisualMeasurementAgeS)
    {
        ++diagnostics.ageRejectedCount;
        finishDiagnostics();
        return;
    }

    bool             didRollback = false;
    FilterCheckpoint presentCheckpoint;
    if (stateAgeFromMeasurement_s > TIMESTAMP_TOLERANCE_S)
    {
        presentCheckpoint.timestamp_s  = stateTimestamp_s;
        presentCheckpoint.nominalState = nominalState;
        presentCheckpoint.errorState   = filter.getState();
        presentCheckpoint.covariance   = filter.getCovariance();
        saveFilterCheckpoint();
        if (!restoreFilterCheckpointAtOrBefore(measurementTimestamp_s) ||
            predictTo(measurementTimestamp_s, false) !=
                FilterStatus::FILTER_STATUS_SUCCESS)
        {
            static_cast<void>(filter.restore(presentCheckpoint.errorState,
                                             presentCheckpoint.covariance));
            nominalState     = presentCheckpoint.nominalState;
            stateTimestamp_s = presentCheckpoint.timestamp_s;
            latestState      = nominalState;
            latestCovariance = filter.getCovariance();
            ++diagnostics.ageRejectedCount;
            finishDiagnostics();
            return;
        }
        didRollback = true;
    }
    else if (stateAgeFromMeasurement_s < -TIMESTAMP_TOLERANCE_S)
    {
        const FilterStatus predictionStatus = predictTo(measurementTimestamp_s);
        if (predictionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            ++diagnostics.ageRejectedCount;
            finishDiagnostics();
            return;
        }
    }

    const auto restorePresentAfterRejectedRollback =
        [this, didRollback, &presentCheckpoint]()
    {
        if (!didRollback)
        {
            return;
        }
        static_cast<void>(filter.restore(presentCheckpoint.errorState,
                                         presentCheckpoint.covariance));
        nominalState     = presentCheckpoint.nominalState;
        stateTimestamp_s = presentCheckpoint.timestamp_s;
        latestState      = nominalState;
        latestCovariance = filter.getCovariance();
    };

    const Eigen::Index positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const MeasurementVarianceVector variances = measurementVariances(
        message_in,
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL ? visualVariance
                                                            : wheelVariance);

    Eigen::VectorXd innovation;
    Eigen::MatrixXd observation;
    Eigen::MatrixXd measurementNoise;
    if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        const NominalStateVector measurement = odometryToState(message_in);
        if (!measurement.allFinite())
        {
            restorePresentAfterRejectedRollback();
            ++diagnostics.numericalRejectedCount;
            finishDiagnostics();
            return;
        }
        const Eigen::Quaterniond measuredQuaternion_bodyToFixed(
            measurement(quaternionIndex + 3),
            measurement(quaternionIndex),
            measurement(quaternionIndex + 1),
            measurement(quaternionIndex + 2));
        const Eigen::Quaterniond predictedQuaternion_bodyToFixed(
            nominalState(quaternionIndex + 3),
            nominalState(quaternionIndex),
            nominalState(quaternionIndex + 1),
            nominalState(quaternionIndex + 2));
        const Eigen::Vector3d attitudeInnovation_body_rad =
            calculateQuaternionError(measuredQuaternion_bodyToFixed,
                                     predictedQuaternion_bodyToFixed);

        innovation               = Eigen::VectorXd::Zero(6);
        innovation.segment<3>(0) = measurement.segment<3>(positionIndex) -
                                   nominalState.segment<3>(positionIndex);
        innovation.segment<3>(3) = attitudeInnovation_body_rad;
        observation              = Eigen::MatrixXd::Zero(6, ERROR_STATE_SIZE);
        observation.block<3, 3>(0, errorPositionIndex) =
            Eigen::Matrix3d::Identity();
        observation.block<3, 3>(3, errorAttitudeIndex) =
            Eigen::Matrix3d::Identity();
        measurementNoise = Eigen::MatrixXd::Zero(6, 6);
        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            measurementNoise(axis, axis) = variances(axis);
            measurementNoise(axis + 3, axis + 3) =
                axis < 2 ? visualAttitudeVariance : variances(axis + 3);
        }
    }
    else
    {
        const Eigen::Vector2d measuredVelocity_body_mPerS(
            message_in.twist.twist.linear.x,
            message_in.twist.twist.linear.y);
        if (!measuredVelocity_body_mPerS.allFinite())
        {
            restorePresentAfterRejectedRollback();
            ++diagnostics.numericalRejectedCount;
            finishDiagnostics();
            return;
        }

        Eigen::Vector2d                predictedVelocity_body_mPerS;
        WheelVelocityObservationMatrix wheelObservation;
        calculateWheelVelocityObservation(nominalState,
                                          predictedVelocity_body_mPerS,
                                          wheelObservation);
        innovation = measuredVelocity_body_mPerS - predictedVelocity_body_mPerS;
        observation            = wheelObservation;
        measurementNoise       = Eigen::MatrixXd::Zero(2, 2);
        measurementNoise(0, 0) = variances(6);
        measurementNoise(1, 1) = variances(7);
    }

    ++diagnostics.acceptedCount;
    if (!calculateNormalizedInnovationSquared(
            innovation,
            observation,
            filter.getCovariance(),
            measurementNoise,
            diagnostics.normalizedInnovationSquared))
    {
        restorePresentAfterRejectedRollback();
        ++diagnostics.numericalRejectedCount;
        finishDiagnostics();
        return;
    }
    const double configuredNisThreshold =
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL ? visualNisThreshold
                                                            : wheelNisThreshold;
    const double nisThreshold =
        selectNisThreshold(configuredNisThreshold, innovation.size());
    if (diagnostics.normalizedInnovationSquared > nisThreshold)
    {
        restorePresentAfterRejectedRollback();
        ++diagnostics.nisRejectedCount;
        finishDiagnostics();
        return;
    }

    const FilterStatus updateStatus =
        filter.update(innovation, observation, measurementNoise);
    if (updateStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        restorePresentAfterRejectedRollback();
        ++diagnostics.numericalRejectedCount;
        logStepFailure(updateStatus);
        finishDiagnostics();
        return;
    }
    diagnostics.correctionNorm         = filter.getState().norm();
    const FilterStatus injectionStatus = injectErrorState();
    if (injectionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        restorePresentAfterRejectedRollback();
        ++diagnostics.numericalRejectedCount;
        logStepFailure(injectionStatus);
        finishDiagnostics();
        return;
    }

    if (didRollback)
    {
        discardFilterCheckpointsAfter(measurementTimestamp_s);
        saveFilterCheckpoint();
        const FilterStatus replayStatus = predictTo(presentTimestamp_s);
        if (replayStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            static_cast<void>(filter.restore(presentCheckpoint.errorState,
                                             presentCheckpoint.covariance));
            nominalState     = presentCheckpoint.nominalState;
            stateTimestamp_s = presentCheckpoint.timestamp_s;
            latestState      = nominalState;
            latestCovariance = filter.getCovariance();
            clearFilterCheckpoints();
            saveFilterCheckpoint();
            ++diagnostics.numericalRejectedCount;
            logStepFailure(replayStatus);
            finishDiagnostics();
            return;
        }
    }
    else
    {
        const std::optional<ImuSample> latestSample =
            imuBuffer.getLatestSample();
        if (latestSample.has_value())
        {
            const Eigen::Index gyroscopeBiasIndex = static_cast<Eigen::Index>(
                StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
            latestAngularVelocity_body_radPerS =
                latestSample->angularVelocity_body_radPerS -
                nominalState.segment<3>(gyroscopeBiasIndex);
        }
    }

    ++diagnostics.fusedCount;
    saveFilterCheckpoint();
    finishDiagnostics();
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
