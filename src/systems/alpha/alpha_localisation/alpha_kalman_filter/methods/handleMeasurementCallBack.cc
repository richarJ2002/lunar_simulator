/*!
 * @file            handleMeasurementCallBack.cc
 *
 * @brief           Implements visual-pose and wheel-velocity corrections.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <optional>

/* External Library Includes */
#include <Eigen/Dense>

/* Object Includes */
#include "objects/FusedMeasurementStruct.h"
#include "objects/ImuSampleStruct.h"
#include "objects/MeasurementFusionResultEnum.h"
#include "objects/MeasurementKindEnum.h"
#include "objects/StateIndexEnum.h"

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
    diagnostics.receivedCount++;
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
        diagnostics.ageRejectedCount++;
        diagnostics.preInitRejectedCount++;
        finishDiagnostics();
        return;
    }

    const double measurementTimestamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    const double callbackAge_s = now().seconds() - measurementTimestamp_s;
    if (!std::isfinite(measurementTimestamp_s) || !std::isfinite(callbackAge_s))
    {
        /* A non-finite stamp is malformed input, not a timing decision. */
        diagnostics.numericalRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* Age rejections are split by reason so a dominant cause (a stamp
     * slightly ahead of a lagging /clock versus a genuinely stale message)
     * can be told apart from the periodic diagnostics alone. A stamp at
     * most maximumFutureStamp_s ahead of this node's clock is fused: the
     * state is predicted forward to it like any newer measurement. */
    if (callbackAge_s < -maximumFutureStamp_s)
    {
        diagnostics.ageRejectedCount++;
        diagnostics.negativeAgeRejectedCount++;
        finishDiagnostics();
        return;
    }
    if (callbackAge_s > maximumVisualMeasurementAgeS)
    {
        diagnostics.ageRejectedCount++;
        diagnostics.tooOldRejectedCount++;
        finishDiagnostics();
        return;
    }

    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    const double     presentTimestamp_s    = stateTimestamp_s;
    const double     stateAgeFromMeasurement_s =
        stateTimestamp_s - measurementTimestamp_s;
    if (std::abs(stateAgeFromMeasurement_s) > maximumVisualMeasurementAgeS)
    {
        diagnostics.ageRejectedCount++;
        diagnostics.stateGapRejectedCount++;
        finishDiagnostics();
        return;
    }

    /* An increment measures motion since the previous visual stamp, so the
     * interval advances on every admitted visual message, fused or not. */
    double intervalStart_s = -1.0;
    if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL &&
        isVisualIncrementMode)
    {
        intervalStart_s       = previousVisualStamp_s;
        previousVisualStamp_s = measurementTimestamp_s;
        if (!(intervalStart_s >= 0.0) ||
            !(measurementTimestamp_s > intervalStart_s))
        {
            finishDiagnostics();
            return;
        }
    }
    FusedMeasurement record;
    if (!buildMeasurementRecord(message_in,
                                kind_in,
                                measurementTimestamp_s,
                                intervalStart_s,
                                record))
    {
        diagnostics.numericalRejectedCount++;
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
            diagnostics.ageRejectedCount++;
            diagnostics.rollbackFailedCount++;
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
            diagnostics.ageRejectedCount++;
            diagnostics.predictFailedCount++;
            finishDiagnostics();
            return;
        }
    }

    diagnostics.acceptedCount++;
    const MeasurementFusionResult fusionResult =
        fuseMeasurementRecord(record,
                              diagnostics.normalizedInnovationSquared,
                              diagnostics.correctionNorm);
    if (fusionResult !=
        MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED)
    {
        if (didRollback)
        {
            static_cast<void>(filter.restore(presentCheckpoint.errorState,
                                             presentCheckpoint.covariance));
            nominalState     = presentCheckpoint.nominalState;
            stateTimestamp_s = presentCheckpoint.timestamp_s;
            latestState      = nominalState;
            latestCovariance = filter.getCovariance();
        }
        if (fusionResult ==
            MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_NIS_REJECTED)
        {
            diagnostics.nisRejectedCount++;
        }
        else
        {
            diagnostics.numericalRejectedCount++;
        }
        finishDiagnostics();
        return;
    }
    /* A record that cannot be stored (older than a full history) is
     * already older than any rollback target, so it never needs replay. */
    static_cast<void>(measurementHistory.insert(record));

    if (didRollback)
    {
        /* Every checkpoint after this epoch predates the new measurement.
         * The replay rebuilds them and re-applies the measurements already
         * fused in between; propagating with the IMU alone would silently
         * drop those updates from the estimate. */
        discardFilterCheckpointsAfter(measurementTimestamp_s);
        saveFilterCheckpoint();
        const FilterStatus replayStatus =
            replayMeasurementsAfter(measurementTimestamp_s, presentTimestamp_s);
        if (replayStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            static_cast<void>(filter.restore(presentCheckpoint.errorState,
                                             presentCheckpoint.covariance));
            nominalState     = presentCheckpoint.nominalState;
            stateTimestamp_s = presentCheckpoint.timestamp_s;
            latestState      = nominalState;
            latestCovariance = filter.getCovariance();
            clearFilterCheckpoints();
            measurementHistory.clear();
            saveFilterCheckpoint();
            diagnostics.numericalRejectedCount++;
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
            latestAngularVelocity_body_radPs =
                latestSample->angularVelocity_body_radPs -
                nominalState.segment<3>(gyroscopeBiasIndex);
        }
    }

    diagnostics.fusedCount++;
    saveFilterCheckpoint();
    finishDiagnostics();
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
