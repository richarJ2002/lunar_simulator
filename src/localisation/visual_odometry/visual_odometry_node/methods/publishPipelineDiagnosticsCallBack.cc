/*!
 * @file            publishPipelineDiagnosticsCallBack.cc
 *
 * @brief           Implements periodic visual-pipeline diagnostics: a
 *                  recorded DiagnosticArray every simulated second and a
 *                  compact console health line every five.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNodeClass.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

namespace localisation::visual_odometry
{

namespace
{

/*!
 * Converts a count delta over a simulated interval into a rate, returning
 * zero for an empty or non-positive interval (the first record, or a paused
 * clock) instead of dividing by zero.
 */
double calculateRateHz(std::uint64_t countDelta_in, double interval_s_in)
{
    return interval_s_in > 0.0
               ? static_cast<double>(countDelta_in) / interval_s_in
               : 0.0;
}

} /* anonymous namespace */

void VisualOdometryNode::publishPipelineDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Rates use the simulated interval since the previous record, so they
     * are per simulated second like every other recorded topic. */
    const rclcpp::Time reportTime = now();
    const double       reportInterval_s =
        previousReportTime.nanoseconds() > 0
                  ? (reportTime - previousReportTime).seconds()
                  : 0.0;
    const double receptionRateHz =
        calculateRateHz(receivedPairCount - previousReportedPairCount,
                        reportInterval_s);
    const double acceptedRateHz =
        calculateRateHz(acceptedPoseCount - previousReportedAcceptedCount,
                        reportInterval_s);

    /* Readiness contract read by the start-up supervisor. */
    std::string readinessReason = "tracking";
    if (!isVisualPoseAvailable)
    {
        readinessReason = "pose unavailable";
    }
    else if (!hasReadinessStreak)
    {
        readinessReason = common::console::formatText(
            "%llu/%llu consecutive poses",
            static_cast<unsigned long long>(consecutiveAcceptedCount),
            static_cast<unsigned long long>(readinessConsecutivePoses));
    }
    else if (reportTime.seconds() - latestAcceptedPoseTime_s >
             readinessMaximumPoseGapS)
    {
        readinessReason = common::console::formatText(
            "no pose for %.1f s",
            reportTime.seconds() - latestAcceptedPoseTime_s);
    }
    const bool isReady = readinessReason == "tracking";

    /* One status carries every pipeline field; the keys are the contract
     * post_processing/python_tools/diagnostics/bag_diagnostics.py parses. */
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name        = "visual_odometry";
    status.hardware_id = baseFrame;
    status.level       = isReady ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message     = common::console::formatText(
        "VO %.1fHz inl %zu fail %llu",
        acceptedRateHz,
        latestInlierCount,
        static_cast<unsigned long long>(consecutiveFailureCount));
    diagnostics::addFlagValue("ready", isReady, status);
    diagnostics::addTextValue("reason", readinessReason, status);
    diagnostics::addCountValue("received", receivedPairCount, status);
    diagnostics::addCountValue("accepted", acceptedPoseCount, status);
    diagnostics::addCountValue("failed", failedPoseCount, status);
    diagnostics::addRealValue("reception_rate_hz", receptionRateHz, status);
    diagnostics::addRealValue("accepted_rate_hz", acceptedRateHz, status);
    diagnostics::addRealValue("age_s", latestAdmissionAge_s, status);
    diagnostics::addRealValue("processing_ms",
                              latestProcessingDuration_ms,
                              status);
    diagnostics::addRealValue("conversion_ms",
                              latestConversionDuration_ms,
                              status);
    diagnostics::addRealValue("disparity_ms",
                              latestDisparityDuration_ms,
                              status);
    diagnostics::addRealValue("detection_ms",
                              latestDetectionDuration_ms,
                              status);
    diagnostics::addRealValue("tracking_ms", latestTrackingDuration_ms, status);
    diagnostics::addRealValue("reconstruction_ms",
                              latestReconstructionDuration_ms,
                              status);
    diagnostics::addRealValue("pnp_ms", latestPnpDuration_ms, status);
    diagnostics::addRealValue("median_parallax_px",
                              latestMedianParallaxPx,
                              status);
    diagnostics::addCountValue("keyframe_retained",
                               keyframeRetainedCount,
                               status);
    diagnostics::addCountValue("detected", latestDetectedCount, status);
    diagnostics::addCountValue("tracked", latestTrackedCount, status);
    diagnostics::addCountValue("stereo_valid", latestStereoValidCount, status);
    diagnostics::addCountValue("correspondences",
                               latestCorrespondenceCount,
                               status);
    diagnostics::addCountValue("inliers", latestInlierCount, status);
    diagnostics::addRealValue("occupancy",
                              latestVisualQuality.occupancyRatio,
                              status);
    diagnostics::addRealValue("near_mid_ratio",
                              latestVisualQuality.nearMidRatio,
                              status);
    diagnostics::addRealValue("disparity_p10_px",
                              latestVisualQuality.disparityPercentilesPx[0],
                              status);
    diagnostics::addRealValue("disparity_p50_px",
                              latestVisualQuality.disparityPercentilesPx[1],
                              status);
    diagnostics::addRealValue("disparity_p90_px",
                              latestVisualQuality.disparityPercentilesPx[2],
                              status);
    diagnostics::addRealValue("depth_p10_m",
                              latestVisualQuality.depthPercentilesM[0],
                              status);
    diagnostics::addRealValue("depth_p50_m",
                              latestVisualQuality.depthPercentilesM[1],
                              status);
    diagnostics::addRealValue("depth_p90_m",
                              latestVisualQuality.depthPercentilesM[2],
                              status);
    diagnostics::addRealValue("reprojection_rms_px",
                              latestVisualQuality.reprojectionRmsPx,
                              status);
    diagnostics::addRealValue("normal_condition",
                              latestVisualQuality.normalCondition,
                              status);
    diagnostics::addRealValue("accepted_interval_s",
                              latestAcceptedInterval_s,
                              status);
    diagnostics::addCountValue("consecutive_failures",
                               consecutiveFailureCount,
                               status);
    diagnostics::addFlagValue("pose_available", isVisualPoseAvailable, status);

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = reportTime;
    record.status.push_back(status);
    p_diagnosticsPublisher->publish(record);

    previousReportedPairCount     = receivedPairCount;
    previousReportedAcceptedCount = acceptedPoseCount;
    previousReportTime            = reportTime;
    diagnosticsRecordCount++;

    /* The console carries only a short health line, every fifth record,
     * with rates over the whole console interval so they are steadier than
     * a one-second window at a few hertz. */
    if (diagnosticsRecordCount % CONSOLE_HEALTH_PERIOD_TICKS != 0U)
    {
        return;
    }
    const double consoleInterval_s =
        previousConsoleTime.nanoseconds() > 0
            ? (reportTime - previousConsoleTime).seconds()
            : 0.0;
    if (consoleInterval_s > 0.0)
    {
        LUNAR_LOG_INFO(
            get_logger(),
            "VO %.1fHz inl %zu fail %llu",
            calculateRateHz(acceptedPoseCount - previousConsoleAcceptedCount,
                            consoleInterval_s),
            latestInlierCount,
            static_cast<unsigned long long>(failedPoseCount -
                                            previousConsoleFailedCount));
    }
    previousConsoleAcceptedCount = acceptedPoseCount;
    previousConsoleFailedCount   = failedPoseCount;
    previousConsoleTime          = reportTime;
}

} /* namespace localisation::visual_odometry */
