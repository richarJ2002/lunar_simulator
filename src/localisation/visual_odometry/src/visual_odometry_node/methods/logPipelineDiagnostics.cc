/**
 * @file            logPipelineDiagnostics.cc
 *
 * @brief           Implements periodic visual-pipeline diagnostics.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

namespace localisation::visual_odometry
{

void VisualOdometryNode::logPipelineDiagnostics()
{
    const std::chrono::steady_clock::time_point reportTime =
        std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(reportTime - diagnosticsStartTime)
            .count();
    const std::uint64_t receivedDelta =
        receivedPairCount - previousReportedPairCount;
    const std::uint64_t acceptedDelta =
        acceptedPoseCount - previousReportedAcceptedCount;
    const double reportingPeriod_s = std::max(elapsed_s, 1.0e-9);

    RCLCPP_INFO(
        get_logger(),
        "visual_diag received=%llu accepted=%llu failed=%llu "
        "reception_rate_hz=%.3f accepted_rate_hz=%.3f age_s=%.6f "
        "processing_ms=%.3f conversion_ms=%.3f disparity_ms=%.3f "
        "detection_ms=%.3f tracking_ms=%.3f "
        "reconstruction_ms=%.3f pnp_ms=%.3f detected=%zu tracked=%zu "
        "stereo_valid=%zu correspondences=%zu inliers=%zu occupancy=%.3f "
        "near_mid_ratio=%.3f disparity_p10_p50_p90=[%.3f,%.3f,%.3f] "
        "depth_p10_p50_p90_m=[%.3f,%.3f,%.3f] reprojection_rms_px=%.3f "
        "normal_condition=%.3e accepted_interval_s=%.6f "
        "consecutive_failures=%llu",
        static_cast<unsigned long long>(receivedPairCount),
        static_cast<unsigned long long>(acceptedPoseCount),
        static_cast<unsigned long long>(failedPoseCount),
        static_cast<double>(receivedDelta) / reportingPeriod_s,
        static_cast<double>(acceptedDelta) / reportingPeriod_s,
        latestAdmissionAge_s,
        latestProcessingDuration_ms,
        latestConversionDuration_ms,
        latestDisparityDuration_ms,
        latestDetectionDuration_ms,
        latestTrackingDuration_ms,
        latestReconstructionDuration_ms,
        latestPnpDuration_ms,
        latestDetectedCount,
        latestTrackedCount,
        latestStereoValidCount,
        latestCorrespondenceCount,
        latestInlierCount,
        latestVisualQuality.occupancyRatio,
        latestVisualQuality.nearMidRatio,
        latestVisualQuality.disparityPercentilesPx[0],
        latestVisualQuality.disparityPercentilesPx[1],
        latestVisualQuality.disparityPercentilesPx[2],
        latestVisualQuality.depthPercentilesM[0],
        latestVisualQuality.depthPercentilesM[1],
        latestVisualQuality.depthPercentilesM[2],
        latestVisualQuality.reprojectionRmsPx,
        latestVisualQuality.normalCondition,
        latestAcceptedInterval_s,
        static_cast<unsigned long long>(consecutiveFailureCount));

    previousReportedPairCount     = receivedPairCount;
    previousReportedAcceptedCount = acceptedPoseCount;
    diagnosticsStartTime          = reportTime;
}

} /* namespace localisation::visual_odometry */
