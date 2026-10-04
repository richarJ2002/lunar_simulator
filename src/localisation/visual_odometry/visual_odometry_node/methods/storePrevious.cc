/*!
 * @File:         storePrevious.cc
 *
 * @Brief:        Implements retention of the current frame as "previous".
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "visual_odometry_node/objects/VisualOdometryNodeClass.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <chrono>

namespace localisation::visual_odometry
{

void VisualOdometryNode::storePrevious(
    const cv::Mat                                                  &left_in,
    const cv::Mat                                                  &right_in,
    const builtin_interfaces::msg::Time                            &stamp_in,
    const std::array<feature_tracking::Point2D,
                     feature_tracking::MAXIMUM_SUPPORTED_FEATURES> &corners_in,
    std::size_t cornerCount_in)
{
    /* Keep the frame's own detections so the next callback tracks them
     * without detecting on the same image again. */
    previousKeyframeCorners     = corners_in;
    previousKeyframeCornerCount = cornerCount_in;

    /* Sparse stereo matches the keyframe's corners once, here, rather than
     * building a dense map of the same pair on every frame. */
    if (isSparseStereo)
    {
        const std::chrono::steady_clock::time_point matchingStart =
            std::chrono::steady_clock::now();
        calculateSparseDisparity(left_in,
                                 right_in,
                                 corners_in,
                                 cornerCount_in,
                                 sparseStereoMaximumDisparityPx,
                                 sparseStereoHalfWindowPx,
                                 sparseStereoMaximumLeftRightDifferencePx,
                                 previousKeyframeDisparityPx);
        latestDisparityDuration_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - matchingStart)
                .count();
    }

    /*!
     * cv::Mat uses shared, reference-counted storage; clone() forces a deep
     * copy so a retained "previous" frame cannot be silently mutated later
     * through the caller's now-stale reference to the same underlying
     * buffer.
     */
    previousLeft = left_in.clone();

    /* Deep-copy the right frame for the same reason as the left frame. */
    previousRight = right_in.clone();

    /* Retain this frame's timestamp, in seconds, for the next callback's
     * elapsed-time calculation. */
    previousStamp_s = stampToSeconds(stamp_in);
}

} /* namespace localisation::visual_odometry */
