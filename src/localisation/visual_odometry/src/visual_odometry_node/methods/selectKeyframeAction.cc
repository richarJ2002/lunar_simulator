/*!
 * @file            selectKeyframeAction.cc
 *
 * @brief           Implements deterministic visual keyframe policy.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace localisation::visual_odometry
{

VisualOdometryNode::KeyframeAction
    VisualOdometryNode::selectKeyframeAction(bool   estimationSucceeded_in,
                                             bool   isPoseAvailable_in,
                                             double keyframeIntervalS_in,
                                             double maximumKeyframeIntervalS_in,
                                             double medianParallaxPx_in,
                                             double survivalRatio_in,
                                             const KeyframePolicy &policy_in)
{
    /* Without pose continuity frames advance only to keep diagnostics
     * current. */
    if (!isPoseAvailable_in)
    {
        return KeyframeAction::ADVANCE_KEYFRAME;
    }
    if (estimationSucceeded_in)
    {
        /* Keep the keyframe only while every threshold says the increment
         * would still be short; anything unmeasurable advances. */
        const bool hasEnoughParallax =
            !std::isfinite(medianParallaxPx_in) ||
            medianParallaxPx_in >= policy_in.minimumParallaxPx;
        const bool isTrackingThinning =
            !std::isfinite(survivalRatio_in) ||
            survivalRatio_in < policy_in.minimumSurvivalRatio;
        const bool isRetentionOver =
            keyframeIntervalS_in >= policy_in.maximumRetentionS;
        return hasEnoughParallax || isTrackingThinning || isRetentionOver
                   ? KeyframeAction::ADVANCE_KEYFRAME
                   : KeyframeAction::RETAIN_KEYFRAME;
    }
    if (keyframeIntervalS_in > maximumKeyframeIntervalS_in)
    {
        return KeyframeAction::MARK_POSE_UNAVAILABLE;
    }
    return KeyframeAction::RETAIN_KEYFRAME;
}

} /* namespace localisation::visual_odometry */
