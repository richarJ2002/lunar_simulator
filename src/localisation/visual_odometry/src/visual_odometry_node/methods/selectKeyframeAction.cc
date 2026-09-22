/**
 * @file            selectKeyframeAction.cc
 *
 * @brief           Implements deterministic visual keyframe policy.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

namespace localisation::visual_odometry
{

VisualOdometryNode::KeyframeAction VisualOdometryNode::selectKeyframeAction(
    bool   estimationSucceeded_in,
    bool   isPoseAvailable_in,
    double keyframeIntervalS_in,
    double maximumKeyframeIntervalS_in)
{
    if (estimationSucceeded_in || !isPoseAvailable_in)
    {
        return KeyframeAction::ADVANCE_KEYFRAME;
    }
    if (keyframeIntervalS_in > maximumKeyframeIntervalS_in)
    {
        return KeyframeAction::MARK_POSE_UNAVAILABLE;
    }
    return KeyframeAction::RETAIN_KEYFRAME;
}

} /* namespace localisation::visual_odometry */
