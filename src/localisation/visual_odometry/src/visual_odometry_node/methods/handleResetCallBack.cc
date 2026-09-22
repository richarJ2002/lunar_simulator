/**
 * @file            handleResetCallBack.cc
 *
 * @brief           Implements explicit visual-odometry epoch reset.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

namespace localisation::visual_odometry
{

void VisualOdometryNode::handleResetCallBack(
    const std_msgs::msg::Empty &message_in)
{
    static_cast<void>(message_in);
    previousLeft.release();
    previousRight.release();
    previousStampS            = 0.0;
    worldFromOptical          = bodyFromOptical;
    accumulatedPoseCovariance = PoseCovariance::zeros();
    isVisualPoseAvailable     = true;
    latestAcceptedInterval_s  = 0.0;
    consecutiveFailureCount   = 0U;
    RCLCPP_INFO(get_logger(), "Visual odometry epoch reset");
}

} /* namespace localisation::visual_odometry */
