/*!
 * @file            handleOdometryCallBack.cc
 *
 * @brief           Implements filtering and republishing one estimate.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/objects/EstimateLowPassFilterNodeClass.h"

namespace control::estimate_filter
{

void EstimateLowPassFilterNode::handleOdometryCallBack(
    const nav_msgs::msg::Odometry &message_in)
{
    EstimateSample sample;
    sample.stamp_s =
        rclcpp::Time(message_in.header.stamp, RCL_ROS_TIME).seconds();
    sample.position_fixed_m = Eigen::Vector3d(message_in.pose.pose.position.x,
                                              message_in.pose.pose.position.y,
                                              message_in.pose.pose.position.z);
    sample.orientation_bodyToFixed =
        Eigen::Quaterniond(message_in.pose.pose.orientation.w,
                           message_in.pose.pose.orientation.x,
                           message_in.pose.pose.orientation.y,
                           message_in.pose.pose.orientation.z);
    sample.linearVelocity_body_mPerS =
        Eigen::Vector3d(message_in.twist.twist.linear.x,
                        message_in.twist.twist.linear.y,
                        message_in.twist.twist.linear.z);
    sample.angularVelocity_body_radPs =
        Eigen::Vector3d(message_in.twist.twist.angular.x,
                        message_in.twist.twist.angular.y,
                        message_in.twist.twist.angular.z);

    /* A different frame pair is a different quantity: restart from it. */
    const bool hasFrameChanged =
        message_in.header.frame_id != previousFrameId ||
        message_in.child_frame_id != previousChildFrameId;
    previousFrameId      = message_in.header.frame_id;
    previousChildFrameId = message_in.child_frame_id;

    if (filter.update(sample, hasFrameChanged) ==
        FilterUpdateResult::FILTER_UPDATE_RESULT_SKIPPED)
    {
        return;
    }

    /* Same stamp and frames as the input; the covariance is passed through
     * and describes the unfiltered estimate. */
    const EstimateSample   &filtered = filter.getOutput();
    nav_msgs::msg::Odometry output   = message_in;
    output.pose.pose.position.x      = filtered.position_fixed_m.x();
    output.pose.pose.position.y      = filtered.position_fixed_m.y();
    output.pose.pose.position.z      = filtered.position_fixed_m.z();
    output.pose.pose.orientation.w   = filtered.orientation_bodyToFixed.w();
    output.pose.pose.orientation.x   = filtered.orientation_bodyToFixed.x();
    output.pose.pose.orientation.y   = filtered.orientation_bodyToFixed.y();
    output.pose.pose.orientation.z   = filtered.orientation_bodyToFixed.z();
    output.twist.twist.linear.x      = filtered.linearVelocity_body_mPerS.x();
    output.twist.twist.linear.y      = filtered.linearVelocity_body_mPerS.y();
    output.twist.twist.linear.z      = filtered.linearVelocity_body_mPerS.z();
    output.twist.twist.angular.x = filtered.angularVelocity_body_radPs.x();
    output.twist.twist.angular.y = filtered.angularVelocity_body_radPs.y();
    output.twist.twist.angular.z = filtered.angularVelocity_body_radPs.z();
    p_outputPublisher->publish(output);
}

} /* namespace control::estimate_filter */
