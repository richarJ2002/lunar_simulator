/*!
 * @File:         handleOdometryCallBack.cc
 *
 * @Brief:        Publishes one truth transform and sampled path point.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/GroundTruthNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::ground_truth
{

void GroundTruthNode::handleOdometryCallBack(
    const nav_msgs::msg::Odometry &odometry_in)
{
    /* Reject a corrupt or not-yet-settled pose before using it. */
    if (!isPoseValid(odometry_in.pose.pose))
    {
        /*!
         * Drop the sample rather than publish a corrupt TF/path point; the
         * throttle bounds log spam if Gazebo keeps producing bad poses.
         */
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "Ignoring an invalid ground-truth pose");

        /* Nothing further can be done with this sample; stop here. */
        return;
    }

    /* Start from a copy of the validated odometry message. */
    nav_msgs::msg::Odometry truthOdometry = odometry_in;

    /*!
     * The pose is already in the shared Gazebo map frame; only the frame id
     * needs to be written explicitly.
     */
    truthOdometry.header.frame_id = mapFrame;

    /*!
     * Publish under ground truth's own comparison child frame, not the
     * estimator's "alpha/base_link".
     */
    truthOdometry.child_frame_id = groundTruthBaseFrame;

    /* Broadcast the map-to-body transform for this sample. */
    publishTransform(truthOdometry);

    /* Add this sample to the retained, rate-limited path. */
    appendPathPose(truthOdometry);

    if (!hasPublishedInitialPose)
    {
        /*!
         * This is the first valid sample this node has seen, which (per
         * alpha_node/main.cpp's fixed startup delay) is already the
         * rover's settled resting pose, not a mid-drop transient --
         * publish it once, latched, as the map-frame origin
         * continuous_ekf/wheel_odometry seed themselves from.
         */
        p_initialPosePublisher->publish(truthOdometry);

        hasPublishedInitialPose = true;
    }
}

} /* namespace localisation::ground_truth */
