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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

    /* Convert the raw body-to-map pose into a rigid transform. */
    tf2::Transform mapFromTruthBody;
    tf2::fromMsg(odometry_in.pose.pose, mapFromTruthBody);

    if (!hasStartupFixedFrame)
    {
        /* The full first pose, including roll and pitch, defines the local
         * fixed frame. Local estimators do not receive this transform. */
        mapFromStartupFixed = mapFromTruthBody;
        hasStartupFixedFrame = true;
        publishStartupFixedTransform(odometry_in.header.stamp);
    }

    /* T_fixed_truth = inverse(T_map_fixed) * T_map_truth. */
    const tf2::Transform startupFixedFromTruthBody =
        calculateFixedFromBody(mapFromStartupFixed, mapFromTruthBody);

    /* Start from a copy so body-frame twist and covariance are preserved. */
    nav_msgs::msg::Odometry truthOdometry = odometry_in;

    /* Comparison pose is now relative to the startup-fixed frame. */
    truthOdometry.header.frame_id = startupFixedFrame;
    truthOdometry.pose.pose.position.x =
        startupFixedFromTruthBody.getOrigin().x();
    truthOdometry.pose.pose.position.y =
        startupFixedFromTruthBody.getOrigin().y();
    truthOdometry.pose.pose.position.z =
        startupFixedFromTruthBody.getOrigin().z();
    truthOdometry.pose.pose.orientation =
        tf2::toMsg(startupFixedFromTruthBody.getRotation());

    /*!
     * Publish under ground truth's own comparison child frame, not the
     * estimator's "alpha/base_link".
     */
    truthOdometry.child_frame_id = groundTruthBaseFrame;

    /* Publish the validated sample outside the private driver boundary. */
    p_odometryPublisher->publish(truthOdometry);

    /* Broadcast the startup-fixed-to-body transform for this sample. */
    publishTransform(truthOdometry);

    /* Add this sample to the retained, rate-limited path. */
    appendPathPose(truthOdometry);

}

} /* namespace localisation::ground_truth */
