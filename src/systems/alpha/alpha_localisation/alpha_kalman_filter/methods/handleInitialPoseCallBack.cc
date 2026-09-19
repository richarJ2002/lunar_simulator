/*!
 * @File:         handleInitialPoseCallBack.cc
 *
 * @Brief:        Implements seeding the filter's map-frame origin from
 *                ground_truth's one-time settled resting pose.
 *
 * @Date:         18/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <cmath>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::handleInitialPoseCallBack(
    const nav_msgs::msg::Odometry &message_in)
{
    /* Convert the message's ROS quaternion into a tf2 quaternion. */
    tf2::Quaternion orientation;
    tf2::fromMsg(message_in.pose.pose.orientation, orientation);

    /* Guard against floating-point drift in the reported quaternion. */
    orientation.normalize();

    const tf2::Vector3 position(message_in.pose.pose.position.x,
                                message_in.pose.pose.position.y,
                                message_in.pose.pose.position.z);

    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z()) || !std::isfinite(orientation.x()) ||
        !std::isfinite(orientation.y()) || !std::isfinite(orientation.z()) ||
        !std::isfinite(orientation.w()))
    {
        /*!
         * A non-finite ground-truth sample must never seed the filter's
         * origin; wait for ground_truth to publish a valid one instead.
         * ground_truth only ever latches its first VALID sample on this
         * topic (see GroundTruthNode::handleOdometryCallBack()), so this
         * is defensive rather than an expected path.
         */
        return;
    }

    /* Seed the rover's settled resting pose, used by odometryToState() to
     * rebase every incoming relative odometry measurement. */
    initialPositionMapM = position;
    initialOrientation = orientation;

    /* handleMeasurementCallBack() drops every measurement until this is
     * true. */
    hasReceivedInitialPose = true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
