/*!
 * @File:         handleInitialPoseCallBack.cc
 *
 * @Brief:        Implements seeding this node's integrated pose and live
 *                roll/pitch cache from ground_truth's one-time settled
 *                resting pose.
 *
 * @Date:         18/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Generic Libraries */
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::handleInitialPoseCallBack(
    const nav_msgs::msg::Odometry &message_in)
{
    /* Convert the message's ROS quaternion into a tf2 quaternion. */
    tf2::Quaternion orientation;
    tf2::fromMsg(message_in.pose.pose.orientation, orientation);

    /* Guard against floating-point drift in the reported quaternion. */
    orientation.normalize();

    const double groundTruthPositionXM = message_in.pose.pose.position.x;
    const double groundTruthPositionYM = message_in.pose.pose.position.y;
    const double groundTruthPositionZM = message_in.pose.pose.position.z;

    if (!std::isfinite(groundTruthPositionXM) ||
        !std::isfinite(groundTruthPositionYM) ||
        !std::isfinite(groundTruthPositionZM) ||
        !std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
        !std::isfinite(orientation.z()) || !std::isfinite(orientation.w()))
    {
        /*!
         * A non-finite ground-truth sample must never seed this node's
         * pose; wait for ground_truth to publish a valid one instead.
         * ground_truth only ever latches its first VALID sample on this
         * topic (see GroundTruthNode::handleOdometryCallBack()), so this
         * is defensive rather than an expected path.
         */
        return;
    }

    /* Destination for the roll/pitch/yaw decomposition below. */
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double groundTruthYawRad = 0.0;

    /* Decompose the settled resting attitude into roll, pitch and yaw. */
    tf2::Matrix3x3(orientation).getRPY(rollRad, pitchRad, groundTruthYawRad);

    /* Seed this node's own integrated pose directly in absolute map-frame
     * terms (see odometryToState.cc's wheel-specific rebase). */
    positionXM = groundTruthPositionXM;
    positionYM = groundTruthPositionYM;
    positionZM = groundTruthPositionZM;
    yawRad = groundTruthYawRad;

    /* Seed the live roll/pitch cache used to project each integration
     * step (see handleJointStateCallBack()) until continuous_ekf's first
     * fused estimate arrives. */
    latestRollRad = rollRad;
    latestPitchRad = pitchRad;

    /* handleJointStateCallBack() withholds integration/publication until
     * this is true. */
    hasReceivedInitialPose = true;
}

} /* namespace localisation::wheel_odometry */
