/*!
 * @File:         handleKalmanFilterCallBack.cc
 *
 * @Brief:        Implements refreshing the live roll/pitch cache from
 *                continuous_ekf's fused estimate.
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

void WheelOdometryNode::handleKalmanFilterCallBack(
    const nav_msgs::msg::Odometry &message_in)
{
    /* Convert the message's ROS quaternion into a tf2 quaternion. */
    tf2::Quaternion orientation;
    tf2::fromMsg(message_in.pose.pose.orientation, orientation);

    /* Guard against floating-point drift in the reported quaternion. */
    orientation.normalize();

    if (!std::isfinite(orientation.x()) || !std::isfinite(orientation.y()) ||
        !std::isfinite(orientation.z()) || !std::isfinite(orientation.w()))
    {
        /*!
         * A non-finite quaternion (e.g. from a malformed upstream message)
         * must never corrupt the live roll/pitch cache; keep whatever was
         * cached before.
         */
        return;
    }

    /* Destination for the roll/pitch/yaw decomposition below. */
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double yawRad = 0.0;

    /* Decompose the fused orientation into roll, pitch and yaw; only
     * roll/pitch are retained below, matching this cache's purpose. */
    tf2::Matrix3x3(orientation).getRPY(rollRad, pitchRad, yawRad);

    /* Refresh the live roll/pitch cache used by handleJointStateCallBack()
     * to project each integration step through the rover's actual current
     * tilt. */
    latestRollRad = rollRad;
    latestPitchRad = pitchRad;
}

} /* namespace localisation::wheel_odometry */
