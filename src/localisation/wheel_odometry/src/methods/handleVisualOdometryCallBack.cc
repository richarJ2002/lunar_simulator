/*!
 * @File:         handleVisualOdometryCallBack.cc
 *
 * @Brief:        Implements acceptance of a visual-odometry slip reference.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::handleVisualOdometryCallBack(
    const nav_msgs::msg::Odometry &message_in)
{
    /*!
     * covariance[0]=var(vx), [7]=var(vy), [35]=var(wz) follow the
     * standard 6x6 row-major ROS twist-covariance layout; the largest of
     * the three gives a coarse trust bound on the whole planar twist.
     */
    const double maximumTwistVariance = std::max(
        {message_in.twist.covariance[0], message_in.twist.covariance[7],
         message_in.twist.covariance[35]});

    /* Visual odometry is only ever used here as a slip-estimation
     * reference, so only the planar body twist (vx, vy, wz) is
     * extracted. */
    const Eigen::Vector3d visualTwistBody(message_in.twist.twist.linear.x,
                                          message_in.twist.twist.linear.y,
                                          message_in.twist.twist.angular.z);

    /*!
     * Reject non-finite twists (e.g. a visual-odometry dropout that
     * still publishes a degenerate message) and twists whose reported
     * uncertainty exceeds the configured trust threshold.
     */
    if (!visualTwistBody.allFinite() ||
        !std::isfinite(maximumTwistVariance) ||
        maximumTwistVariance > maximumVisualTwistVariance)
    {
        /* Leave the previously cached reference untouched rather than
         * corrupting it with an untrustworthy sample. */
        return;
    }

    /* Cache the accepted twist as the new slip-estimation reference. */
    latestVisualTwistBody = visualTwistBody;

    /* Record when this reference was produced, for the staleness and
     * timeout checks elsewhere in this node. */
    latestVisualStampS = stampToSeconds(message_in.header.stamp);

    /* Mark that at least one reference has now been accepted. */
    hasVisualReference = true;
}

} /* namespace localisation::wheel_odometry */
