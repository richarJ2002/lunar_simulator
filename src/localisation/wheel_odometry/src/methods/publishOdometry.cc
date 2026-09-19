/*!
 * @File:         publishOdometry.cc
 *
 * @Brief:        Implements publication of the integrated wheel-odometry
 *                message.
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::publishOdometry(
    const builtin_interfaces::msg::Time &stamp_in,
    const Eigen::Vector3d &bodyTwist_in)
{
    /* Build one odometry message to publish below. */
    nav_msgs::msg::Odometry output;

    /* Stamp the message with the triggering joint-state time. */
    output.header.stamp = stamp_in;

    /* Set the parent frame the integrated pose is expressed in. */
    output.header.frame_id = odomFrame;

    /* Set the child frame the pose and twist apply to. */
    output.child_frame_id = baseFrame;

    /* Write the integrated x position. */
    output.pose.pose.position.x = positionXM;

    /* Write the integrated y position. */
    output.pose.pose.position.y = positionYM;

    /* Write the integrated z position, projected through the live
     * roll/pitch cache at every step (see handleJointStateCallBack()). */
    output.pose.pose.position.z = positionZM;

    /*!
     * Orientation reports the full attitude used for this cycle's
     * integration (latestRollRad/latestPitchRad borrowed from
     * continuous_ekf, yawRad this node's own), not a claim that this node
     * independently measured roll/pitch itself -- this is what lets
     * odometryToState.cc recover the same rotation for its map-frame
     * velocity conversion (see its wheel-specific rebase) that this node
     * already applied internally to position above.
     */
    tf2::Quaternion orientation;
    orientation.setRPY(latestRollRad, latestPitchRad, yawRad);
    output.pose.pose.orientation = tf2::toMsg(orientation);

    /* Write the current body-frame x velocity. */
    output.twist.twist.linear.x = bodyTwist_in.x();

    /* Write the current body-frame y velocity. */
    output.twist.twist.linear.y = bodyTwist_in.y();

    /* Write the current yaw rate. */
    output.twist.twist.angular.z = bodyTwist_in.z();

    /*!
     * Fixed diagonal covariance: modest uncertainty on the observable
     * planar states (x, y, yaw and their rates); a large uncertainty on
     * roll/pitch, which this node never observes, only borrows from
     * continuous_ekf for projection; and a moderate (not fabricated)
     * uncertainty on z/z-rate, now that both are genuine estimates
     * derived from the same rolling-constraint solve, projected through
     * an independently-sourced tilt -- looser than x/y since that
     * projection compounds the borrowed roll/pitch's own uncertainty on
     * top of the encoder-derived planar estimate. First-pass values, not
     * yet empirically tuned.
     */
    output.pose.covariance[0] = 0.03;

    /* Modest y-position variance, matching x. */
    output.pose.covariance[7] = 0.03;

    /* Moderate z-position variance; see the block comment above. */
    output.pose.covariance[14] = 0.05;

    /* Large, effectively "unknown", roll variance: never observed, only
     * borrowed for projection. */
    output.pose.covariance[21] = 1.0e3;

    /* Large, effectively "unknown", pitch variance; see roll above. */
    output.pose.covariance[28] = 1.0e3;

    /* Modest yaw variance; yaw is directly observable. */
    output.pose.covariance[35] = 0.04;

    /* Modest x-rate variance. */
    output.twist.covariance[0] = 0.01;

    /* Modest y-rate variance. */
    output.twist.covariance[7] = 0.01;

    /* Moderate z-rate variance; see the pose-covariance block comment
     * above -- the body-frame twist has no z component by construction
     * (no vertical degree of freedom in the rolling-constraint solve), but
     * odometryToState.cc's map-frame rotation of this twist still produces
     * a genuine, borrowed-tilt-dependent vertical component. */
    output.twist.covariance[14] = 0.02;

    /* Large, effectively "unknown", roll-rate variance. */
    output.twist.covariance[21] = 1.0e3;

    /* Large, effectively "unknown", pitch-rate variance. */
    output.twist.covariance[28] = 1.0e3;

    /* Modest yaw-rate variance; yaw rate is directly observable. */
    output.twist.covariance[35] = 0.02;

    /* Publish the fully populated message. */
    odometryPublisher->publish(output);
}

} /* namespace localisation::wheel_odometry */
