/*!
 * @File:         odometryToState.cc
 *
 * @Brief:        Implements conversion of one relative odometry message into
 *                an absolute EKF state vector.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::StateVector AlphaKalmanFilterNode::odometryToState(
    const nav_msgs::msg::Odometry &message_in, MeasurementKind kind_in) const
{
    /* Every state defaults to zero; only entries this function sets below
     * carry a meaningful value. */
    StateVector measurement = StateVector::Zero();

    /* wheel_odometry is the one source that reports pose/orientation
     * directly in absolute map-frame terms already (see this method's own
     * doc comment); every other source reports relative to the configured
     * initial pose and must be rebased into the map frame below. */
    const bool isWheel = (kind_in == MeasurementKind::MEASUREMENT_KIND_WHEEL);

    const tf2::Vector3 relativePositionM(message_in.pose.pose.position.x,
                                         message_in.pose.pose.position.y,
                                         message_in.pose.pose.position.z);

    /*!
     * Rebase the reported position by rotating it into the initial
     * orientation and adding the configured initial map-frame position --
     * skipped for wheel, whose position already includes both.
     */
    const tf2::Vector3 positionMapM =
        isWheel ? relativePositionM
                : initialPositionMapM +
                      tf2::Matrix3x3(initialOrientation) * relativePositionM;

    /* Write the rebased x position into the state vector. */
    measurement(0) = positionMapM.x();

    /* Write the rebased y position into the state vector. */
    measurement(1) = positionMapM.y();

    /* Write the rebased z position into the state vector. */
    measurement(2) = positionMapM.z();

    /* Convert the message's ROS quaternion into a tf2 quaternion. */
    tf2::Quaternion relativeOrientation;
    tf2::fromMsg(message_in.pose.pose.orientation, relativeOrientation);

    /* Guard against floating-point drift in the reported quaternion. */
    relativeOrientation.normalize();

    /*!
     * Compose the relative orientation with the initial orientation to get
     * absolute attitude in the map frame, then decompose to roll/pitch/yaw
     * because that is the EKF's orientation state representation -- skipped
     * for wheel, whose orientation already carries the live roll/pitch it
     * borrowed from this node's own previous fused output (see this
     * method's own doc comment), so composing with initialOrientation
     * again would double-apply that tilt.
     */
    const tf2::Quaternion orientationMap =
        isWheel ? relativeOrientation : initialOrientation * relativeOrientation;

    /* Destination for the roll/pitch/yaw decomposition below. */
    double rollRad = 0.0;
    double pitchRad = 0.0;
    double yawRad = 0.0;

    /* Decompose the absolute orientation into roll, pitch and yaw. */
    tf2::Matrix3x3(orientationMap).getRPY(rollRad, pitchRad, yawRad);

    /* Write the absolute roll into the state vector. */
    measurement(3) = rollRad;

    /* Write the absolute pitch into the state vector. */
    measurement(4) = pitchRad;

    /* Write the absolute yaw into the state vector. */
    measurement(5) = yawRad;

    /*!
     * ROS convention expresses twist.linear in the child (body) frame; the
     * EKF's linear-velocity state is expressed in the map frame, so rotate
     * it into map using the absolute orientation just computed. Angular
     * rate is left in the body frame, matching the EKF's own state
     * definition, so no rotation is applied there.
     */
    const tf2::Vector3 velocityBody(message_in.twist.twist.linear.x,
                                    message_in.twist.twist.linear.y,
                                    message_in.twist.twist.linear.z);

    /* Rotate the body-frame velocity into the map frame. */
    const tf2::Vector3 velocityOdom =
        tf2::Matrix3x3(orientationMap) * velocityBody;

    /* Write the map-frame x velocity into the state vector. */
    measurement(6) = velocityOdom.x();

    /* Write the map-frame y velocity into the state vector. */
    measurement(7) = velocityOdom.y();

    /* Write the map-frame z velocity into the state vector. */
    measurement(8) = velocityOdom.z();

    /* Copy the body-frame roll rate through unrotated. */
    measurement(9) = message_in.twist.twist.angular.x;

    /* Copy the body-frame pitch rate through unrotated. */
    measurement(10) = message_in.twist.twist.angular.y;

    /* Copy the body-frame yaw rate through unrotated. */
    measurement(11) = message_in.twist.twist.angular.z;

    /* Hand the fully populated state vector back to the caller. */
    return measurement;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
