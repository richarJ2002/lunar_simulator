/*!
 * @File:         publishOdometry.cc
 *
 * @Brief:        Publishes the current integrated odometry estimate.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/InertialOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <tf2/LinearMath/Matrix3x3.h>

namespace localisation::inertial_odometry
{

void InertialOdometryNode::publishOdometry(
    const builtin_interfaces::msg::Time &stamp)
{
    /* Construct the output message to populate below. */
    nav_msgs::msg::Odometry output;

    /* Stamp the message with the caller-supplied sample time. */
    output.header.stamp = stamp;

    /* Odometry is expressed in the fixed odom frame. */
    output.header.frame_id = odomFrame;

    /* The moving frame this odometry describes is the rover body. */
    output.child_frame_id = baseFrame;

    /* Publish the integrated x position. */
    output.pose.pose.position.x = positionM[0];

    /* Publish the integrated y position. */
    output.pose.pose.position.y = positionM[1];

    /* Publish the integrated z position. */
    output.pose.pose.position.z = positionM[2];

    /* Publish the integrated orientation. */
    output.pose.pose.orientation = toMessage(orientation);

    /*!
     * nav_msgs::msg::Odometry twist is defined in the child (body) frame by
     * ROS convention, but velocityMps is integrated in the odom frame, so it
     * is rotated into the body frame with the orientation's transpose
     * before publishing.
     */
    const tf2::Vector3 velocityOdom(velocityMps[0],
                                    velocityMps[1],
                                    velocityMps[2]);

    /* Rotate the odom-frame velocity into the body frame. */
    const tf2::Vector3 velocityBody =
        tf2::Matrix3x3(orientation).transpose() * velocityOdom;

    /* Publish the body-frame x linear velocity. */
    output.twist.twist.linear.x = velocityBody.x();

    /* Publish the body-frame y linear velocity. */
    output.twist.twist.linear.y = velocityBody.y();

    /* Publish the body-frame z linear velocity. */
    output.twist.twist.linear.z = velocityBody.z();

    /* Angular rate is already a body-frame state; publish it as-is. */
    output.twist.twist.angular.x = filteredAngularRate[0];

    /* Publish the body-frame y angular rate. */
    output.twist.twist.angular.y = filteredAngularRate[1];

    /* Publish the body-frame z angular rate. */
    output.twist.twist.angular.z = filteredAngularRate[2];

    /*!
     * Fixed diagonal covariance values reflecting this estimator's known
     * relative uncertainty (position/orientation drift over time, angular
     * rate/velocity noise) rather than a dynamically tracked covariance.
     */
    output.pose.covariance[0] = 0.25;

    /* Pose covariance: y position variance. */
    output.pose.covariance[7] = 0.25;

    /* Pose covariance: yaw variance (larger; least observable axis). */
    output.pose.covariance[14] = 0.50;

    /* Pose covariance: roll variance. */
    output.pose.covariance[21] = 0.08;

    /* Pose covariance: pitch variance. */
    output.pose.covariance[28] = 0.08;

    /* Pose covariance: yaw-rate-coupled orientation variance. */
    output.pose.covariance[35] = 0.08;

    /* Twist covariance: x linear-velocity variance. */
    output.twist.covariance[0] = 0.10;

    /* Twist covariance: y linear-velocity variance. */
    output.twist.covariance[7] = 0.10;

    /* Twist covariance: z linear-velocity variance (larger). */
    output.twist.covariance[14] = 0.20;

    /* Twist covariance: roll-rate variance. */
    output.twist.covariance[21] = 0.02;

    /* Twist covariance: pitch-rate variance. */
    output.twist.covariance[28] = 0.02;

    /* Twist covariance: yaw-rate variance. */
    output.twist.covariance[35] = 0.02;

    /* Publish the assembled odometry message. */
    odometryPublisher->publish(output);
}

} /* namespace localisation::inertial_odometry */
