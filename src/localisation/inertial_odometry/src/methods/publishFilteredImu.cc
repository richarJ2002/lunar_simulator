/*!
 * @File:         publishFilteredImu.cc
 *
 * @Brief:        Publishes the bias-corrected, filtered IMU sample.
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

void InertialOdometryNode::publishFilteredImu(
    const sensor_msgs::msg::Imu &input,
    const tf2::Vector3          &accelerationOdom)
{
    /*!
     * Reuse the input message so unrelated fields (e.g. covariance layout
     * conventions already applied upstream) are preserved, then overwrite
     * only the fields this node actually filters.
     */
    sensor_msgs::msg::Imu output = input;

    /* Republish under this node's own body frame id. */
    output.header.frame_id = baseFrame;

    /* Publish the integrated orientation as the filtered orientation. */
    output.orientation = toMessage(orientation);

    /* Publish the filtered x angular rate. */
    output.angular_velocity.x = filteredAngularRate[0];

    /* Publish the filtered y angular rate. */
    output.angular_velocity.y = filteredAngularRate[1];

    /* Publish the filtered z angular rate. */
    output.angular_velocity.z = filteredAngularRate[2];

    /*!
     * The published IMU acceleration is expressed in the body frame, so the
     * gravity-free odom-frame acceleration computed for integration is
     * rotated back into the body frame using the orientation's transpose
     * (i.e. the inverse rotation for an orthonormal rotation matrix).
     */
    const tf2::Vector3 gravityFreeAccelerationBody =
        tf2::Matrix3x3(orientation).transpose() * accelerationOdom;

    /* Publish the body-frame x acceleration. */
    output.linear_acceleration.x = gravityFreeAccelerationBody.x();

    /* Publish the body-frame y acceleration. */
    output.linear_acceleration.y = gravityFreeAccelerationBody.y();

    /* Publish the body-frame z acceleration. */
    output.linear_acceleration.z = gravityFreeAccelerationBody.z();

    /* Publish the assembled filtered IMU message. */
    filteredImuPublisher->publish(output);
}

} /* namespace localisation::inertial_odometry */
