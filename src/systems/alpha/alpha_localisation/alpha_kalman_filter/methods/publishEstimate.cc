/*!
 * @File:         publishEstimate.cc
 *
 * @Brief:        Implements publication of the fused estimate as odometry,
 *                then hands it to publishEstimatedPath() for TF/path.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <cstddef>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::publishEstimate(const rclcpp::Time &stamp_in)
{
    if (!hasEstimate)
    {
        /*!
         * Nothing has been published yet; avoid emitting a message built
         * from the default-constructed (identity) state before the filter
         * has actually produced a posterior.
         */
        return;
    }

    /* Destination for the attitude reconstructed from state below. */
    tf2::Quaternion orientation;

    /* Reconstruct the orientation quaternion from the state's roll,
     * pitch and yaw. */
    orientation.setRPY(latestState(3), latestState(4), latestState(5));

    /* Guard against floating-point drift in the reconstructed
     * quaternion. */
    orientation.normalize();

    /*!
     * The EKF's linear-velocity state is expressed in the map frame, but
     * nav_msgs::msg::Odometry.twist.linear is defined in the child (body)
     * frame by ROS convention, so rotate it back into body before
     * publishing. Angular rate is already a body-frame state and needs no
     * rotation.
     */
    const tf2::Vector3 velocityOdom(latestState(6), latestState(7),
                                    latestState(8));

    /* Apply the map-to-body rotation described above. */
    const tf2::Vector3 velocityBody =
        tf2::Matrix3x3(orientation).transpose() * velocityOdom;

    /* Destination odometry message assembled below. */
    nav_msgs::msg::Odometry output;

    /* Timestamp the message with the caller-supplied stamp. */
    output.header.stamp = stamp_in;

    /* Parent frame configured for this node. */
    output.header.frame_id = odomFrame;

    /* Child frame configured for this node. */
    output.child_frame_id = baseFrame;

    /* Copy the x position from the state vector. */
    output.pose.pose.position.x = latestState(0);

    /* Copy the y position. */
    output.pose.pose.position.y = latestState(1);

    /* Copy the z position. */
    output.pose.pose.position.z = latestState(2);

    /* Convert the reconstructed quaternion to the ROS message type. */
    output.pose.pose.orientation = tf2::toMsg(orientation);

    /* Copy the body-frame x linear velocity computed above. */
    output.twist.twist.linear.x = velocityBody.x();

    /* Copy the body-frame y linear velocity. */
    output.twist.twist.linear.y = velocityBody.y();

    /* Copy the body-frame z linear velocity. */
    output.twist.twist.linear.z = velocityBody.z();

    /* Copy the body-frame roll rate straight from the state vector. */
    output.twist.twist.angular.x = latestState(9);

    /* Copy the body-frame pitch rate. */
    output.twist.twist.angular.y = latestState(10);

    /* Copy the body-frame yaw rate. */
    output.twist.twist.angular.z = latestState(11);

    for (Eigen::Index index = 0; index < 6; ++index)
    {
        /*!
         * Only the six diagonal pose/twist covariance entries are filled;
         * the EKF's full 12x12 covariance carries cross-terms that
         * nav_msgs::msg::Odometry's two independent 6x6 blocks cannot
         * represent, so off-diagonal coupling is intentionally dropped
         * here rather than misrepresented.
         */
        const std::size_t covarianceIndex =
            static_cast<std::size_t>(index * 6 + index);

        /* Copy this axis's pose variance onto the output message. */
        output.pose.covariance[covarianceIndex] =
            latestCovariance(index, index);

        /* Copy this axis's twist variance onto the output message. */
        output.twist.covariance[covarianceIndex] =
            latestCovariance(index + 6, index + 6);
    }

    /* Publish the fully assembled odometry message. */
    p_outputPublisher->publish(output);

    /* Validate this same estimate and, if valid, broadcast its TF
     * transform and append it to the retained estimated path. */
    publishEstimatedPath(output);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
