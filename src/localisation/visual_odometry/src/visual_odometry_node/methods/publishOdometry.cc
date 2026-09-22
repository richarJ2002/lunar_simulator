/*!
 * @File:         publishOdometry.cc
 *
 * @Brief:        Implements visual-odometry message publication.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cstddef>

#include <opencv2/calib3d.hpp>

namespace localisation::visual_odometry
{

/* This is a private method with a single, already-reviewed call site
 * (handleStereoCallBack.cc), which passes dtS_in/inlierCount_in from distinctly
 * named local variables, not positionally from ambiguous data. */
void VisualOdometryNode::publishOdometry(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    const builtin_interfaces::msg::Time &stamp_in,
    double                               dtS_in,
    const cv::Matx44d                   &previousPose_in,
    const cv::Matx44d                   &currentPose_in,
    const PoseCovariance                &poseCovariance_in,
    const PoseCovariance                &relativeCovariance_in)
{
    /* Message to populate and publish below. */
    nav_msgs::msg::Odometry output;

    /* Timestamp the message with the current stereo frame's stamp. */
    output.header.stamp = stamp_in;

    /* Pose and twist below are expressed in this fixed frame. */
    output.header.frame_id = odomFrame;

    /* Child frame this odometry describes the motion of. */
    output.child_frame_id = baseFrame;

    /* Copy the accumulated x position. */
    output.pose.pose.position.x = currentPose_in(0, 3);

    /* Copy the accumulated y position. */
    output.pose.pose.position.y = currentPose_in(1, 3);

    /* Copy the accumulated z position. */
    output.pose.pose.position.z = currentPose_in(2, 3);

    /* Convert the accumulated rotation matrix to a ROS quaternion. */
    output.pose.pose.orientation = rotationToQuaternion(currentPose_in);

    /*!
     * Twist is derived by finite-differencing pose rather than tracked
     * directly, since this estimator only ever produces discrete relative
     * poses from PnP. previousFromCurrent expresses the relative motion in
     * the BODY frame (matching ROS twist convention for child_frame_id),
     * and dividing by dtS_in converts the per-step displacement into a
     * per-second rate.
     */
    const cv::Matx44d previousFromCurrent =
        multiply(invertRigid(previousPose_in), currentPose_in);

    /* Relative body-frame translation between the two accumulated
     * poses. */
    const cv::Vec3d translationBody(previousFromCurrent(0, 3),
                                    previousFromCurrent(1, 3),
                                    previousFromCurrent(2, 3));

    /* Convert the x displacement into a per-second linear rate. */
    output.twist.twist.linear.x = translationBody[0] / dtS_in;

    /* Convert the y displacement into a per-second linear rate. */
    output.twist.twist.linear.y = translationBody[1] / dtS_in;

    /* Convert the z displacement into a per-second linear rate. */
    output.twist.twist.linear.z = translationBody[2] / dtS_in;

    /* Relative rotation matrix, extracted for the Rodrigues conversion
     * below. */
    cv::Mat relativeRotation(3, 3, CV_64F);

    /* Copy the relative rotation block out of the homogeneous
     * transform. */
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            relativeRotation.at<double>(row, column) =
                previousFromCurrent(row, column);
        }
    }

    /* Axis-angle (Rodrigues) form of the relative rotation, computed
     * below. */
    cv::Mat relativeRotationVector;

    /*!
     * Convert the relative rotation matrix to an axis-angle (Rodrigues)
     * vector: its direction is the rotation axis and its magnitude is the
     * rotation angle in radians, so dividing by dtS_in directly yields an
     * angular-rate vector, matching ROS twist.angular convention.
     */
    cv::Rodrigues(relativeRotation, relativeRotationVector);

    /* Convert the x-axis rotation component into a per-second angular
     * rate. */
    output.twist.twist.angular.x =
        relativeRotationVector.at<double>(0) / dtS_in;

    /* Convert the y-axis rotation component into a per-second angular
     * rate. */
    output.twist.twist.angular.y =
        relativeRotationVector.at<double>(1) / dtS_in;

    /* Convert the z-axis rotation component into a per-second angular
     * rate. */
    output.twist.twist.angular.z =
        relativeRotationVector.at<double>(2) / dtS_in;

    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            const std::size_t covarianceIndex =
                static_cast<std::size_t>(row * 6 + column);
            output.pose.covariance[covarianceIndex] =
                poseCovariance_in(row, column);
            output.twist.covariance[covarianceIndex] =
                relativeCovariance_in(row, column) / (dtS_in * dtS_in);
        }
    }

    /* Publish the fully assembled odometry message. */
    p_odometryPublisher->publish(output);
}

} /* namespace localisation::visual_odometry */
