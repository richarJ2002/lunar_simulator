/**
 * @file            publishEstimate.cc
 *
 * @brief           Publishes the bias-aware ESKF estimate as odometry.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cstddef>

/* External Library Includes */
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* Object Includes */
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::publishEstimate(const rclcpp::Time &stamp_in)
{
    if (!hasEstimate)
    {
        return;
    }

    const Eigen::Index positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    Eigen::Quaterniond quaternion_bodyToFixed(latestState(quaternionIndex + 3),
                                              latestState(quaternionIndex),
                                              latestState(quaternionIndex + 1),
                                              latestState(quaternionIndex + 2));
    quaternion_bodyToFixed.normalize();
    const Eigen::Matrix3d rotation_bodyToFixed =
        quaternion_bodyToFixed.toRotationMatrix();
    const Eigen::Matrix3d rotation_fixedToBody =
        rotation_bodyToFixed.transpose();
    const Eigen::Vector3d velocity_body_mPerS =
        rotation_fixedToBody * latestState.segment<3>(velocityIndex);

    nav_msgs::msg::Odometry output;
    output.header.stamp         = stamp_in;
    output.header.frame_id      = odomFrame;
    output.child_frame_id       = baseFrame;
    output.pose.pose.position.x = latestState(positionIndex);
    output.pose.pose.position.y = latestState(positionIndex + 1);
    output.pose.pose.position.z = latestState(positionIndex + 2);
    output.pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(quaternion_bodyToFixed.x(),
                                   quaternion_bodyToFixed.y(),
                                   quaternion_bodyToFixed.z(),
                                   quaternion_bodyToFixed.w()));
    output.twist.twist.linear.x  = velocity_body_mPerS.x();
    output.twist.twist.linear.y  = velocity_body_mPerS.y();
    output.twist.twist.linear.z  = velocity_body_mPerS.z();
    output.twist.twist.angular.x = latestAngularVelocity_body_radPerS.x();
    output.twist.twist.angular.y = latestAngularVelocity_body_radPerS.y();
    output.twist.twist.angular.z = latestAngularVelocity_body_radPerS.z();

    Eigen::Matrix<double, 6, ERROR_STATE_SIZE> poseJacobian =
        Eigen::Matrix<double, 6, ERROR_STATE_SIZE>::Zero();
    poseJacobian.block<3, 3>(0, errorPositionIndex) =
        Eigen::Matrix3d::Identity();
    poseJacobian.block<3, 3>(3, errorAttitudeIndex) = rotation_bodyToFixed;
    const Eigen::Matrix<double, 6, 6> poseCovariance =
        poseJacobian * latestCovariance * poseJacobian.transpose();

    Eigen::Matrix3d velocityCrossMatrix = Eigen::Matrix3d::Zero();
    velocityCrossMatrix << 0.0, -velocity_body_mPerS.z(),
        velocity_body_mPerS.y(), velocity_body_mPerS.z(), 0.0,
        -velocity_body_mPerS.x(), -velocity_body_mPerS.y(),
        velocity_body_mPerS.x(), 0.0;
    Eigen::Matrix<double, 6, ERROR_STATE_SIZE> twistJacobian =
        Eigen::Matrix<double, 6, ERROR_STATE_SIZE>::Zero();
    twistJacobian.block<3, 3>(0, errorVelocityIndex) = rotation_fixedToBody;
    twistJacobian.block<3, 3>(0, errorAttitudeIndex) = velocityCrossMatrix;
    twistJacobian.block<3, 3>(3, errorGyroscopeBiasIndex) =
        -Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 6, 6> twistCovariance =
        twistJacobian * latestCovariance * twistJacobian.transpose();
    twistCovariance.block<3, 3>(3, 3) +=
        processNoise.block<3, 3>(errorAttitudeIndex, errorAttitudeIndex);

    for (Eigen::Index row = 0; row < 6; ++row)
    {
        for (Eigen::Index column = 0; column < 6; ++column)
        {
            const std::size_t covarianceIndex =
                static_cast<std::size_t>(row * 6 + column);
            output.pose.covariance[covarianceIndex] =
                poseCovariance(row, column);
            output.twist.covariance[covarianceIndex] =
                twistCovariance(row, column);
        }
    }

    p_outputPublisher->publish(output);
    publishEstimatedPath(output);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
