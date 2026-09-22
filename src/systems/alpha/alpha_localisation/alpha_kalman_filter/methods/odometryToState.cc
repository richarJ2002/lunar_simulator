/**
 * @file            odometryToState.cc
 *
 * @brief           Implements conversion from ROS odometry to nominal state.
 *
 * @date            20/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
/* None */

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::NominalStateVector
    AlphaKalmanFilterNode::odometryToState(
        const nav_msgs::msg::Odometry &message_in) const
{
    NominalStateVector measurement = NominalStateVector::Zero();

    tf2::Quaternion quaternion_bodyToFixed;
    tf2::fromMsg(message_in.pose.pose.orientation, quaternion_bodyToFixed);
    quaternion_bodyToFixed.normalize();

    const Eigen::Index positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);

    measurement.segment<3>(positionIndex) =
        Eigen::Vector3d(message_in.pose.pose.position.x,
                        message_in.pose.pose.position.y,
                        message_in.pose.pose.position.z);
    measurement(quaternionIndex)     = quaternion_bodyToFixed.x();
    measurement(quaternionIndex + 1) = quaternion_bodyToFixed.y();
    measurement(quaternionIndex + 2) = quaternion_bodyToFixed.z();
    measurement(quaternionIndex + 3) = quaternion_bodyToFixed.w();

    return measurement;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
