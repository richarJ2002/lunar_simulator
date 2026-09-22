/*!
 * @file            publishStartupFixedTransform.cc
 *
 * @brief           Publishes the static map-to-startup-fixed transform.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/GroundTruthNode.h"

/* Data Includes */
#include <geometry_msgs/msg/transform_stamped.hpp>

/* External Library Includes */
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace localisation::ground_truth
{

void GroundTruthNode::publishStartupFixedTransform(
    const builtin_interfaces::msg::Time &stamp_in)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp_in;
    transform.header.frame_id = mapFrame;
    transform.child_frame_id = startupFixedFrame;
    transform.transform = tf2::toMsg(mapFromStartupFixed);
    p_staticTransformBroadcaster->sendTransform(transform);
}

} /* namespace localisation::ground_truth */
