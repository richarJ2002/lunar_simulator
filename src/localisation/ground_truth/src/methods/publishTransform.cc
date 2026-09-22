/*!
 * @File:         publishTransform.cc
 *
 * @Brief:        Publishes startup-fixed to ground-truth body transform.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/GroundTruthNode.h"

/* Data include */
#include <geometry_msgs/msg/transform_stamped.hpp>

/* Generic Libraries */
/* None */

namespace localisation::ground_truth
{

void GroundTruthNode::publishTransform(
    const nav_msgs::msg::Odometry &odometry_in)
{
    /* Build one transform message to broadcast below. */
    geometry_msgs::msg::TransformStamped transform;

    /* Reuse the odometry's timestamp and parent frame id. */
    transform.header = odometry_in.header;

    /* Reuse the odometry's already-rewritten child frame id. */
    transform.child_frame_id = odometry_in.child_frame_id;

    /*!
     * Copy the x position across; TF needs only the rigid pose, not the
     * odometry twist.
     */
    transform.transform.translation.x = odometry_in.pose.pose.position.x;

    /* Copy the y position across. */
    transform.transform.translation.y = odometry_in.pose.pose.position.y;

    /* Copy the z position across. */
    transform.transform.translation.z = odometry_in.pose.pose.position.z;

    /*!
     * Copy the orientation across; it is already expressed in the same map
     * frame TF requires.
     */
    transform.transform.rotation = odometry_in.pose.pose.orientation;

    /* Broadcast the assembled transform to the TF tree. */
    p_transformBroadcaster->sendTransform(transform);
}

} /* namespace localisation::ground_truth */
