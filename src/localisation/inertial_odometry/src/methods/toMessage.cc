/*!
 * @File:         toMessage.cc
 *
 * @Brief:        Converts a tf2 quaternion to a message quaternion.
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
/* None */

namespace localisation::inertial_odometry
{

geometry_msgs::msg::Quaternion
    InertialOdometryNode::toMessage(const tf2::Quaternion &quaternion)
{
    /*!
     * tf2::Quaternion and geometry_msgs::msg::Quaternion store the same
     * component order but are unrelated types, so each field is copied by
     * hand below rather than relying on an implicit conversion.
     */
    geometry_msgs::msg::Quaternion result;

    /* Copy the x component. */
    result.x = quaternion.x();

    /* Copy the y component. */
    result.y = quaternion.y();

    /* Copy the z component. */
    result.z = quaternion.z();

    /* Copy the w (scalar) component. */
    result.w = quaternion.w();

    /* Return the fully populated message. */
    return result;
}

} /* namespace localisation::inertial_odometry */
