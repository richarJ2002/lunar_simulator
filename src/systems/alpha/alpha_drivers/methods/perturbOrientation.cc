/*!
 * @file            perturbOrientation.cc
 *
 * @brief           Adds a small body-frame rotation to an orientation.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

/* External Library Includes */
#include <tf2/LinearMath/Quaternion.h>

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::perturbOrientation(
    geometry_msgs::msg::Quaternion &orientation_inout)
{
    /* Load the current orientation as a tf2 quaternion for composing. */
    tf2::Quaternion orientation(orientation_inout.x, orientation_inout.y,
                                orientation_inout.z, orientation_inout.w);

    /*!
     * Composing a small random rotation (rather than perturbing each
     * quaternion component independently) keeps the result a valid unit
     * quaternion up to the final normalize() call, avoiding any bias toward
     * a particular axis that independent-component noise would introduce.
     */
    tf2::Quaternion perturbation;

    /* Sample an independent small roll/pitch/yaw rotation. */
    perturbation.setRPY(sampleGaussian(imuOrientationStddev_rad),
                        sampleGaussian(imuOrientationStddev_rad),
                        sampleGaussian(imuOrientationStddev_rad));

    /* Apply the perturbation on top of the original orientation. */
    orientation *= perturbation;

    /* Renormalize to cancel any floating-point drift from the multiply. */
    orientation.normalize();

    /* Write the perturbed x component back to the ROS message. */
    orientation_inout.x = orientation.x();

    /* Write the perturbed y component back to the ROS message. */
    orientation_inout.y = orientation.y();

    /* Write the perturbed z component back to the ROS message. */
    orientation_inout.z = orientation.z();

    /* Write the perturbed w component back to the ROS message. */
    orientation_inout.w = orientation.w();
}

} /* namespace systems::alpha::alpha_drivers */
