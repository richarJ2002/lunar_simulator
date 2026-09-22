/*!
 * @file            calculateGravityFreeAccelerationFixed.cc
 *
 * @brief           Implements tilt-aware gravity removal.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/InertialOdometryNode.h"

/* External Library Includes */
#include <tf2/LinearMath/Matrix3x3.h>

namespace localisation::inertial_odometry
{

tf2::Vector3 InertialOdometryNode::calculateGravityFreeAccelerationFixed(
    const tf2::Quaternion &orientationBodyToFixed_in,
    const tf2::Vector3 &specificForceBody_in,
    const tf2::Vector3 &gravitySpecificForceFixed_in)
{
    return tf2::Matrix3x3(orientationBodyToFixed_in) * specificForceBody_in -
           gravitySpecificForceFixed_in;
}

} /* namespace localisation::inertial_odometry */
