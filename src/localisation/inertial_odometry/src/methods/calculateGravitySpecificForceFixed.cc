/*!
 * @file            calculateGravitySpecificForceFixed.cc
 *
 * @brief           Implements the stationary gravity-direction prior.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/InertialOdometryNode.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace localisation::inertial_odometry
{

tf2::Vector3 InertialOdometryNode::calculateGravitySpecificForceFixed(
    const tf2::Vector3 &stationaryMeanBody_in,
    double gravityMagnitudeMps2_in)
{
    const double measuredMagnitude = stationaryMeanBody_in.length();
    if (!std::isfinite(measuredMagnitude) || measuredMagnitude <= 1.0e-12 ||
        !std::isfinite(gravityMagnitudeMps2_in) ||
        gravityMagnitudeMps2_in <= 0.0)
    {
        return tf2::Vector3(0.0, 0.0, 0.0);
    }
    return stationaryMeanBody_in *
           (gravityMagnitudeMps2_in / measuredMagnitude);
}

} /* namespace localisation::inertial_odometry */
