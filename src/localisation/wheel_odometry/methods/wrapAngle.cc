/*!
 * @File:         wrapAngle.cc
 *
 * @Brief:        Implements wrapping an angle to [-pi, pi].
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNodeClass.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::wheel_odometry
{

double WheelOdometryNode::wrapAngle(double angle_rad_in)
{
    /* A local named constant reads better than the bare literal below. */
    constexpr double pi = 3.14159265358979323846;

    /*!
     * Integrated yaw would otherwise grow without bound as the rover
     * turns repeatedly; wrapping keeps it in the conventional [-pi, pi]
     * range used throughout the odometry and TF outputs. A simple
     * iterative wrap is sufficient because a single integration step
     * never advances yaw by more than a small fraction of a full turn.
     */
    while (angle_rad_in > pi)
    {
        /* Subtract one full turn until back within range. */
        angle_rad_in -= 2.0 * pi;
    }

    /* Symmetric wrap for an angle that drifted below -pi. */
    while (angle_rad_in < -pi)
    {
        /* Add one full turn until back within range. */
        angle_rad_in += 2.0 * pi;
    }

    /* Return the now-wrapped angle. */
    return angle_rad_in;
}

} /* namespace localisation::wheel_odometry */
