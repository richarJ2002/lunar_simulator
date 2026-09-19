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
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::wheel_odometry
{

double WheelOdometryNode::wrapAngle(double angleRad_in)
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
    while (angleRad_in > pi)
    {
        /* Subtract one full turn until back within range. */
        angleRad_in -= 2.0 * pi;
    }

    /* Symmetric wrap for an angle that drifted below -pi. */
    while (angleRad_in < -pi)
    {
        /* Add one full turn until back within range. */
        angleRad_in += 2.0 * pi;
    }

    /* Return the now-wrapped angle. */
    return angleRad_in;
}

} /* namespace localisation::wheel_odometry */
