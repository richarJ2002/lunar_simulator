/*!
 * @File:         computeSteeringAngle.cc
 *
 * @Brief:        Implements per-wheel Ackermann/crab steering-angle
 *                computation.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AckermannControllerNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>

namespace control::ackermann_controller
{

double AckermannControllerNode::computeSteeringAngle(double vxMps_in,
                                                     double vyMps_in,
                                                     double yawRateRadps_in,
                                                     double wheelXM_in,
                                                     double wheelYM_in)
{
    /*!
     * Rigid-body composition: the wheel's required contact-point velocity
     * is the commanded body velocity plus the yaw-rate contribution at the
     * wheel's fixed offset (see the class-level derivation). atan2(0, 0)
     * is well-defined (returns 0), so a fully stopped command steers to
     * zero rather than being undefined.
     */
    return std::atan2(vyMps_in + (yawRateRadps_in * wheelXM_in),
                      vxMps_in - (yawRateRadps_in * wheelYM_in));
}

} /* namespace control::ackermann_controller */
