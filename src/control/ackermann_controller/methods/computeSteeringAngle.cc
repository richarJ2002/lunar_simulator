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
#include "objects/AckermannControllerNodeClass.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>

namespace control::ackermann_controller
{

double AckermannControllerNode::computeSteeringAngle(double vx_mPs_in,
                                                     double vy_mPs_in,
                                                     double yawRate_radPs_in,
                                                     double wheelX_m_in,
                                                     double wheelY_m_in)
{
    /*!
     * Rigid-body composition: the wheel's required contact-point velocity
     * is the commanded body velocity plus the yaw-rate contribution at the
     * wheel's fixed offset (see the class-level derivation). atan2(0, 0)
     * is well-defined (returns 0), so a fully stopped command steers to
     * zero rather than being undefined.
     */
    return std::atan2(vy_mPs_in + (yawRate_radPs_in * wheelX_m_in),
                      vx_mPs_in - (yawRate_radPs_in * wheelY_m_in));
}

} /* namespace control::ackermann_controller */
