/*!
 * @File:         computeWheelSpeed_radPs.cc
 *
 * @Brief:        Implements per-wheel drive-rate computation.
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

double AckermannControllerNode::computeWheelSpeed_radPs(double vx_mPs_in,
                                                       double vy_mPs_in,
                                                       double yawRate_radPs_in,
                                                       double wheelX_m_in,
                                                       double wheelY_m_in,
                                                       double wheelRadius_m_in)
{
    /*!
     * Magnitude of the same rigid-body contact-point velocity used by
     * computeSteeringAngle(), converted from linear contact-point speed to
     * angular drive rate by the (validated-positive, at construction)
     * common wheel radius.
     */
    const double contactPointSpeed_mPs =
        std::hypot(vx_mPs_in - (yawRate_radPs_in * wheelY_m_in),
                   vy_mPs_in + (yawRate_radPs_in * wheelX_m_in));

    return contactPointSpeed_mPs / wheelRadius_m_in;
}

} /* namespace control::ackermann_controller */
