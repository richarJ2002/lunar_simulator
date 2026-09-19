/*!
 * @File:         computeWheelSpeedRadps.cc
 *
 * @Brief:        Implements per-wheel drive-rate computation.
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

double AckermannControllerNode::computeWheelSpeedRadps(double vxMps_in,
                                                       double vyMps_in,
                                                       double yawRateRadps_in,
                                                       double wheelXM_in,
                                                       double wheelYM_in,
                                                       double wheelRadiusM_in)
{
    /*!
     * Magnitude of the same rigid-body contact-point velocity used by
     * computeSteeringAngle(), converted from linear contact-point speed to
     * angular drive rate by the (validated-positive, at construction)
     * common wheel radius.
     */
    const double contactPointSpeedMps =
        std::hypot(vxMps_in - (yawRateRadps_in * wheelYM_in),
                   vyMps_in + (yawRateRadps_in * wheelXM_in));

    return contactPointSpeedMps / wheelRadiusM_in;
}

} /* namespace control::ackermann_controller */
