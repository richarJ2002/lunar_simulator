/*!
 * @File:         wrapAngle.cc
 *
 * @Brief:        Implements canonical [-pi, pi] angle wrapping.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <cmath>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

double AlphaKalmanFilterNode::wrapAngle(double angleRad_in)
{
    /*!
     * atan2(sin(x), cos(x)) is a numerically robust way to wrap any angle
     * into [-pi, pi] without a branching modulo.
     */
    return std::atan2(std::sin(angleRad_in), std::cos(angleRad_in));
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
