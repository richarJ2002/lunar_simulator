/*!
 * @File:         wrapAngle.cc
 *
 * @Brief:        Wraps one filter angle to its principal interval.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "kalman_filter/objects/ContinuousExtendedKalmanFilter.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>

namespace lunar_simulator::localisation::kalman_filter
{

double ContinuousExtendedKalmanFilter::wrapAngle(double angleRad_in) noexcept
{
    constexpr double pi = 3.14159265358979323846;
    return std::remainder(angleRad_in, 2.0 * pi);
}

} /* namespace lunar_simulator::localisation::kalman_filter */
