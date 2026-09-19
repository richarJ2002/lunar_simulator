/*!
 * @File:         getState.cc
 *
 * @Brief:        Implements the current-state accessor.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

const Eigen::VectorXd &ContinuousExtendedKalmanFilter::getState() const noexcept
{
    return state;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
