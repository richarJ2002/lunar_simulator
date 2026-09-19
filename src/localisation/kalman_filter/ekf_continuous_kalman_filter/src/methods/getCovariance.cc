/*!
 * @File:         getCovariance.cc
 *
 * @Brief:        Implements the current-covariance accessor.
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

const Eigen::MatrixXd &
    ContinuousExtendedKalmanFilter::getCovariance() const noexcept
{
    return covariance;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
