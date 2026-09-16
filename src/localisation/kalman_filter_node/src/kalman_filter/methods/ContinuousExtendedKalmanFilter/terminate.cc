/*!
 * @File:         terminate.cc
 *
 * @Brief:        Terminates the continuous extended Kalman filter lifecycle.
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
/* None */

namespace lunar_simulator::localisation::kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::terminate() noexcept
{
    state.setZero();
    covariance.setIdentity();
    processNoise.setZero();
    stateTimestampS = 0.0;
    hasTimestamp = false;
    isInitialized = false;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
