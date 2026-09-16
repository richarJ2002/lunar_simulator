/*!
 * @File:         init.cc
 *
 * @Brief:        Initializes the continuous extended Kalman filter.
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

FilterStatus ContinuousExtendedKalmanFilter::init(
    const StateMatrix &processNoise_in,
    const StateMatrix &initialCovariance_in) noexcept
{
    if (!processNoise_in.allFinite() || !initialCovariance_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
    }

    for (Eigen::Index index = 0; index < stateSize; ++index)
    {
        if (processNoise_in(index, index) < 0.0 ||
            initialCovariance_in(index, index) <= 0.0)
        {
            return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
        }
    }

    state.setZero();
    covariance =
        0.5 * (initialCovariance_in + initialCovariance_in.transpose());
    processNoise = 0.5 * (processNoise_in + processNoise_in.transpose());
    stateTimestampS = 0.0;
    hasTimestamp = false;
    isInitialized = true;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
