/*!
 * @File:         setState.cc
 *
 * @Brief:        Implements caller-driven state overwrite.
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

FilterStatus ContinuousExtendedKalmanFilter::setState(
    const Eigen::VectorXd &state_in) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }

    if (state_in.size() != stateSize)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    if (!state_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    state = state_in;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
