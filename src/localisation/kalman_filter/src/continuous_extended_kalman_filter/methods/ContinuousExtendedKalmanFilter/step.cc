/*!
 * @File:         step.cc
 *
 * @Brief:        Executes one high-level continuous-discrete EKF step.
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

FilterStatus ContinuousExtendedKalmanFilter::step(
    double timestampS_in, const StateVector *p_measurement_in,
    const MeasurementMask &measurementMask_in,
    const StateVector &measurementVariances_in,
    StateVector &state_out, StateMatrix &covariance_out) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!std::isfinite(timestampS_in))
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    if (!hasTimestamp)
    {
        stateTimestampS = timestampS_in;
        hasTimestamp = true;
        if (p_measurement_in != nullptr)
        {
            if (!p_measurement_in->allFinite())
            {
                return FilterStatus::FILTER_STATUS_INVALID_INPUT;
            }
            for (Eigen::Index index = 0; index < stateSize; ++index)
            {
                if (measurementMask_in[static_cast<std::size_t>(index)])
                {
                    state(index) = (*p_measurement_in)(index);
                }
            }
        }
    }

    FilterStatus status = predictTo(timestampS_in);
    if (status != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        return status;
    }
    if (p_measurement_in != nullptr)
    {
        status = correct(*p_measurement_in, measurementMask_in,
                         measurementVariances_in);
        if (status != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return status;
        }
    }

    state_out = state;
    covariance_out = covariance;
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
