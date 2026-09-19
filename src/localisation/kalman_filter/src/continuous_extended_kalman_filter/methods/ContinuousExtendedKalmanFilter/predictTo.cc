/*!
 * @File:         predictTo.cc
 *
 * @Brief:        Propagates the EKF to a requested monotonic timestamp.
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
#include <algorithm>

namespace lunar_simulator::localisation::kalman_filter
{

FilterStatus
ContinuousExtendedKalmanFilter::predictTo(double targetTimestampS_in) noexcept
{
    double remainingTimeS = targetTimestampS_in - stateTimestampS;
    constexpr double timestampToleranceS = 1.0e-9;
    if (remainingTimeS < -timestampToleranceS)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }
    if (remainingTimeS <= 0.0)
    {
        return FilterStatus::FILTER_STATUS_SUCCESS;
    }

    while (remainingTimeS > 0.0)
    {
        const double timeStepS = std::min(remainingTimeS, 0.02);
        const FilterStatus status = predictStep(timeStepS);
        if (status != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return status;
        }
        stateTimestampS += timeStepS;
        remainingTimeS -= timeStepS;
    }

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
