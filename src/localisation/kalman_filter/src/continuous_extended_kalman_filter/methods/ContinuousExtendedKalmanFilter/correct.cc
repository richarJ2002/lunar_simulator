/*!
 * @File:         correct.cc
 *
 * @Brief:        Applies one direct-state EKF measurement correction.
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

FilterStatus ContinuousExtendedKalmanFilter::correct(
    const StateVector &measurement_in,
    const MeasurementMask &measurementMask_in,
    const StateVector &measurementVariances_in) noexcept
{
    if (!measurement_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    StateMatrix observation = StateMatrix::Zero();
    StateMatrix measurementNoise = StateMatrix::Identity();
    StateVector innovation = StateVector::Zero();
    bool hasMeasurement = false;
    for (Eigen::Index index = 0; index < stateSize; ++index)
    {
        if (measurementMask_in[static_cast<std::size_t>(index)])
        {
            const double variance = measurementVariances_in(index);
            if (!std::isfinite(variance) || variance <= 0.0)
            {
                return FilterStatus::FILTER_STATUS_INVALID_INPUT;
            }
            hasMeasurement = true;
            observation(index, index) = 1.0;
            measurementNoise(index, index) = variance;
            innovation(index) = measurement_in(index) - state(index);
            if (index >= 3 && index <= 5)
            {
                innovation(index) = wrapAngle(innovation(index));
            }
        }
    }
    if (!hasMeasurement)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    const StateMatrix innovationCovariance =
        observation * covariance * observation.transpose() + measurementNoise;
    const Eigen::LDLT<StateMatrix> decomposition(innovationCovariance);
    if (decomposition.info() != Eigen::Success)
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }
    const StateMatrix gain = covariance * observation.transpose() *
                             decomposition.solve(StateMatrix::Identity());
    state += gain * innovation;
    state(3) = wrapAngle(state(3));
    state(4) = wrapAngle(state(4));
    state(5) = wrapAngle(state(5));

    /* Joseph covariance update preserves symmetry and positive semidefiniteness. */
    const StateMatrix residual = StateMatrix::Identity() - gain * observation;
    covariance = residual * covariance * residual.transpose() +
                 gain * measurementNoise * gain.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());

    if (!state.allFinite() || !covariance.allFinite())
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
