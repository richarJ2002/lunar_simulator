/*!
 * @File:         predict.cc
 *
 * @Brief:        Implements one caller-linearized continuous EKF prediction
 *                step.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

/* Generic Libraries */
#include <cmath>

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::predict(
    double                 dtS_in,
    const Eigen::VectorXd &stateDerivative_in,
    const Eigen::MatrixXd &processJacobian_in,
    const Eigen::MatrixXd &processNoise_in) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }

    /* A negative or non-finite step cannot be integrated meaningfully. */
    if (!std::isfinite(dtS_in) || dtS_in < 0.0)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /*!
     * Every caller-supplied matrix must match this filter's configured state
     * size in every dimension.
     */
    if (stateDerivative_in.size() != stateSize ||
        processJacobian_in.rows() != stateSize ||
        processJacobian_in.cols() != stateSize ||
        processNoise_in.rows() != stateSize ||
        processNoise_in.cols() != stateSize)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    if (!stateDerivative_in.allFinite() || !processJacobian_in.allFinite() ||
        !processNoise_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /* Forward-Euler-integrate the state by this step's derivative. */
    state += stateDerivative_in * dtS_in;

    /* Continuous covariance equation: Pdot = F P + P F^T + Q. */
    covariance +=
        (processJacobian_in * covariance +
         covariance * processJacobian_in.transpose() + processNoise_in) *
        dtS_in;

    /*!
     * Re-symmetrize to cancel the asymmetry floating-point arithmetic
     * accumulates over repeated updates.
     */
    covariance = 0.5 * (covariance + covariance.transpose());

    if (!state.allFinite() || !covariance.allFinite())
    {
        /*!
         * A non-finite result means this step's linearization broke down
         * numerically; report it rather than propagate garbage.
         */
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
