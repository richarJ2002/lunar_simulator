/**
 * @file            restore.cc
 *
 * @brief           Implements validated estimator checkpoint restoration.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::restore(
    const Eigen::VectorXd &state_in,
    const Eigen::MatrixXd &covariance_in) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }
    if (state_in.size() != stateSize || covariance_in.rows() != stateSize ||
        covariance_in.cols() != stateSize || !state_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    const Eigen::MatrixXd symmetricCovariance =
        0.5 * (covariance_in + covariance_in.transpose());
    if (!isCovarianceValid(symmetricCovariance))
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    state      = state_in;
    covariance = symmetricCovariance;
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
