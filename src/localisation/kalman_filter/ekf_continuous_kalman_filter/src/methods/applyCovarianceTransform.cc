/**
 * @file            applyCovarianceTransform.cc
 *
 * @brief           Implements validated covariance reset transformation.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::applyCovarianceTransform(
    const Eigen::MatrixXd &transform_in) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }
    if (transform_in.rows() != stateSize || transform_in.cols() != stateSize ||
        !transform_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    Eigen::MatrixXd transformedCovariance =
        transform_in * covariance * transform_in.transpose();
    transformedCovariance =
        0.5 * (transformedCovariance + transformedCovariance.transpose());
    if (!isCovarianceValid(transformedCovariance))
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }
    covariance = transformedCovariance;
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
