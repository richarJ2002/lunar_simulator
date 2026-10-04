/**
 * @file            applyCovarianceTransform.cc
 *
 * @brief           Implements validated covariance reset transformation.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/ContinuousExtendedKalmanFilterClass.h"

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::applyCovarianceTransform(
    const Eigen::MatrixXd &transform_in) noexcept
{
    /* Nothing to transform before initialization. */
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }

    /* The Jacobian must be square, sized and finite. */
    if (transform_in.rows() != stateSize || transform_in.cols() != stateSize ||
        !transform_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /* Apply the reset Jacobian to both covariance sides. */
    Eigen::MatrixXd transformedCovariance =
        transform_in * covariance * transform_in.transpose();

    /* Re-symmetrize; the product need not be exactly symmetric. */
    transformedCovariance =
        0.5 * (transformedCovariance + transformedCovariance.transpose());

    /* Refuse a transform that breaks covariance validity. */
    if (!isCovarianceValid(transformedCovariance))
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    /* Commit only after every check above has passed. */
    covariance = transformedCovariance;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
