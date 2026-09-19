/*!
 * @File:         initialize.cc
 *
 * @Brief:        Implements sizing and seeding of the filter's state and
 *                covariance.
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

FilterStatus ContinuousExtendedKalmanFilter::initialize(
    Eigen::Index           stateSize_in,
    const Eigen::VectorXd &initialState_in,
    const Eigen::MatrixXd &initialCovariance_in) noexcept
{
    /* A non-positive state size has no meaningful filter to construct. */
    if (stateSize_in <= 0)
    {
        return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
    }

    /*!
     * Every dimension below must exactly match the requested state size;
     * with dynamically-sized matrices this is a runtime check the old
     * fixed-size engine got for free at compile time.
     */
    if (initialState_in.size() != stateSize_in ||
        initialCovariance_in.rows() != stateSize_in ||
        initialCovariance_in.cols() != stateSize_in)
    {
        return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
    }

    /*!
     * Reject a configuration containing NaN/Inf before it can corrupt the
     * filter's state.
     */
    if (!initialState_in.allFinite() || !initialCovariance_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
    }

    /*!
     * Every diagonal covariance entry must be strictly positive for the matrix
     * to remain a valid covariance.
     */
    for (Eigen::Index index = 0; index < stateSize_in; ++index)
    {
        if (initialCovariance_in(index, index) <= 0.0)
        {
            return FilterStatus::FILTER_STATUS_INVALID_CONFIGURATION;
        }
    }

    /*!
     * Record the configured size, allocating this filter's dynamic matrices at
     * exactly that size.
     */
    stateSize = stateSize_in;

    /* Seed the state directly from the caller-supplied initial value. */
    state = initialState_in;

    /*!
     * Symmetrize the caller-supplied covariance, guarding against a caller
     * that passed a not-quite-symmetric matrix.
     */
    covariance =
        0.5 * (initialCovariance_in + initialCovariance_in.transpose());

    /*!
     * Only now that every field above is valid does the filter become usable by
     * predict(), update(), getState(), getCovariance(), setState() and
     * terminate().
     */
    isInitialized = true;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
