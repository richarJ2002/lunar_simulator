/*!
 * @File:         update.cc
 *
 * @Brief:        Implements one caller-linearized Joseph-form measurement
 *                correction.
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

FilterStatus ContinuousExtendedKalmanFilter::update(
    const Eigen::VectorXd &innovation_in,
    const Eigen::MatrixXd &observationMatrix_in,
    const Eigen::MatrixXd &measurementNoise_in) noexcept
{
    if (!isInitialized)
    {
        return FilterStatus::FILTER_STATUS_NOT_INITIALIZED;
    }

    /*!
     * Measurement dimension is whatever the caller's innovation carries;
     * an empty correction would silently apply a zero-effect update.
     */
    const Eigen::Index measurementSize = innovation_in.size();

    if (measurementSize <= 0)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /*!
     * The observation matrix must map this filter's state space to exactly the
     * innovation's measurement space.
     */
    if (observationMatrix_in.rows() != measurementSize ||
        observationMatrix_in.cols() != stateSize ||
        measurementNoise_in.rows() != measurementSize ||
        measurementNoise_in.cols() != measurementSize)
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    if (!innovation_in.allFinite() || !observationMatrix_in.allFinite() ||
        !measurementNoise_in.allFinite())
    {
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    /*!
     * Innovation covariance S = H P H^T + R, needed to compute the Kalman
     * gain below.
     */
    const Eigen::MatrixXd innovationCovariance =
        observationMatrix_in * covariance * observationMatrix_in.transpose() +
        measurementNoise_in;

    /*!
     * Factor the innovation covariance so it can be inverted (via solve)
     * for the gain below, without forming an explicit inverse.
     */
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovationCovariance);

    /*!
     * A failed factorization means the innovation covariance was not
     * positive definite, so the update cannot proceed numerically.
     */
    if (decomposition.info() != Eigen::Success)
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    /* Kalman gain K = P H^T S^-1, computed via the LDLT solve above
     * rather than an explicit matrix inverse. */
    const Eigen::MatrixXd gain =
        covariance * observationMatrix_in.transpose() *
        decomposition.solve(
            Eigen::MatrixXd::Identity(measurementSize, measurementSize));

    /* Apply the gain-weighted innovation to correct the state estimate. */
    state += gain * innovation_in;

    /*!
     * Joseph-form covariance update: P' = (I - K H) P (I - K H)^T +
     * K R K^T. This form is used instead of the simpler P' = (I - K H) P
     * because it stays numerically symmetric and positive semidefinite
     * even when K is not the exact optimal gain.
     */
    const Eigen::MatrixXd residual =
        Eigen::MatrixXd::Identity(stateSize, stateSize) -
        gain * observationMatrix_in;

    /* Evaluate the Joseph-form expression above. */
    covariance = residual * covariance * residual.transpose() +
                 gain * measurementNoise_in * gain.transpose();

    /*!
     * Re-symmetrize to cancel the asymmetry floating-point arithmetic
     * accumulates over repeated updates.
     */
    covariance = 0.5 * (covariance + covariance.transpose());

    if (!state.allFinite() || !covariance.allFinite())
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
