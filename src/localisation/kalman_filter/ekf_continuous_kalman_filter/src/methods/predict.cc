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

    /* Compute into temporaries so a rejected numerical step cannot corrupt
     * the last valid posterior retained by the engine. */
    const Eigen::VectorXd predictedState =
        state + stateDerivative_in * dtS_in;

    /* First-order discretization Phi = I + F dt. Propagating covariance as
     * Phi P Phi^T + Q dt preserves positive semidefiniteness; directly
     * applying forward Euler to Pdot can make a valid covariance indefinite
     * when strongly coupled state blocks begin at very different scales. */
    const Eigen::MatrixXd transitionMatrix =
        Eigen::MatrixXd::Identity(stateSize, stateSize) +
        processJacobian_in * dtS_in;
    Eigen::MatrixXd predictedCovariance =
        transitionMatrix * covariance * transitionMatrix.transpose() +
        processNoise_in * dtS_in;

    /*!
     * Re-symmetrize to cancel the asymmetry floating-point arithmetic
     * accumulates over repeated updates.
     */
    predictedCovariance =
        0.5 * (predictedCovariance + predictedCovariance.transpose());

    if (!predictedState.allFinite() ||
        !isCovarianceValid(predictedCovariance))
    {
        /*!
         * A non-finite result means this step's linearization broke down
         * numerically; report it rather than propagate garbage.
         */
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }

    state      = predictedState;
    covariance = predictedCovariance;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
