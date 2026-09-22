/**
 * @file            isCovarianceValid.cc
 *
 * @brief           Implements covariance validity checks.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

/* External Library Includes */
#include <Eigen/Eigenvalues>

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

bool ContinuousExtendedKalmanFilter::isCovarianceValid(
    const Eigen::MatrixXd &covariance_in) noexcept
{
    if (covariance_in.rows() <= 0 ||
        covariance_in.rows() != covariance_in.cols() ||
        !covariance_in.allFinite())
    {
        return false;
    }
    constexpr double SYMMETRY_TOLERANCE = 1.0e-12;
    constexpr double MINIMUM_EIGENVALUE = -1.0e-10;
    if (!covariance_in.isApprox(covariance_in.transpose(),
                                SYMMETRY_TOLERANCE))
    {
        return false;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> decomposition(
        covariance_in, Eigen::EigenvaluesOnly);
    return decomposition.info() == Eigen::Success &&
           decomposition.eigenvalues().minCoeff() >= MINIMUM_EIGENVALUE;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
