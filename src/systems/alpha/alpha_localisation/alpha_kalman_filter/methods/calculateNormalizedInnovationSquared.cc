/**
 * @file            calculateNormalizedInnovationSquared.cc
 *
 * @brief           Implements positive-definite NIS calculation.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

bool AlphaKalmanFilterNode::calculateNormalizedInnovationSquared(
    const Eigen::VectorXd &innovation_in,
    const Eigen::MatrixXd &observationMatrix_in,
    const Eigen::MatrixXd &stateCovariance_in,
    const Eigen::MatrixXd &measurementNoise_in,
    double                &nis_out)
{
    const Eigen::Index measurementSize = innovation_in.size();
    if (measurementSize <= 0 ||
        observationMatrix_in.rows() != measurementSize ||
        observationMatrix_in.cols() != stateCovariance_in.rows() ||
        stateCovariance_in.rows() != stateCovariance_in.cols() ||
        measurementNoise_in.rows() != measurementSize ||
        measurementNoise_in.cols() != measurementSize ||
        !innovation_in.allFinite() || !observationMatrix_in.allFinite() ||
        !stateCovariance_in.allFinite() || !measurementNoise_in.allFinite())
    {
        return false;
    }

    const Eigen::MatrixXd innovationCovariance =
        observationMatrix_in * stateCovariance_in *
            observationMatrix_in.transpose() +
        measurementNoise_in;
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovationCovariance);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive())
    {
        return false;
    }

    nis_out = innovation_in.dot(decomposition.solve(innovation_in));
    return std::isfinite(nis_out) && nis_out >= 0.0;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
