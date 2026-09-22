/**
 * @file            calculateAttitudeResetJacobian.cc
 *
 * @brief           Implements the right-error attitude reset Jacobian.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

Eigen::Matrix3d AlphaKalmanFilterNode::calculateAttitudeResetJacobian(
    const Eigen::Vector3d &attitudeCorrectionBodyRad_in)
{
    Eigen::Matrix3d correctionSkew;
    correctionSkew << 0.0,
        -attitudeCorrectionBodyRad_in.z(),
        attitudeCorrectionBodyRad_in.y(),
        attitudeCorrectionBodyRad_in.z(),
        0.0,
        -attitudeCorrectionBodyRad_in.x(),
        -attitudeCorrectionBodyRad_in.y(),
        attitudeCorrectionBodyRad_in.x(),
        0.0;

    const double correctionMagnitudeRad =
        attitudeCorrectionBodyRad_in.norm();
    if (correctionMagnitudeRad < 1.0e-6)
    {
        return Eigen::Matrix3d::Identity() - 0.5 * correctionSkew +
               (1.0 / 6.0) * correctionSkew * correctionSkew;
    }

    const double magnitudeSquared =
        correctionMagnitudeRad * correctionMagnitudeRad;
    const double firstCoefficient =
        (1.0 - std::cos(correctionMagnitudeRad)) / magnitudeSquared;
    const double secondCoefficient =
        (correctionMagnitudeRad - std::sin(correctionMagnitudeRad)) /
        (magnitudeSquared * correctionMagnitudeRad);
    return Eigen::Matrix3d::Identity() - firstCoefficient * correctionSkew +
           secondCoefficient * correctionSkew * correctionSkew;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
