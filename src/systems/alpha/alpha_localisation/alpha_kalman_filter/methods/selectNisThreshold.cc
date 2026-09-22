/**
 * @file            selectNisThreshold.cc
 *
 * @brief           Selects configured or 99% chi-square NIS thresholds.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <array>
#include <limits>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

double AlphaKalmanFilterNode::selectNisThreshold(
    double       configuredThreshold_in,
    Eigen::Index measurementDimension_in)
{
    if (configuredThreshold_in > 0.0)
    {
        return configuredThreshold_in;
    }

    /* 99th percentiles of chi-square distributions for 1..12 degrees of
     * freedom, covering every currently configurable active update size. */
    constexpr std::array<double, 12> CHI_SQUARE_99_PERCENT{
        6.635,  9.210,  11.345, 13.277, 15.086, 16.812,
        18.475, 20.090, 21.666, 23.209, 24.725, 26.217};
    if (measurementDimension_in <= 0 ||
        measurementDimension_in >
            static_cast<Eigen::Index>(CHI_SQUARE_99_PERCENT.size()))
    {
        return std::numeric_limits<double>::infinity();
    }
    return CHI_SQUARE_99_PERCENT[static_cast<std::size_t>(
        measurementDimension_in - 1)];
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
