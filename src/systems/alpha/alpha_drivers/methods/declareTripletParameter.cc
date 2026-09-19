/*!
 * @file            declareTripletParameter.cc
 *
 * @brief           Declares and validates a three-axis bias parameter.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <stdexcept>

namespace systems::alpha::alpha_drivers
{

std::array<double, AlphaDriverNode::AXIS_COUNT>
AlphaDriverNode::declareTripletParameter(
    const std::string            &name_in,
    const std::array<double, 3U> &defaults_in)
{
    /*!
     * ROS parameters do not support std::array directly, so the default is
     * passed through as a std::vector and converted back after validation.
     */
    const std::vector<double> configuredValues =
        declare_parameter<std::vector<double>>(
            name_in,
            std::vector<double>(defaults_in.begin(), defaults_in.end()));

    /* Reject a configured override with the wrong number of axes. */
    if (configuredValues.size() != AXIS_COUNT)
    {
        /* Fail fast at construction rather than misbehave later. */
        throw std::invalid_argument(name_in +
                                    " must contain exactly three values");
    }

    /* Check every configured axis value before trusting any of them. */
    for (const double configuredValue : configuredValues)
    {
        /* Reject a NaN or infinite bias, which would otherwise silently
         * corrupt every downstream sensor reading. */
        if (!std::isfinite(configuredValue))
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(name_in + " values must be finite");
        }
    }

    /* Every value passed validation; assemble the fixed-size result. */
    return std::array<double, AXIS_COUNT>{configuredValues[X_AXIS],
                                          configuredValues[Y_AXIS],
                                          configuredValues[Z_AXIS]};
}

} /* namespace systems::alpha::alpha_drivers */
