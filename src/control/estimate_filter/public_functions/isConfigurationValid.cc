/*!
 * @file            isConfigurationValid.cc
 *
 * @brief           Implements the estimate filter's configuration check.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/public_functions/public_functions.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <initializer_list>

namespace control::estimate_filter
{

bool isConfigurationValid(const LowPassConfiguration &configuration_in)
{
    /* Each value becomes a time constant or a limit and must be a finite,
     * positive number. */
    for (const double value : {configuration_in.velocityCutoffHz,
                               configuration_in.positionCutoffHz,
                               configuration_in.attitudeCutoffHz,
                               configuration_in.maximumGap_s})
    {
        if (!std::isfinite(value) || !(value > 0.0))
        {
            return false;
        }
    }
    return true;
}

} /* namespace control::estimate_filter */
