/*!
 * @file            declareNonnegativeParameter.cc
 *
 * @brief           Declares and validates one nonnegative noise parameter.
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

double
AlphaDriverNode::declareNonnegativeParameter(const std::string &name_in,
                                             double defaultValue_in)
{
    /* Read the raw parameter value before validating it below. */
    const double parameterValue =
        declare_parameter<double>(name_in, defaultValue_in);

    /*!
     * Noise standard deviations feed directly into std::normal_distribution
     * and covariance diagonals; a NaN, infinite or negative value here would
     * silently corrupt every downstream estimator, so it is rejected at
     * startup instead.
     */
    if (!std::isfinite(parameterValue) || parameterValue < 0.0)
    {
        /* Fail fast at construction rather than misbehave later. */
        throw std::invalid_argument(name_in +
                                    " must be finite and nonnegative");
    }

    /* The value passed every check; hand it back to the caller. */
    return parameterValue;
}

} /* namespace systems::alpha::alpha_drivers */
