/*!
 * @File:         loadSixValues.cc
 *
 * @Brief:        Implements six-value wheel-parameter loading with fallback.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AckermannControllerNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>

namespace control::ackermann_controller
{

void AckermannControllerNode::loadSixValues(
    const std::string         &name_in,
    const std::vector<double> &defaults_in,
    std::array<double, 6>     &values_out)
{
    /* Read the configured override, or provided defaults when none was set */
    std::vector<double> configured =
        declare_parameter<std::vector<double>>(name_in, defaults_in);

    /*!
     * Every six-wheel parameter (wheel x/y position, drive direction) must
     * contain exactly one value per wheel in the fixed front-left..rear-right
     * order documented on the class.
     */
    if (configured.size() != values_out.size())
    {
        /*!
         * A misconfigured override (wrong length) is treated as an
         * operator mistake rather than a fatal error: warn and fall back
         * to the known-good Alpha defaults so the node still starts and
         * produces plausible wheel commands.
         */
        RCLCPP_WARN(get_logger(),
                    "Parameter %s needs six values; using defaults",
                    name_in.c_str());

        /* Discard the invalid override and use the caller's defaults. */
        configured = defaults_in;
    }

    /* Copy the validated (or defaulted) six values into the output. */
    std::copy(configured.begin(), configured.end(), values_out.begin());
}

} /* namespace control::ackermann_controller */
