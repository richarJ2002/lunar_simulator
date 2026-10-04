/*!
 * @file            isSeverityEnabled.cc
 *
 * @brief           Implements the console severity-level check.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/public_functions/public_functions.h"

/* External Library Includes */
#include <rcutils/logging.h>

namespace common::console
{

bool isSeverityEnabled(const rclcpp::Logger &logger_in,
                       ConsoleSeverity       severity_in)
{
    /* rcutils severities are spaced 10 apart from DEBUG = 10. */
    int rcutilsSeverity = RCUTILS_LOG_SEVERITY_FATAL;
    switch (severity_in)
    {
    case ConsoleSeverity::CONSOLE_SEVERITY_DEBUG:
        rcutilsSeverity = RCUTILS_LOG_SEVERITY_DEBUG;
        break;
    case ConsoleSeverity::CONSOLE_SEVERITY_INFO:
        rcutilsSeverity = RCUTILS_LOG_SEVERITY_INFO;
        break;
    case ConsoleSeverity::CONSOLE_SEVERITY_WARN:
        rcutilsSeverity = RCUTILS_LOG_SEVERITY_WARN;
        break;
    case ConsoleSeverity::CONSOLE_SEVERITY_ERROR:
        rcutilsSeverity = RCUTILS_LOG_SEVERITY_ERROR;
        break;
    case ConsoleSeverity::CONSOLE_SEVERITY_FATAL:
        rcutilsSeverity = RCUTILS_LOG_SEVERITY_FATAL;
        break;
    }

    /* Logging may be queried before any RCLCPP_* macro has run, so make
     * sure rcutils logging has been initialised first. */
    RCUTILS_LOGGING_AUTOINIT;
    return rcutils_logging_logger_is_enabled_for(logger_in.get_name(),
                                                 rcutilsSeverity);
}

} /* namespace common::console */
