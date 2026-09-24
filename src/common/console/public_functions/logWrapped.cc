/*!
 * @file            logWrapped.cc
 *
 * @brief           Implements wrapped console logging, the one place the
 *                  project calls the RCLCPP_* logging macros.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/public_functions.h"

/* External Library Includes */
#include <rclcpp/logging.hpp>

namespace common::console
{

void logWrapped(const rclcpp::Logger &logger_in,
                ConsoleSeverity       severity_in,
                const std::string    &text_in)
{
    /* One log record per wrapped line keeps every console line short. The
     * "%s" format stops text containing '%' being reinterpreted. */
    for (const std::string &line : wrapText(text_in, MAXIMUM_MESSAGE_LENGTH))
    {
        switch (severity_in)
        {
        case ConsoleSeverity::CONSOLE_SEVERITY_DEBUG:
            RCLCPP_DEBUG(logger_in, "%s", line.c_str());
            break;
        case ConsoleSeverity::CONSOLE_SEVERITY_INFO:
            RCLCPP_INFO(logger_in, "%s", line.c_str());
            break;
        case ConsoleSeverity::CONSOLE_SEVERITY_WARN:
            RCLCPP_WARN(logger_in, "%s", line.c_str());
            break;
        case ConsoleSeverity::CONSOLE_SEVERITY_ERROR:
            RCLCPP_ERROR(logger_in, "%s", line.c_str());
            break;
        case ConsoleSeverity::CONSOLE_SEVERITY_FATAL:
            RCLCPP_FATAL(logger_in, "%s", line.c_str());
            break;
        }
    }
}

} /* namespace common::console */
