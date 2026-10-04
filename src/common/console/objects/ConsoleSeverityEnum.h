/*!
 * @file            ConsoleSeverityEnum.h
 *
 * @brief           Declares the severity levels accepted by the wrapped
 *                  console logging helpers.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_SEVERITY_ENUM_H
#define LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_SEVERITY_ENUM_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace common::console
{

/*!
 * @brief           Console severity of one wrapped log message, mapped
 *                  one-to-one onto the rcutils severities by logWrapped().
 */
enum class ConsoleSeverity : std::uint8_t
{
    /*!
     * @brief       Detailed information that is hidden at the default INFO
     *              level.
     */
    CONSOLE_SEVERITY_DEBUG = 0U,

    /*!
     * @brief       Normal operator-facing progress information.
     */
    CONSOLE_SEVERITY_INFO = 1U,

    /*!
     * @brief       A recoverable problem the operator should notice.
     */
    CONSOLE_SEVERITY_WARN = 2U,

    /*!
     * @brief       A failure of the current operation.
     */
    CONSOLE_SEVERITY_ERROR = 3U,

    /*!
     * @brief       A failure the node cannot continue from.
     */
    CONSOLE_SEVERITY_FATAL = 4U
};

} /* namespace common::console */

#endif /* LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_SEVERITY_ENUM_H */
