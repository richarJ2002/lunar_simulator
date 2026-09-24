/*!
 * @file            console.h
 *
 * @brief           Public entry point of the console module: the LUNAR_LOG_*
 *                  macros every project node uses instead of RCLCPP_*.
 *
 *                  Every message is formatted, word-wrapped to at most
 *                  MAXIMUM_MESSAGE_LENGTH characters of text per line, and
 *                  written as one log record per line, so the operator's
 *                  terminal never carries a project line whose text is
 *                  longer than 40 characters. A CTest lint
 *                  (test/lint/check_console_logging.py) fails if project
 *                  code outside this module calls RCLCPP_* directly.
 *
 *                  These are macros, not functions, for two reasons C++17
 *                  functions cannot provide: each throttled call site needs
 *                  its own function-local static ThrottleGate (the same
 *                  per-call-site state RCLCPP_*_THROTTLE keeps), and a
 *                  disabled severity must skip argument formatting
 *                  entirely. All real work happens in the ordinary,
 *                  unit-tested functions declared in public_functions.h.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_H
#define LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_H

/* C++ Standard Library Includes */
#include <cstdint>

/* External Library Includes */
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

/* Other Project Module Includes */
#include "console/public_functions.h"

/* Object Includes */
#include "console/objects/ConsoleSeverity.h"
#include "console/objects/ThrottleGate.h"

/*!
 * @brief           Formats a printf-style message and logs it wrapped.
 *
 * @param           logger
 *                  rclcpp::Logger (or expression yielding one).
 * @param           severity
 *                  common::console::ConsoleSeverity of every line.
 * @param           ...
 *                  printf-style format string and its arguments.
 */
#define LUNAR_LOG(logger, severity, ...)                                       \
    do                                                                         \
    {                                                                          \
        const ::rclcpp::Logger &lunarLogLogger = (logger);                     \
        if (::common::console::isSeverityEnabled(lunarLogLogger, (severity)))  \
        {                                                                      \
            ::common::console::logWrapped(                                     \
                lunarLogLogger,                                                \
                (severity),                                                    \
                ::common::console::formatText(__VA_ARGS__));                   \
        }                                                                      \
    } while (false)

/*!
 * @brief           Logs a wrapped message at most once per period at one
 *                  call site.
 *
 * @param           logger
 *                  rclcpp::Logger (or expression yielding one).
 * @param           severity
 *                  common::console::ConsoleSeverity of every line.
 * @param           clock
 *                  rclcpp::Clock whose time drives the period; a node's
 *                  *get_clock() follows simulation time under use_sim_time.
 * @param           periodMs
 *                  Minimum interval between messages, in milliseconds.
 * @param           ...
 *                  printf-style format string and its arguments.
 */
#define LUNAR_LOG_THROTTLE(logger, severity, clock, periodMs, ...)             \
    do                                                                         \
    {                                                                          \
        static ::common::console::ThrottleGate lunarLogThrottleGate;           \
        constexpr std::int64_t LUNAR_LOG_NANOSECONDS_PER_MILLISECOND =         \
            1000000;                                                           \
        if (lunarLogThrottleGate.shouldEmit(                                   \
                (clock).now().nanoseconds(),                                   \
                static_cast<std::int64_t>(periodMs) *                          \
                    LUNAR_LOG_NANOSECONDS_PER_MILLISECOND))                    \
        {                                                                      \
            LUNAR_LOG((logger), (severity), __VA_ARGS__);                      \
        }                                                                      \
    } while (false)

/*! @brief Wrapped DEBUG message; see LUNAR_LOG. */
#define LUNAR_LOG_DEBUG(logger, ...)                                           \
    LUNAR_LOG((logger),                                                        \
              ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_DEBUG,      \
              __VA_ARGS__)

/*! @brief Wrapped INFO message; see LUNAR_LOG. */
#define LUNAR_LOG_INFO(logger, ...)                                            \
    LUNAR_LOG((logger),                                                        \
              ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_INFO,       \
              __VA_ARGS__)

/*! @brief Wrapped WARN message; see LUNAR_LOG. */
#define LUNAR_LOG_WARN(logger, ...)                                            \
    LUNAR_LOG((logger),                                                        \
              ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_WARN,       \
              __VA_ARGS__)

/*! @brief Wrapped ERROR message; see LUNAR_LOG. */
#define LUNAR_LOG_ERROR(logger, ...)                                           \
    LUNAR_LOG((logger),                                                        \
              ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_ERROR,      \
              __VA_ARGS__)

/*! @brief Throttled wrapped INFO message; see LUNAR_LOG_THROTTLE. */
#define LUNAR_LOG_INFO_THROTTLE(logger, clock, periodMs, ...)                  \
    LUNAR_LOG_THROTTLE(                                                        \
        (logger),                                                              \
        ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_INFO,             \
        clock,                                                                 \
        periodMs,                                                              \
        __VA_ARGS__)

/*! @brief Throttled wrapped WARN message; see LUNAR_LOG_THROTTLE. */
#define LUNAR_LOG_WARN_THROTTLE(logger, clock, periodMs, ...)                  \
    LUNAR_LOG_THROTTLE(                                                        \
        (logger),                                                              \
        ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_WARN,             \
        clock,                                                                 \
        periodMs,                                                              \
        __VA_ARGS__)

/*! @brief Throttled wrapped ERROR message; see LUNAR_LOG_THROTTLE. */
#define LUNAR_LOG_ERROR_THROTTLE(logger, clock, periodMs, ...)                 \
    LUNAR_LOG_THROTTLE(                                                        \
        (logger),                                                              \
        ::common::console::ConsoleSeverity::CONSOLE_SEVERITY_ERROR,            \
        clock,                                                                 \
        periodMs,                                                              \
        __VA_ARGS__)

#endif /* LUNAR_SIMULATOR_COMMON_CONSOLE_CONSOLE_H */
