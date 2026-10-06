/*!
 * @file            public_functions.h
 *
 * @brief           Declares the public console formatting, wrapping and
 *                  logging functions behind the SRS_LOG_* macros.
 *
 * @date            24/09/2026
 */

#ifndef SRS_COMMON_CONSOLE_PUBLIC_FUNCTIONS_H
#define SRS_COMMON_CONSOLE_PUBLIC_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <cstddef>
#include <string>
#include <vector>

/* External Library Includes */
#include <rclcpp/logger.hpp>

/* Object Includes */
#include "console/objects/ConsoleSeverityEnum.h"

namespace common::console
{

/*!
 * @brief           Longest message text, in characters, printed on one
 *                  console line after the fixed "[LEVEL] [node]: " prefix.
 */
inline constexpr std::size_t MAXIMUM_MESSAGE_LENGTH = 40U;

/*!
 * @brief           Formats a printf-style message into an owned string.
 *
 *                  The format attribute lets GCC and Clang check every
 *                  argument against its conversion, as they do for
 *                  RCLCPP_* macros.
 *
 * @param[in]       format_in
 *                  printf-style format string; must not be null.
 *
 * @return          The formatted text, or format_in itself when the
 *                  arguments cannot be encoded.
 */
[[gnu::format(printf, 1, 2)]] std::string formatText(const char *format_in,
                                                     ...);

/*!
 * @brief           Word-wraps text into console lines no longer than a
 *                  limit.
 *
 *                  Explicit newlines start a new line. Words are packed
 *                  greedily; continuation lines of one paragraph are
 *                  indented by two spaces, and the indent counts toward
 *                  the limit. A word longer than a line is split. Runs of
 *                  spaces and tabs collapse to one separator, and blank
 *                  paragraphs are dropped.
 *
 * @param[in]       text_in
 *                  Text to wrap.
 *
 * @param[in]       maximumLineLength_in
 *                  Longest permitted line in characters; values below 3
 *                  are treated as 3 so an indented continuation can always
 *                  carry at least one character.
 *
 * @return          At least one line; a single empty line for empty or
 *                  whitespace-only text.
 */
std::vector<std::string> wrapText(const std::string &text_in,
                                  std::size_t        maximumLineLength_in);

/*!
 * @brief           Reports whether a logger would print a message of the
 *                  given severity, so disabled messages are never
 *                  formatted.
 *
 * @param[in]       logger_in
 *                  Logger the message would be written to.
 *
 * @param[in]       severity_in
 *                  Severity of the message.
 *
 * @return          True when the logger's effective level admits the
 *                  severity.
 */
bool isSeverityEnabled(const rclcpp::Logger &logger_in,
                       ConsoleSeverity       severity_in);

/*!
 * @brief           Wraps text at MAXIMUM_MESSAGE_LENGTH and writes each
 *                  resulting line as its own log record.
 *
 *                  This is the only project code that calls the RCLCPP_*
 *                  logging macros directly; see console.h.
 *
 * @param[in]       logger_in
 *                  Logger the lines are written to.
 *
 * @param[in]       severity_in
 *                  Severity applied to every line.
 *
 * @param[in]       text_in
 *                  Already formatted message text.
 */
void logWrapped(const rclcpp::Logger &logger_in,
                ConsoleSeverity       severity_in,
                const std::string    &text_in);

} /* namespace common::console */

#endif /* SRS_COMMON_CONSOLE_PUBLIC_FUNCTIONS_H */
