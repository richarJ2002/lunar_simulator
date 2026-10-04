/*!
 * @file            formatText.cc
 *
 * @brief           Implements printf-style formatting into a string.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/public_functions/public_functions.h"

/* C++ Standard Library Includes */
#include <cstdarg>
#include <cstdio>

namespace common::console
{

std::string formatText(const char *format_in, ...)
{
    std::va_list arguments;
    va_start(arguments, format_in);

    /* Measure first with a copy, because a va_list is consumed by use. */
    std::va_list measuringArguments;
    va_copy(measuringArguments, arguments);
    const int requiredLength =
        std::vsnprintf(nullptr, 0U, format_in, measuringArguments);
    va_end(measuringArguments);

    /* An encoding error still logs something rather than nothing. */
    std::string text = format_in;
    if (requiredLength >= 0)
    {
        /* vsnprintf writes a terminator, so reserve one extra byte and
         * drop it again once the text is complete. */
        const std::size_t textLength = static_cast<std::size_t>(requiredLength);
        text.assign(textLength + 1U, '\0');
        static_cast<void>(
            std::vsnprintf(text.data(), text.size(), format_in, arguments));
        text.resize(textLength);
    }
    va_end(arguments);
    return text;
}

} /* namespace common::console */
