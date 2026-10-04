/*!
 * @file            wrapText.cc
 *
 * @brief           Implements console word wrapping across paragraphs.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/public_functions/public_functions.h"

/* C++ Standard Library Includes */
#include <algorithm>

/* Other Project Module Includes */
#include "console/private_functions/private_functions.h"

namespace common::console
{

std::vector<std::string> wrapText(const std::string &text_in,
                                  std::size_t        maximumLineLength_in)
{
    /* Two characters of continuation indent plus one of text. */
    constexpr std::size_t MINIMUM_LINE_LENGTH = 3U;
    const std::size_t     lineLength =
        std::max(maximumLineLength_in, MINIMUM_LINE_LENGTH);

    std::vector<std::string> lines;
    std::size_t              paragraphStart = 0U;
    while (paragraphStart <= text_in.size())
    {
        /* Each explicit newline ends one paragraph. */
        const std::size_t paragraphEnd    = text_in.find('\n', paragraphStart);
        const std::size_t paragraphLength = paragraphEnd == std::string::npos
                                                ? std::string::npos
                                                : paragraphEnd - paragraphStart;
        wrapParagraph(text_in.substr(paragraphStart, paragraphLength),
                      lineLength,
                      lines);
        if (paragraphEnd == std::string::npos)
        {
            break;
        }
        paragraphStart = paragraphEnd + 1U;
    }

    /* A log record always carries at least one (possibly empty) line. */
    if (lines.empty())
    {
        lines.emplace_back();
    }
    return lines;
}

} /* namespace common::console */
