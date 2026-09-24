/*!
 * @file            wrapParagraph.cc
 *
 * @brief           Implements greedy word wrapping of one paragraph.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/private_functions.h"

/* C++ Standard Library Includes */
#include <algorithm>

namespace common::console
{

void wrapParagraph(const std::string        &paragraph_in,
                   std::size_t               lineLength_in,
                   std::vector<std::string> &lines_inout)
{
    /* Continuation lines are indented so a wrapped message reads as one. */
    const std::string CONTINUATION_INDENT(2U, ' ');

    std::string currentLine;
    bool        isFirstLine = true;

    /* Text capacity of the line being built, excluding any indent. */
    const auto lineCapacity =
        [&isFirstLine, &CONTINUATION_INDENT, lineLength_in]()
    {
        return isFirstLine ? lineLength_in
                           : lineLength_in - CONTINUATION_INDENT.size();
    };

    /* Emits the line being built, indenting it if it continues a line. */
    const auto emitLine = [&]()
    {
        lines_inout.push_back(isFirstLine ? currentLine
                                          : CONTINUATION_INDENT + currentLine);
        currentLine.clear();
        isFirstLine = false;
    };

    std::size_t position = 0U;
    while (position < paragraph_in.size())
    {
        /* Skip the run of separators before the next word. */
        const std::size_t wordStart =
            paragraph_in.find_first_not_of(" \t", position);
        if (wordStart == std::string::npos)
        {
            break;
        }
        const std::size_t wordEnd =
            std::min(paragraph_in.find_first_of(" \t", wordStart),
                     paragraph_in.size());
        std::string word = paragraph_in.substr(wordStart, wordEnd - wordStart);
        position         = wordEnd;

        while (!word.empty())
        {
            const std::size_t separatorLength = currentLine.empty() ? 0U : 1U;
            if (currentLine.size() + separatorLength + word.size() <=
                lineCapacity())
            {
                /* The whole word fits on the current line. */
                if (separatorLength > 0U)
                {
                    currentLine += ' ';
                }
                currentLine += word;
                word.clear();
            }
            else if (currentLine.empty())
            {
                /* The word is longer than a whole line: split it. */
                const std::size_t chunkLength = lineCapacity();
                currentLine                   = word.substr(0U, chunkLength);
                word.erase(0U, chunkLength);
                emitLine();
            }
            else
            {
                /* Move the word to a fresh continuation line. */
                emitLine();
            }
        }
    }

    /* Emit the final partial line, if any text remains. */
    if (!currentLine.empty())
    {
        emitLine();
    }
}

} /* namespace common::console */
