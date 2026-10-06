/*!
 * @file            private_functions.h
 *
 * @brief           Declares the console module's internal helpers.
 *
 * @date            24/09/2026
 */

#ifndef SRS_COMMON_CONSOLE_PRIVATE_FUNCTIONS_H
#define SRS_COMMON_CONSOLE_PRIVATE_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <cstddef>
#include <string>
#include <vector>

namespace common::console
{

/*!
 * @brief           Word-wraps one newline-free paragraph and appends its
 *                  lines.
 *
 * @param[in]       paragraph_in
 *                  Text containing no newline characters.
 *
 * @param[in]       lineLength_in
 *                  Longest permitted line in characters, at least 3.
 *
 * @param[in,out]   lines_inout
 *                  Line list the paragraph's lines are appended to;
 *                  nothing is appended for a blank paragraph.
 */
void wrapParagraph(const std::string        &paragraph_in,
                   std::size_t               lineLength_in,
                   std::vector<std::string> &lines_inout);

} /* namespace common::console */

#endif /* SRS_COMMON_CONSOLE_PRIVATE_FUNCTIONS_H */
