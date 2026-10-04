/*!
 * @file            addRealValue.cc
 *
 * @brief           Implements the real-valued diagnostic entry.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "diagnostics/public_functions/public_functions.h"

/* C++ Standard Library Includes */
#include <array>
#include <cstdio>

/* External Library Includes */
#include <diagnostic_msgs/msg/key_value.hpp>

namespace common::diagnostics
{

void addRealValue(const std::string                      &key_in,
                  double                                  value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout)
{
    /* "%.9g" never needs more than 24 characters, sign and exponent
     * included, so a fixed buffer cannot truncate. */
    std::array<char, 32U> text{};
    static_cast<void>(
        std::snprintf(text.data(), text.size(), "%.9g", value_in));

    diagnostic_msgs::msg::KeyValue entry;
    entry.key   = key_in;
    entry.value = text.data();
    status_inout.values.push_back(entry);
}

} /* namespace common::diagnostics */
