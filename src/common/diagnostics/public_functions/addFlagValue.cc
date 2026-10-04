/*!
 * @file            addFlagValue.cc
 *
 * @brief           Implements the boolean diagnostic entry.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "diagnostics/public_functions/public_functions.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/key_value.hpp>

namespace common::diagnostics
{

void addFlagValue(const std::string                      &key_in,
                  bool                                    value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout)
{
    diagnostic_msgs::msg::KeyValue entry;
    entry.key   = key_in;
    entry.value = value_in ? "true" : "false";
    status_inout.values.push_back(entry);
}

} /* namespace common::diagnostics */
