/*!
 * @file            addTextValue.cc
 *
 * @brief           Implements the text diagnostic entry.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "diagnostics/public_functions.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/key_value.hpp>

namespace common::diagnostics
{

void addTextValue(const std::string                      &key_in,
                  const std::string                      &value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout)
{
    diagnostic_msgs::msg::KeyValue entry;
    entry.key   = key_in;
    entry.value = value_in;
    status_inout.values.push_back(entry);
}

} /* namespace common::diagnostics */
