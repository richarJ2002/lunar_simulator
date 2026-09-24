/*!
 * @file            public_functions.h
 *
 * @brief           Declares the helpers that append typed key/value entries
 *                  to a diagnostic_msgs DiagnosticStatus.
 *
 *                  Values are written as text in one fixed format per type
 *                  (see each function), which is the contract
 *                  post_processing/python_tools/diagnostics/
 *                  bag_diagnostics.py parses. Each type has its own
 *                  function name rather than an overload, so an integer
 *                  literal can never silently select the flag or real
 *                  form.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_COMMON_DIAGNOSTICS_PUBLIC_FUNCTIONS_H
#define LUNAR_SIMULATOR_COMMON_DIAGNOSTICS_PUBLIC_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <cstdint>
#include <string>

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace common::diagnostics
{

/*!
 * @brief           Appends a real-valued entry, written with "%.9g" so
 *                  small and large magnitudes keep nine significant digits
 *                  ("nan" and "inf" for non-finite values).
 *
 * @param[in]       key_in
 *                  Entry key.
 *
 * @param[in]       value_in
 *                  Entry value, in the unit the key's name states.
 *
 * @param[in,out]   status_inout
 *                  Status the entry is appended to.
 */
void addRealValue(const std::string                      &key_in,
                  double                                  value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout);

/*!
 * @brief           Appends an unsigned counter entry, written in decimal.
 *
 * @param[in]       key_in
 *                  Entry key.
 *
 * @param[in]       value_in
 *                  Entry value.
 *
 * @param[in,out]   status_inout
 *                  Status the entry is appended to.
 */
void addCountValue(const std::string                      &key_in,
                   std::uint64_t                           value_in,
                   diagnostic_msgs::msg::DiagnosticStatus &status_inout);

/*!
 * @brief           Appends a boolean entry, written as "true" or "false".
 *
 * @param[in]       key_in
 *                  Entry key.
 *
 * @param[in]       value_in
 *                  Entry value.
 *
 * @param[in,out]   status_inout
 *                  Status the entry is appended to.
 */
void addFlagValue(const std::string                      &key_in,
                  bool                                    value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout);

/*!
 * @brief           Appends a text entry unchanged.
 *
 * @param[in]       key_in
 *                  Entry key.
 *
 * @param[in]       value_in
 *                  Entry value.
 *
 * @param[in,out]   status_inout
 *                  Status the entry is appended to.
 */
void addTextValue(const std::string                      &key_in,
                  const std::string                      &value_in,
                  diagnostic_msgs::msg::DiagnosticStatus &status_inout);

} /* namespace common::diagnostics */

#endif /* LUNAR_SIMULATOR_COMMON_DIAGNOSTICS_PUBLIC_FUNCTIONS_H */
