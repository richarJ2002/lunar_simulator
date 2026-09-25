/*!
 * @file            public_functions.h
 *
 * @brief           Declares the estimate filter's configuration check.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_PUBLIC_FUNCTIONS_H
#define LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_PUBLIC_FUNCTIONS_H

/* Object Includes */
#include "estimate_filter/objects/LowPassConfiguration.h"

namespace control::estimate_filter
{

/*!
 * @brief           Checks a low-pass configuration.
 *
 * @param[in]       configuration_in
 *                  Configuration to check.
 *
 * @return          True when every corner frequency and the gap limit are
 *                  finite and positive.
 */
bool isConfigurationValid(const LowPassConfiguration &configuration_in);

} /* namespace control::estimate_filter */

#endif /* LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_PUBLIC_FUNCTIONS_H */
