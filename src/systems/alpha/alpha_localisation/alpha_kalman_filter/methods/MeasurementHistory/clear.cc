/*!
 * @file            clear.cc
 *
 * @brief           Implements removal of all retained fused measurements.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/MeasurementHistory.h"

/* C++ Standard Library Includes */
/* None */

/* C Standard Library Includes */
/* None */

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Object Includes */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void MeasurementHistory::clear() noexcept
{
    recordCount = 0U;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
