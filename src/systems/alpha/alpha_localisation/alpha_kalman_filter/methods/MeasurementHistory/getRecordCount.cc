/*!
 * @file            getRecordCount.cc
 *
 * @brief           Implements fused-measurement history record-count access.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/MeasurementHistoryClass.h"

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

std::size_t MeasurementHistory::getRecordCount() const noexcept
{
    return recordCount;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
