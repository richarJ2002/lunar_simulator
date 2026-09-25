/*!
 * @file            getRecord.cc
 *
 * @brief           Implements chronological fused-measurement access.
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

std::optional<FusedMeasurement>
    MeasurementHistory::getRecord(std::size_t recordIndex_in) const noexcept
{
    if (recordIndex_in >= recordCount)
    {
        return std::nullopt;
    }
    return records[recordIndex_in];
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
