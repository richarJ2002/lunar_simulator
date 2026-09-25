/*!
 * @file            insert.cc
 *
 * @brief           Implements timestamp-ordered insertion into the fused
 *                  measurement history.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/MeasurementHistory.h"

/* C++ Standard Library Includes */
#include <cmath>

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

bool MeasurementHistory::insert(const FusedMeasurement &record_in) noexcept
{
    /* A non-finite stamp has no position in the replay order. */
    if (!std::isfinite(record_in.timestamp_s))
    {
        return false;
    }

    /* At capacity, drop the oldest record by shifting the rest down. A new
     * record older than every retained one would be the next to go, so it
     * is refused rather than displacing a newer record. */
    if (recordCount == CAPACITY)
    {
        if (record_in.timestamp_s < records[0U].timestamp_s)
        {
            return false;
        }
        for (std::size_t index = 1U; index < CAPACITY; ++index)
        {
            records[index - 1U] = records[index];
        }
        --recordCount;
    }

    /* Records normally arrive in order, so search from the newest end and
     * shift only the few records stamped after the new one. Equal stamps
     * keep arrival order. */
    std::size_t insertIndex = recordCount;
    while (insertIndex > 0U &&
           records[insertIndex - 1U].timestamp_s > record_in.timestamp_s)
    {
        records[insertIndex] = records[insertIndex - 1U];
        --insertIndex;
    }
    records[insertIndex] = record_in;
    ++recordCount;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
