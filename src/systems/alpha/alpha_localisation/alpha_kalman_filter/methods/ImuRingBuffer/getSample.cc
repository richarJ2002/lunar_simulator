/**
 * @file            getSample.cc
 *
 * @brief           Implements chronological IMU sample lookup.
 *
 * @date            20/09/2026
 */

/* Matching Declaration Include */
#include "objects/ImuRingBuffer.h"

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

std::optional<ImuSample>
    ImuRingBuffer::getSample(std::size_t sampleIndex_in) const noexcept
{
    if (sampleIndex_in >= sampleCount)
    {
        return std::nullopt;
    }

    /* nextWriteIndex identifies the oldest entry only when the buffer is
     * full; before that point chronological storage begins at index zero. */
    const std::size_t oldestIndex =
        sampleCount == CAPACITY ? nextWriteIndex : 0U;
    const std::size_t storageIndex = (oldestIndex + sampleIndex_in) % CAPACITY;

    return samples[storageIndex];
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
