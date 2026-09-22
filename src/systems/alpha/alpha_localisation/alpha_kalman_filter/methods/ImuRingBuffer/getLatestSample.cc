/**
 * @file            getLatestSample.cc
 *
 * @brief           Implements newest IMU sample lookup.
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

std::optional<ImuSample> ImuRingBuffer::getLatestSample() const noexcept
{
    if (sampleCount == 0U)
    {
        return std::nullopt;
    }

    const std::size_t latestIndex = (nextWriteIndex + CAPACITY - 1U) % CAPACITY;
    return samples[latestIndex];
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
