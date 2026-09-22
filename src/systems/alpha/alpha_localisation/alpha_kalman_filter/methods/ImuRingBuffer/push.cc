/**
 * @file            push.cc
 *
 * @brief           Implements chronological insertion into the IMU ring buffer.
 *
 * @date            20/09/2026
 */

/* Matching Declaration Include */
#include "objects/ImuRingBuffer.h"

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

bool ImuRingBuffer::push(const ImuSample &sample_in) noexcept
{
    /* Reject malformed physical data before it can enter prediction. */
    if (!std::isfinite(sample_in.timestamp_s) ||
        !sample_in.linearAcceleration_body_mPerS2.allFinite() ||
        !sample_in.angularVelocity_body_radPerS.allFinite())
    {
        return false;
    }

    /* Preserve strict chronological order across duplicate messages and
     * simulation-clock reversals. */
    const std::optional<ImuSample> latestSample = getLatestSample();
    if (latestSample.has_value() &&
        sample_in.timestamp_s <= latestSample->timestamp_s)
    {
        return false;
    }

    /* Fixed storage turns insertion at capacity into replacement of the
     * oldest sample, never heap growth in the sensor callback. */
    samples[nextWriteIndex] = sample_in;
    nextWriteIndex          = (nextWriteIndex + 1U) % CAPACITY;
    if (sampleCount < CAPACITY)
    {
        ++sampleCount;
    }

    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
