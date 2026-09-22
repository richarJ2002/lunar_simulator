/**
 * @file            getSampleCount.cc
 *
 * @brief           Implements IMU ring-buffer sample-count access.
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

std::size_t ImuRingBuffer::getSampleCount() const noexcept
{
    return sampleCount;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
