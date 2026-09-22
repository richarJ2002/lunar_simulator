/**
 * @file            clear.cc
 *
 * @brief           Implements removal of all retained IMU samples.
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

void ImuRingBuffer::clear() noexcept
{
    nextWriteIndex = 0U;
    sampleCount    = 0U;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
