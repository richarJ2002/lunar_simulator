/**
 * @file            discardFilterCheckpointsAfter.cc
 *
 * @brief           Truncates estimator checkpoint history after rollback.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::discardFilterCheckpointsAfter(
    double targetTimestampS_in) noexcept
{
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    std::size_t retainedCount = 0U;
    while (retainedCount < filterCheckpointCount)
    {
        const FilterCheckpoint &checkpoint = filterCheckpoints[
            (oldestFilterCheckpointIndex + retainedCount) %
            FILTER_CHECKPOINT_CAPACITY];
        if (checkpoint.timestamp_s >
            targetTimestampS_in + TIMESTAMP_TOLERANCE_S)
        {
            break;
        }
        ++retainedCount;
    }
    filterCheckpointCount = retainedCount;
    nextFilterCheckpointIndex =
        (oldestFilterCheckpointIndex + retainedCount) %
        FILTER_CHECKPOINT_CAPACITY;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
