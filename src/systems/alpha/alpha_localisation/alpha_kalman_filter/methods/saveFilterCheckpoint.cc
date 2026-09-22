/**
 * @file            saveFilterCheckpoint.cc
 *
 * @brief           Retains one bounded-lag estimator checkpoint.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::saveFilterCheckpoint()
{
    if (!hasInitialState)
    {
        return;
    }

    FilterCheckpoint checkpoint;
    checkpoint.timestamp_s  = stateTimestamp_s;
    checkpoint.nominalState = nominalState;
    checkpoint.errorState   = filter.getState();
    checkpoint.covariance   = filter.getCovariance();
    if (filterCheckpointCount > 0U)
    {
        const std::size_t latestIndex =
            (nextFilterCheckpointIndex + FILTER_CHECKPOINT_CAPACITY - 1U) %
            FILTER_CHECKPOINT_CAPACITY;
        if (std::abs(filterCheckpoints[latestIndex].timestamp_s -
                     stateTimestamp_s) <= 1.0e-9)
        {
            filterCheckpoints[latestIndex] = checkpoint;
            return;
        }
    }

    filterCheckpoints[nextFilterCheckpointIndex] = checkpoint;
    nextFilterCheckpointIndex =
        (nextFilterCheckpointIndex + 1U) % FILTER_CHECKPOINT_CAPACITY;
    if (filterCheckpointCount < FILTER_CHECKPOINT_CAPACITY)
    {
        ++filterCheckpointCount;
    }
    else
    {
        oldestFilterCheckpointIndex = nextFilterCheckpointIndex;
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
