/**
 * @file            restoreFilterCheckpointAtOrBefore.cc
 *
 * @brief           Restores one bounded-lag estimator checkpoint.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

bool AlphaKalmanFilterNode::restoreFilterCheckpointAtOrBefore(
    double targetTimestampS_in)
{
    constexpr double        TIMESTAMP_TOLERANCE_S = 1.0e-9;
    const FilterCheckpoint *p_selectedCheckpoint  = nullptr;
    for (std::size_t index = 0U; index < filterCheckpointCount; ++index)
    {
        const FilterCheckpoint &checkpoint =
            filterCheckpoints[(oldestFilterCheckpointIndex + index) %
                              FILTER_CHECKPOINT_CAPACITY];
        if (checkpoint.timestamp_s <=
            targetTimestampS_in + TIMESTAMP_TOLERANCE_S)
        {
            p_selectedCheckpoint = &checkpoint;
        }
        else
        {
            break;
        }
    }
    if (p_selectedCheckpoint == nullptr)
    {
        return false;
    }

    const FilterStatus restoreStatus =
        filter.restore(p_selectedCheckpoint->errorState,
                       p_selectedCheckpoint->covariance);
    if (restoreStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        return false;
    }
    stateTimestamp_s = p_selectedCheckpoint->timestamp_s;
    nominalState     = p_selectedCheckpoint->nominalState;
    latestState      = nominalState;
    latestCovariance = filter.getCovariance();
    hasEstimate      = true;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
