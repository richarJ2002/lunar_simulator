/**
 * @file            clearFilterCheckpoints.cc
 *
 * @brief           Clears bounded-lag estimator checkpoints.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::clearFilterCheckpoints() noexcept
{
    nextFilterCheckpointIndex = 0U;
    oldestFilterCheckpointIndex = 0U;
    filterCheckpointCount = 0U;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
