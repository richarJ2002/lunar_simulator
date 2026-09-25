/*!
 * @file            publishPropagatedEstimate.cc
 *
 * @brief           Implements once-per-propagation estimate publication.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::publishPropagatedEstimate()
{
    /* Sub-nanosecond slack so a period that is an exact multiple of the IMU
     * interval is not skipped by floating-point rounding. */
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    if (!hasEstimate || (lastPublishedStateTimestamp_s >= 0.0 &&
                         stateTimestamp_s - lastPublishedStateTimestamp_s <
                             minimumOutputPeriodS - TIMESTAMP_TOLERANCE_S))
    {
        return;
    }

    /* The message carries the time the state is valid at, not the time it
     * happened to be published. */
    const rclcpp::Time stateStamp(
        static_cast<std::int64_t>(std::llround(stateTimestamp_s * 1.0e9)),
        RCL_ROS_TIME);
    publishEstimate(stateStamp);
    lastPublishedStateTimestamp_s = stateTimestamp_s;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
