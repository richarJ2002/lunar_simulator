/*!
 * @File:         publishWheelSlip.cc
 *
 * @Brief:        Implements publication of the fused per-wheel slip state.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::publishWheelSlip()
{
    std_msgs::msg::Float64MultiArray output;

    output.data.resize(static_cast<std::size_t>(WHEEL_COUNT));

    /* Publish the current cached per-wheel slip states, in wheel order;
     * latestState defaults to zero ("no slip assumed") until the first
     * real observation is fused. */
    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        output.data[static_cast<std::size_t>(wheel)] =
            latestState(SLIP_STATE_START_INDEX + wheel);
    }

    p_wheelSlipPublisher->publish(output);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
