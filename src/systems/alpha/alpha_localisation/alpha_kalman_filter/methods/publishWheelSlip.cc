/*!
 * @File:         publishWheelSlip.cc
 *
 * @Brief:        Publishes the retained neutral no-slip compatibility value.
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

    /* Slip is no longer an ESKF state. Keep the existing topic stable while
     * wheel feedback remains disabled by publishing the neutral prior. */
    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        output.data[static_cast<std::size_t>(wheel)] = 0.0;
    }

    p_wheelSlipPublisher->publish(output);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
