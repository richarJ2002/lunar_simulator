/*!
 * @File:         handleWheelSlipEstimateCallBack.cc
 *
 * @Brief:        Implements application of continuous_ekf's fused per-wheel
 *                slip estimate to its own wheel.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Generic Libraries */
#include <algorithm>
#include <cmath>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::handleWheelSlipEstimateCallBack(
    const std_msgs::msg::Float64MultiArray &message_in)
{
    if (!shouldApplySlipFeedback)
    {
        return;
    }

    if (message_in.data.size() != slipRatios.size())
    {
        /*!
         * A malformed message (wrong element count) must never reach the
         * rolling-constraint solve; keep whatever was applied before.
         * continuous_ekf always publishes exactly slipRatios.size()
         * elements (see AlphaKalmanFilterNode::publishWheelSlip()).
         */
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Ignoring wheel-slip estimate with %zu elements (expected %zu)",
            message_in.data.size(),
            slipRatios.size());

        return;
    }

    /*!
     * Each wheel's own fused estimate is applied to that wheel alone --
     * unlike the old shared-value model, a single wheel's slip no longer
     * has to speak for the whole rover (see AlphaKalmanFilterNode's own
     * class-level doc comment for why rotational slip needs this).
     */
    for (std::size_t wheel = 0U; wheel < slipRatios.size(); ++wheel)
    {
        const double estimate = message_in.data[wheel];

        if (!std::isfinite(estimate))
        {
            /* A non-finite fused estimate for this wheel must never reach
             * the rolling-constraint solve; keep whatever was applied
             * before for it and continue with the rest. */
            continue;
        }

        /* Defensively clamp into range even though continuous_ekf already
         * bounds its own slip states the same way -- this node must stay
         * correct even if used with a different fusion source. */
        slipRatios[wheel] =
            std::clamp(estimate, -maximumSlipRatio, maximumSlipRatio);
    }

    /* Republish immediately so a subscriber to the applied-ratio topic
     * sees the freshly fused values without waiting for the next
     * joint-state cycle. */
    publishSlipRatios();
}

} /* namespace localisation::wheel_odometry */
