/*!
 * @file            enforceCommandGateCallBack.cc
 *
 * @brief           Implements the between-command gate check and the stop
 *                  sent when an open gate closes.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNodeClass.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::enforceCommandGateCallBack()
{
    const CommandGateDecision decision =
        evaluateCommandGate(latestSystemState,
                            now().seconds() - latestSystemStateReceipt_s,
                            maximumStateHeartbeatAge_s);

    /* Announce a gate transition exactly once. */
    if (decision.isOpen && !wasGateOpen)
    {
        LUNAR_LOG_INFO(get_logger(), "CMD gate open");
    }
    else if (!decision.isOpen && wasGateOpen)
    {
        /*!
         * One exact zero-velocity command, without actuator noise, holding
         * the last steering angles so stopping never re-steers a wheel.
         */
        const std::size_t wheelCount = latestForwardedSteering_rad.empty()
                                           ? 6U
                                           : latestForwardedSteering_rad.size();

        /* Build a zero-velocity stop holding last steering. */
        actuator_msgs::msg::Actuators stop;

        /* Stamp the stop with the current time. */
        stop.header.stamp = now();

        /* Hold last steering so stopping never re-steers a wheel. */
        stop.position = latestForwardedSteering_rad.empty()
                            ? std::vector<double>(wheelCount, 0.0)
                            : latestForwardedSteering_rad;

        /* Command zero drive velocity on every wheel. */
        stop.velocity.assign(wheelCount, 0.0);

        /* Publish the stop on the raw command topic. */
        p_rawWheelCommandPublisher->publish(stop);

        /* Count the stop command sent. */
        stopCommandCount++;

        /* Warn with the gate-closure reason. */
        LUNAR_LOG_WARN(get_logger(),
                       "CMD gate closed: %s; rover stopped",
                       decision.reason.c_str());
    }
    wasGateOpen = decision.isOpen;
}

} /* namespace systems::alpha::alpha_drivers */
