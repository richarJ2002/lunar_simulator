/*!
 * @file            enforceCommandGateCallBack.cc
 *
 * @brief           Implements the between-command gate check and the stop
 *                  sent when an open gate closes.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::enforceCommandGateCallBack()
{
    const CommandGateDecision decision =
        evaluateCommandGate(latestSystemState,
                            now().seconds() - latestSystemStateReceipt_s,
                            maximumStateHeartbeatAgeS);

    if (decision.isOpen && !wasGateOpen)
    {
        LUNAR_LOG_INFO(get_logger(), "CMD gate open");
    }
    else if (!decision.isOpen && wasGateOpen)
    {
        /* One exact zero-velocity command, without actuator noise, holding
         * the last steering angles so stopping never re-steers a wheel. */
        const std::size_t wheelCount = latestForwardedSteering_rad.empty()
                                           ? 6U
                                           : latestForwardedSteering_rad.size();
        actuator_msgs::msg::Actuators stop;
        stop.header.stamp = now();
        stop.position     = latestForwardedSteering_rad.empty()
                                ? std::vector<double>(wheelCount, 0.0)
                                : latestForwardedSteering_rad;
        stop.velocity.assign(wheelCount, 0.0);
        p_rawWheelCommandPublisher->publish(stop);
        ++stopCommandCount;
        LUNAR_LOG_WARN(get_logger(),
                       "CMD gate closed: %s; rover stopped",
                       decision.reason.c_str());
    }
    wasGateOpen = decision.isOpen;
}

} /* namespace systems::alpha::alpha_drivers */
