/*!
 * @file            evaluateCommandGate.cc
 *
 * @brief           Implements the pure command-gate decision.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

CommandGateDecision AlphaDriverNode::evaluateCommandGate(
    const std::optional<alpha_supervisor::SystemState> &systemState_in,
    double                                              heartbeatAge_s_in,
    double maximumHeartbeatAge_s_in)
{
    CommandGateDecision decision;
    if (!systemState_in.has_value())
    {
        decision.reason = "no system state";
        return decision;
    }
    if (*systemState_in != alpha_supervisor::SystemState::SYSTEM_STATE_READY)
    {
        decision.reason =
            "system " + alpha_supervisor::systemStateName(*systemState_in);
        return decision;
    }
    /* A negative age (clock reset) is as untrustworthy as a stale one. */
    if (!(heartbeatAge_s_in >= 0.0) ||
        heartbeatAge_s_in > maximumHeartbeatAge_s_in)
    {
        decision.reason = common::console::formatText("heartbeat %.1f s old",
                                                      heartbeatAge_s_in);
        return decision;
    }
    decision.isOpen = true;
    return decision;
}

} /* namespace systems::alpha::alpha_drivers */
