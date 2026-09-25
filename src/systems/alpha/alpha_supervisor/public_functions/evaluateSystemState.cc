/*!
 * @file            evaluateSystemState.cc
 *
 * @brief           Implements the supervisor's pure state machine.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "alpha_supervisor/public_functions.h"

namespace systems::alpha::alpha_supervisor
{

SupervisorDecision
    evaluateSystemState(SystemState                            previousState_in,
                        const std::vector<ComponentReadiness> &components_in,
                        double                                 now_s,
                        double supervisorStart_s,
                        double minimumInitWindow_s,
                        double componentTimeout_s)
{
    SupervisorDecision decision;
    decision.requiredCount = components_in.size();

    /* Count fresh, ready components and remember the first that is not. */
    for (const ComponentReadiness &component : components_in)
    {
        const bool isFresh =
            component.hasReport &&
            now_s - component.reportTime_s <= componentTimeout_s;
        if (isFresh && component.isReady)
        {
            ++decision.readyCount;
            continue;
        }
        if (decision.blockingComponent.empty())
        {
            decision.blockingComponent = component.name;
            if (!component.hasReport)
            {
                decision.blockingReason = "no report";
            }
            else if (!isFresh)
            {
                decision.blockingReason = "stale";
            }
            else
            {
                decision.blockingReason = component.reason;
            }
        }
    }
    const bool areAllReady = decision.readyCount == decision.requiredCount;

    switch (previousState_in)
    {
    case SystemState::SYSTEM_STATE_INITIALISING:
        /* Hold the start-up window open even when everything is ready. */
        if (areAllReady && now_s - supervisorStart_s < minimumInitWindow_s)
        {
            decision.blockingReason = "minimum window";
        }
        decision.state =
            areAllReady && now_s - supervisorStart_s >= minimumInitWindow_s
                ? SystemState::SYSTEM_STATE_READY
                : SystemState::SYSTEM_STATE_INITIALISING;
        break;
    case SystemState::SYSTEM_STATE_READY:
    case SystemState::SYSTEM_STATE_HOLD:
        /* After start-up, readiness alone decides; a regression holds. */
        decision.state = areAllReady ? SystemState::SYSTEM_STATE_READY
                                     : SystemState::SYSTEM_STATE_HOLD;
        break;
    }
    return decision;
}

} /* namespace systems::alpha::alpha_supervisor */
