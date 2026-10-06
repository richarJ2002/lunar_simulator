/*!
 * @file            publishSystemStateCallBack.cc
 *
 * @brief           Implements system state evaluation and publication.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "alpha_supervisor/objects/AlphaStartupSupervisorNodeClass.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "diagnostics/diagnostics.h"

namespace systems::alpha::alpha_supervisor
{

void AlphaStartupSupervisorNode::publishSystemStateCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Under simulation time the clock reads zero until /clock arrives; the
     * start-up window cannot begin before time does. */
    const double now_s = now().seconds();
    if (!(now_s > 0.0))
    {
        return;
    }
    if (!hasStarted)
    {
        hasStarted        = true;
        supervisorStart_s = now_s;
    }

    const SystemState        previousState = systemState;
    const SupervisorDecision decision      = evaluateSystemState(previousState,
                                                            components,
                                                            now_s,
                                                            supervisorStart_s,
                                                            minimumInitWindowS,
                                                            componentTimeoutS);
    systemState                            = decision.state;

    /* One short operator line per state, naming what is still pending. */
    std::string summary;
    switch (decision.state)
    {
    case SystemState::SYSTEM_STATE_READY:
        summary = "READY: commands enabled";
        break;
    case SystemState::SYSTEM_STATE_INITIALISING:
        summary =
            decision.blockingComponent.empty()
                ? common::console::formatText("INIT %zu/%zu: minimum window",
                                              decision.readyCount,
                                              decision.requiredCount)
                : common::console::formatText(
                      "INIT %zu/%zu: waiting %s",
                      decision.readyCount,
                      decision.requiredCount,
                      decision.blockingComponent.c_str());
        break;
    case SystemState::SYSTEM_STATE_HOLD:
        summary =
            common::console::formatText("HOLD %zu/%zu: %s",
                                        decision.readyCount,
                                        decision.requiredCount,
                                        decision.blockingComponent.c_str());
        break;
    }

    /* Log a state change, or a change of the component still pending, but
     * not every tick of an unchanged wait. */
    const bool hasStateChanged = decision.state != previousState;
    const bool hasPendingChanged =
        decision.blockingComponent != latestDecision.blockingComponent;
    if (hasStateChanged || lastPublication_s < 0.0 ||
        (decision.state != SystemState::SYSTEM_STATE_READY &&
         hasPendingChanged))
    {
        if (decision.state == SystemState::SYSTEM_STATE_HOLD)
        {
            SRS_LOG_WARN(get_logger(),
                           "%s (%s); commands blocked",
                           summary.c_str(),
                           decision.blockingReason.c_str());
        }
        else if (decision.state == SystemState::SYSTEM_STATE_READY ||
                 decision.blockingComponent.empty())
        {
            SRS_LOG_INFO(get_logger(), "%s", summary.c_str());
        }
        else
        {
            SRS_LOG_INFO(get_logger(),
                           "%s (%s)",
                           summary.c_str(),
                           decision.blockingReason.c_str());
        }
    }
    latestDecision = decision;

    /* Publish on change and at least once per heartbeat period. */
    if (!hasStateChanged && lastPublication_s >= 0.0 &&
        now_s - lastPublication_s < heartbeatPeriodS)
    {
        return;
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name    = "system_state";
    status.message = summary;
    switch (decision.state)
    {
    case SystemState::SYSTEM_STATE_READY:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
    case SystemState::SYSTEM_STATE_INITIALISING:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
    case SystemState::SYSTEM_STATE_HOLD:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
    }
    diagnostics::addTextValue("state", systemStateName(decision.state), status);
    diagnostics::addCountValue("ready_components", decision.readyCount, status);
    diagnostics::addCountValue("required_components",
                               decision.requiredCount,
                               status);
    diagnostics::addTextValue("blocking_component",
                              decision.blockingComponent,
                              status);
    diagnostics::addTextValue("blocking_reason",
                              decision.blockingReason,
                              status);
    diagnostics::addRealValue("since_start_s",
                              now_s - supervisorStart_s,
                              status);

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = now();
    record.status.push_back(status);
    p_systemStatePublisher->publish(record);
    lastPublication_s = now_s;
}

} /* namespace systems::alpha::alpha_supervisor */
