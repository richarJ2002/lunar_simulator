/*!
 * @file            publishDiagnosticsCallBack.cc
 *
 * @brief           Implements the driver's readiness and gate diagnostics.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "diagnostics/diagnostics.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* The driver is ready while both raw sensor streams are fresh. */
    const double now_s      = now().seconds();
    const double imuAge_s   = now_s - latestRawImuReceipt_s;
    const double jointAge_s = now_s - latestRawJointStateReceipt_s;
    std::string  reason     = "inputs fresh";
    if (latestRawImuReceipt_s < 0.0)
    {
        reason = "no raw IMU";
    }
    else if (latestRawJointStateReceipt_s < 0.0)
    {
        reason = "no raw joint states";
    }
    else if (imuAge_s > readinessMaximumInputAgeS)
    {
        reason = common::console::formatText("raw IMU %.1f s old", imuAge_s);
    }
    else if (jointAge_s > readinessMaximumInputAgeS)
    {
        reason =
            common::console::formatText("raw joints %.1f s old", jointAge_s);
    }
    const bool isReady = reason == "inputs fresh";

    const CommandGateDecision gate =
        evaluateCommandGate(latestSystemState,
                            now_s - latestSystemStateReceipt_s,
                            maximumStateHeartbeatAgeS);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name        = "alpha_driver_node";
    status.hardware_id = get_name();
    status.level       = isReady ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
        gate.isOpen ? "CMD gate open" : "CMD blocked: " + gate.reason;
    diagnostics::addFlagValue("ready", isReady, status);
    diagnostics::addTextValue("reason", reason, status);
    diagnostics::addFlagValue("gate_open", gate.isOpen, status);
    diagnostics::addTextValue("gate_reason", gate.reason, status);
    diagnostics::addCountValue("commands_forwarded",
                               forwardedCommandCount,
                               status);
    diagnostics::addCountValue("commands_blocked", blockedCommandCount, status);
    diagnostics::addCountValue("stop_commands", stopCommandCount, status);

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = now();
    record.status.push_back(status);
    p_diagnosticsPublisher->publish(record);
}

} /* namespace systems::alpha::alpha_drivers */
