/*!
 * @file            publishDiagnosticsCallBack.cc
 *
 * @brief           Implements the wheel odometry readiness report.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/WheelOdometryNode.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

namespace localisation::wheel_odometry
{

void WheelOdometryNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Readiness contract read by the start-up supervisor. */
    const double now_s  = now().seconds();
    std::string  reason = "publishing";
    if (!hasAllJoints)
    {
        reason = "waiting for 12 joints";
    }
    else if (latestPublishTime_s < 0.0 ||
             now_s - latestPublishTime_s > readinessMaximumGapS)
    {
        reason = common::console::formatText(
            "no odometry for %.1f s",
            latestPublishTime_s < 0.0 ? 0.0 : now_s - latestPublishTime_s);
    }
    else if (latestPublishTime_s - continuousPublishStart_s <
             readinessContinuousPeriodS)
    {
        reason = common::console::formatText("continuous %.1f/%.1f s",
                                             latestPublishTime_s -
                                                 continuousPublishStart_s,
                                             readinessContinuousPeriodS);
    }
    const bool isReady = reason == "publishing";

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name        = "wheel_odometry";
    status.hardware_id = baseFrame;
    status.level       = isReady ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message     = reason;
    diagnostics::addFlagValue("ready", isReady, status);
    diagnostics::addTextValue("reason", reason, status);

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = now();
    record.status.push_back(status);
    p_diagnosticsPublisher->publish(record);
}

} /* namespace localisation::wheel_odometry */
