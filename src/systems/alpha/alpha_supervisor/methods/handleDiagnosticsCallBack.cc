/*!
 * @file            handleDiagnosticsCallBack.cc
 *
 * @brief           Implements recording of component readiness reports.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "alpha_supervisor/objects/AlphaStartupSupervisorNodeClass.h"

namespace systems::alpha::alpha_supervisor
{

void AlphaStartupSupervisorNode::handleDiagnosticsCallBack(
    const diagnostic_msgs::msg::DiagnosticArray &message_in)
{
    /* Receipt time, not the publisher's stamp, measures freshness, so a
     * component with a wrong stamp cannot look fresh forever. */
    const double receipt_s = now().seconds();
    for (const diagnostic_msgs::msg::DiagnosticStatus &status :
         message_in.status)
    {
        for (ComponentReadiness &component : components)
        {
            if (component.name != status.name)
            {
                continue;
            }

            /* Only a status that states readiness is a readiness report. */
            bool        hasReadyValue = false;
            bool        isReady       = false;
            std::string reason;
            for (const diagnostic_msgs::msg::KeyValue &entry : status.values)
            {
                if (entry.key == "ready")
                {
                    hasReadyValue = true;
                    isReady       = entry.value == "true";
                }
                else if (entry.key == "reason")
                {
                    reason = entry.value;
                }
            }
            if (!hasReadyValue)
            {
                continue;
            }
            component.hasReport    = true;
            component.isReady      = isReady;
            component.reason       = reason;
            component.reportTime_s = receipt_s;
        }
    }
}

} /* namespace systems::alpha::alpha_supervisor */
