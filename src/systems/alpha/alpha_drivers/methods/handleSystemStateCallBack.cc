/*!
 * @file            handleSystemStateCallBack.cc
 *
 * @brief           Implements receipt of the supervisor's system state.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::handleSystemStateCallBack(
    const diagnostic_msgs::msg::DiagnosticArray &message_in)
{
    for (const diagnostic_msgs::msg::DiagnosticStatus &status :
         message_in.status)
    {
        if (status.name != "system_state")
        {
            continue;
        }
        for (const diagnostic_msgs::msg::KeyValue &entry : status.values)
        {
            if (entry.key == "state")
            {
                /* Unknown text parses to no value, which keeps the gate
                 * closed. Receipt time, not the stamp, is the heartbeat. */
                latestSystemState =
                    alpha_supervisor::parseSystemState(entry.value);
                latestSystemStateReceipt_s = now().seconds();
            }
        }
    }
}

} /* namespace systems::alpha::alpha_drivers */
