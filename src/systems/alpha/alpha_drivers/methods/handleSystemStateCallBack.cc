/*!
 * @file            handleSystemStateCallBack.cc
 *
 * @brief           Implements receipt of the supervisor's system state.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNodeClass.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::handleSystemStateCallBack(
    const diagnostic_msgs::msg::DiagnosticArray &message_in)
{
    /* Scan every status entry for the system-state report. */
    for (const diagnostic_msgs::msg::DiagnosticStatus &status :
         message_in.status)
    {
        /* Ignore entries that are not the system-state report. */
        if (status.name != "system_state")
        {
            continue;
        }

        /* Scan the report pairs for the state field. */
        for (const diagnostic_msgs::msg::KeyValue &entry : status.values)
        {
            /* Only the state field updates the latched state. */
            if (entry.key == "state")
            {
                /*!
                 * Unknown text parses to no value, which keeps the gate
                 * closed. Receipt time, not the stamp, is the heartbeat.
                 */
                latestSystemState =
                    alpha_supervisor::parseSystemState(entry.value);

                /* Record receipt time as the gate heartbeat. */
                latestSystemStateReceipt_s = now().seconds();
            }
        }
    }
}

} /* namespace systems::alpha::alpha_drivers */
