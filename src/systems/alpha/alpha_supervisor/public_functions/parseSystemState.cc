/*!
 * @file            parseSystemState.cc
 *
 * @brief           Implements parsing of the published SystemState text.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "alpha_supervisor/public_functions/public_functions.h"

namespace systems::alpha::alpha_supervisor
{

std::optional<SystemState> parseSystemState(const std::string &stateName_in)
{
    /* Round-trip through systemStateName() so both stay in step. */
    for (const SystemState state : {SystemState::SYSTEM_STATE_INITIALISING,
                                    SystemState::SYSTEM_STATE_READY,
                                    SystemState::SYSTEM_STATE_HOLD})
    {
        if (systemStateName(state) == stateName_in)
        {
            return state;
        }
    }
    return std::nullopt;
}

} /* namespace systems::alpha::alpha_supervisor */
