/*!
 * @file            systemStateName.cc
 *
 * @brief           Implements the published text form of SystemState.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "alpha_supervisor/public_functions/public_functions.h"

namespace systems::alpha::alpha_supervisor
{

std::string systemStateName(SystemState state_in)
{
    switch (state_in)
    {
    case SystemState::SYSTEM_STATE_READY:
        return "READY";
    case SystemState::SYSTEM_STATE_HOLD:
        return "HOLD";
    case SystemState::SYSTEM_STATE_INITIALISING:
        break;
    }
    return "INITIALISING";
}

} /* namespace systems::alpha::alpha_supervisor */
