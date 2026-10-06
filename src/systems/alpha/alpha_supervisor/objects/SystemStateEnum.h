/*!
 * @file            SystemStateEnum.h
 *
 * @brief           Declares Alpha's start-up and command-readiness states.
 *
 * @date            24/09/2026
 */

#ifndef SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SYSTEM_STATE_ENUM_H
#define SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SYSTEM_STATE_ENUM_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace systems::alpha::alpha_supervisor
{

/*!
 * @brief           Whole-system readiness published by the start-up
 *                  supervisor and enforced by the driver's command gate.
 *
 *                  The text form of each state (systemStateName()) is the
 *                  published "state" value on the system state topic.
 */
enum class SystemState : std::uint8_t
{
    /*! Start-up window: at least one required component is not ready yet,
     *  or the minimum initialisation window has not elapsed. Commands are
     *  blocked. */
    SYSTEM_STATE_INITIALISING = 0U,

    /*! Every required component reports ready with a fresh status.
     *  Commands are forwarded. */
    SYSTEM_STATE_READY = 1U,

    /*! A required component regressed or went stale after READY. Commands
     *  are blocked until every component is ready again. */
    SYSTEM_STATE_HOLD = 2U
};

} /* namespace systems::alpha::alpha_supervisor */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SYSTEM_STATE_ENUM_H */
