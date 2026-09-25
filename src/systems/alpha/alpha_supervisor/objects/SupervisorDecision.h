/*!
 * @file            SupervisorDecision.h
 *
 * @brief           Declares the result of one supervisor state evaluation.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SUPERVISOR_DECISION_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SUPERVISOR_DECISION_H

/* C++ Standard Library Includes */
#include <cstddef>
#include <string>

/* Object Includes */
#include "alpha_supervisor/objects/SystemState.h"

namespace systems::alpha::alpha_supervisor
{

/*!
 * @brief           Next system state plus the evidence behind it.
 */
struct SupervisorDecision
{
  public:
    /*!
     * @brief           State the supervisor transitions to.
     *
     * @frame           N/A
     * @units           N/A
     */
    SystemState state{SystemState::SYSTEM_STATE_INITIALISING};

    /*!
     * @brief           Number of required components that are ready and
     *                  fresh.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t readyCount{0U};

    /*!
     * @brief           Number of required components.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t requiredCount{0U};

    /*!
     * @brief           First required component (in configured order) that
     *                  is not ready or not fresh; empty when none is.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string blockingComponent;

    /*!
     * @brief           Why the blocking component holds the system: its
     *                  own reason, "no report" or "stale"; or "minimum
     *                  window" when only the start-up window is pending.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string blockingReason;
};

} /* namespace systems::alpha::alpha_supervisor */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_SUPERVISOR_DECISION_H \
        */
