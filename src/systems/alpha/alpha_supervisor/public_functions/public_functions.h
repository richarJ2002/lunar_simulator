/*!
 * @file            public_functions.h
 *
 * @brief           Declares the supervisor's pure state evaluation and the
 *                  text form of SystemState shared with the command gate.
 *
 * @date            24/09/2026
 */

#ifndef SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_PUBLIC_FUNCTIONS_H
#define SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_PUBLIC_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <optional>
#include <string>
#include <vector>

/* Object Includes */
#include "alpha_supervisor/objects/ComponentReadinessStruct.h"
#include "alpha_supervisor/objects/SupervisorDecisionStruct.h"
#include "alpha_supervisor/objects/SystemStateEnum.h"

namespace systems::alpha::alpha_supervisor
{

/*!
 * @brief           Evaluates the next system state from every required
 *                  component's latest readiness report.
 *
 *                  A component counts as ready only when it has reported,
 *                  its latest report says ready, and that report is no
 *                  older than componentTimeout_s. From INITIALISING the
 *                  system becomes READY only when every component counts as
 *                  ready and at least minimumInitWindow_s has passed since
 *                  the supervisor started. From READY any component that
 *                  stops counting as ready moves the system to HOLD. From
 *                  HOLD the system returns to READY once every component
 *                  counts as ready again; the start-up window does not
 *                  apply twice. The function is pure and deterministic.
 *
 * @param[in]       previousState_in
 *                  State before this evaluation.
 *
 * @param[in]       components_in
 *                  Latest report of every required component, in the
 *                  configured order; the first failing one is reported
 *                  as blocking.
 *
 * @param[in]       now_s
 *                  Current supervisor time, ROS seconds.
 *
 * @param[in]       supervisorStart_s
 *                  Supervisor time at which the start-up window began, ROS
 *                  seconds.
 *
 * @param[in]       minimumInitWindow_s
 *                  Minimum time from supervisorStart_s to READY, seconds.
 *
 * @param[in]       componentTimeout_s
 *                  Largest age of a report that still counts, seconds.
 *
 * @return          The next state and the evidence behind it.
 */
SupervisorDecision
    evaluateSystemState(SystemState                            previousState_in,
                        const std::vector<ComponentReadiness> &components_in,
                        double                                 now_s,
                        double supervisorStart_s,
                        double minimumInitWindow_s,
                        double componentTimeout_s);

/*!
 * @brief           Returns the published text form of a state.
 *
 * @param[in]       state_in
 *                  State to name.
 *
 * @return          "INITIALISING", "READY" or "HOLD".
 */
std::string systemStateName(SystemState state_in);

/*!
 * @brief           Parses the published text form of a state.
 *
 * @param[in]       stateName_in
 *                  Text received on the system state topic.
 *
 * @return          The state, or no value for unrecognised text (which the
 *                  command gate treats as not ready).
 */
std::optional<SystemState> parseSystemState(const std::string &stateName_in);

} /* namespace systems::alpha::alpha_supervisor */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_PUBLIC_FUNCTIONS_H */
