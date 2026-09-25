/*!
 * @file            ComponentReadiness.h
 *
 * @brief           Declares the supervisor's latest view of one required
 *                  component's self-reported readiness.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_COMPONENT_READINESS_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_COMPONENT_READINESS_H

/* C++ Standard Library Includes */
#include <string>

namespace systems::alpha::alpha_supervisor
{

/*!
 * @brief           Latest readiness report received from one component on
 *                  the diagnostics topic.
 */
struct ComponentReadiness
{
  public:
    /*!
     * @brief           Diagnostic status name of the component, e.g.
     *                  "continuous_ekf".
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string name;

    /*!
     * @brief           Whether any status has been received from the
     *                  component.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasReport{false};

    /*!
     * @brief           The component's latest "ready" value.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isReady{false};

    /*!
     * @brief           The component's latest "reason" text (at most 40
     *                  characters by contract).
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string reason;

    /*!
     * @brief           Supervisor time at which the latest status arrived.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double reportTime_s{0.0};
};

} /* namespace systems::alpha::alpha_supervisor */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_COMPONENT_READINESS_H \
        */
