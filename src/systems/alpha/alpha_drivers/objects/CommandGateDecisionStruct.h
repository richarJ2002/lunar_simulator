/*!
 * @file            CommandGateDecisionStruct.h
 *
 * @brief           Declares the outcome of one command-gate evaluation.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVERS_COMMAND_GATE_DECISION_STRUCT_H
#define LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVERS_COMMAND_GATE_DECISION_STRUCT_H

/* C++ Standard Library Includes */
#include <string>

namespace systems::alpha::alpha_drivers
{

/*!
 * @brief           Whether wheel commands may reach the actuators now, and
 *                  why not when they may not.
 */
struct CommandGateDecision
{
  public:
    /*!
     * @brief           True when commands are forwarded to the actuators.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isOpen{false};

    /*!
     * @brief           Short reason the gate is closed (at most 40
     *                  characters with its "CMD blocked: " prefix); empty
     *                  when open.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string reason;
};

} /* namespace systems::alpha::alpha_drivers */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVERS_COMMAND_GATE_DECISION_STRUCT_H */
