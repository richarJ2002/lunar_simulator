/*!
 * @file            AlphaStartupSupervisorNodeClass.h
 *
 * @brief           Declares the node that decides when Alpha may accept
 *                  motion commands.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_ALPHA_STARTUP_SUPERVISOR_NODE_CLASS_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_ALPHA_STARTUP_SUPERVISOR_NODE_CLASS_H

/* C++ Standard Library Includes */
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

/* Other Project Module Includes */
#include "alpha_supervisor/public_functions/public_functions.h"
#include "console/console.h"

/* Object Includes */
#include "alpha_supervisor/objects/ComponentReadinessStruct.h"
#include "alpha_supervisor/objects/SupervisorDecisionStruct.h"
#include "alpha_supervisor/objects/SystemStateEnum.h"

namespace systems::alpha::alpha_supervisor
{

/*!
 * @brief           Aggregates every required component's self-reported
 *                  readiness into one system state and publishes it for
 *                  the driver's command gate.
 *
 * Each component publishes a status on the shared diagnostics topic with
 * a "ready" flag and a short "reason"; this node owns only the
 * aggregation (evaluateSystemState()), never a component's own criteria.
 * The state is published latched (reliable, transient-local) on the
 * system state topic whenever it changes and at least once per
 * heartbeat period, so the gate can also treat a silent supervisor as not
 * ready. All callbacks run in the node's mutually exclusive default
 * callback group, which is the only synchronization the members need.
 */
class AlphaStartupSupervisorNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters and wires the diagnostics
     *                  subscription, state publisher and evaluation timer.
     *
     * @throws          std::invalid_argument if no component is required or
     *                  a time limit is not positive.
     */
    AlphaStartupSupervisorNode() :
        Node("startup_supervisor")
    {
        /* Declared first so the topic defaults below can be rooted at the
         * owning system's namespace. */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: every component's self-reported status. */
        const std::string diagnosticsTopic =
            declare_parameter<std::string>("diagnostics_topic",
                                           "/" + systemName + "/diagnostics");

        /* Output topic: the latched system state the gate enforces. */
        const std::string systemStateTopic =
            declare_parameter<std::string>("system_state_topic",
                                           "/" + systemName + "/system/state");

        /* Components whose readiness gates commands, in the order the
         * terminal names the first one still pending. */
        const std::vector<std::string> requiredComponents =
            declare_parameter<std::vector<std::string>>("required_components",
                                                        {"alpha_driver_node",
                                                         "inertial_odometry",
                                                         "visual_odometry",
                                                         "wheel_odometry",
                                                         "continuous_ekf"});

        /* Shortest start-up window before READY, even if every component
         * is ready earlier. */
        minimumInitWindowS =
            declare_parameter<double>("minimum_init_window_s", 10.0);

        /* Oldest component report that still counts. */
        componentTimeoutS =
            declare_parameter<double>("component_timeout_s", 2.0);

        /* Longest interval between two state publications. */
        heartbeatPeriodS = declare_parameter<double>("heartbeat_period_s", 1.0);

        if (requiredComponents.empty() || !(minimumInitWindowS >= 0.0) ||
            !(componentTimeoutS > 0.0) || !(heartbeatPeriodS > 0.0))
        {
            throw std::invalid_argument(
                "supervisor needs components and positive time limits");
        }
        for (const std::string &componentName : requiredComponents)
        {
            ComponentReadiness component;
            component.name = componentName;
            components.push_back(component);
        }

        /* Latched so a late subscriber (the gate after a restart, a
         * recorder, an operator's echo) immediately sees the state. */
        p_systemStatePublisher =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                systemStateTopic,
                rclcpp::QoS(1).reliable().transient_local());

        p_diagnosticsSubscription =
            create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                diagnosticsTopic,
                rclcpp::QoS(50),
                [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr
                           p_message_in)
                { handleDiagnosticsCallBack(*p_message_in); });

        /* Evaluate four times per simulated second so a regression is
         * published well inside the gate's heartbeat limit. */
        p_evaluationTimer =
            create_timer(std::chrono::milliseconds(250),
                         [this]() { publishSystemStateCallBack(); });

        LUNAR_LOG_INFO(get_logger(),
                       "INIT 0/%zu: commands blocked",
                       components.size());
    }

    /*!
     * @brief           Releases the node's ROS interfaces.
     */
    ~AlphaStartupSupervisorNode() override = default;

    AlphaStartupSupervisorNode(const AlphaStartupSupervisorNode &otherNode_in) =
        delete;
    AlphaStartupSupervisorNode &
        operator=(const AlphaStartupSupervisorNode &otherNode_in) = delete;
    AlphaStartupSupervisorNode(AlphaStartupSupervisorNode &&otherNode_in) =
        delete;
    AlphaStartupSupervisorNode &
        operator=(AlphaStartupSupervisorNode &&otherNode_in) = delete;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Records the readiness of every required component
     *                  found in one diagnostics array.
     *
     * @param[in]       message_in
     *                  Array published by any node on the diagnostics
     *                  topic; statuses of other names, and statuses without
     *                  a "ready" value, are ignored.
     */
    void handleDiagnosticsCallBack(
        const diagnostic_msgs::msg::DiagnosticArray &message_in);

    /*!
     * @brief           Evaluates the system state and publishes it when it
     *                  changes or a heartbeat is due.
     *
     *                  Runs from a simulation-time timer every 250 ms. The
     *                  start-up window begins at the first call with a
     *                  non-zero clock.
     */
    void publishSystemStateCallBack();

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Latest report of each required component, in the
     *                  configured order.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::vector<ComponentReadiness> components;

    /*!
     * @brief           Shortest start-up window before READY.
     *
     * @frame           N/A
     * @units           seconds
     */
    double minimumInitWindowS{10.0};

    /*!
     * @brief           Oldest component report that still counts.
     *
     * @frame           N/A
     * @units           seconds
     */
    double componentTimeoutS{2.0};

    /*!
     * @brief           Longest interval between state publications.
     *
     * @frame           N/A
     * @units           seconds
     */
    double heartbeatPeriodS{1.0};

    /*!
     * @brief           Current system state.
     *
     * @frame           N/A
     * @units           N/A
     */
    SystemState systemState{SystemState::SYSTEM_STATE_INITIALISING};

    /*!
     * @brief           Evidence behind the latest evaluation, kept to log
     *                  only when the pending component or reason changes.
     *
     * @frame           N/A
     * @units           N/A
     */
    SupervisorDecision latestDecision;

    /*!
     * @brief           Whether the start-up window has begun (first
     *                  evaluation with a non-zero clock).
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasStarted{false};

    /*!
     * @brief           Time at which the start-up window began.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double supervisorStart_s{0.0};

    /*!
     * @brief           Time of the latest state publication; negative
     *                  before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double lastPublication_s{-1.0};

    /*!
     * @brief           Publishes the latched system state.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_systemStatePublisher;

    /*!
     * @brief           Receives every component's status.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsSubscription;

    /*!
     * @brief           Simulation-time timer driving
     *                  publishSystemStateCallBack().
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_evaluationTimer;
};

} /* namespace systems::alpha::alpha_supervisor */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_SUPERVISOR_ALPHA_STARTUP_SUPERVISOR_NODE_CLASS_H \
        */
