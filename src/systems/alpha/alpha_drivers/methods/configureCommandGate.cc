/*!
 * @file            configureCommandGate.cc
 *
 * @brief           Configures the command gate's system-state input,
 *                  readiness diagnostics and timers.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

/* C++ Standard Library Includes */
#include <stdexcept>

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::configureCommandGate(const std::string &systemName_in)
{
    /* Input topic: the start-up supervisor's latched system state. */
    const std::string systemStateTopic =
        declare_parameter<std::string>("system_state_topic",
                                       "/" + systemName_in + "/system/state");

    /* Output topic: this node's readiness and gate counters. */
    const std::string diagnosticsTopic =
        declare_parameter<std::string>("diagnostics_topic",
                                       "/" + systemName_in + "/diagnostics");

    /* A READY state older than this no longer opens the gate, so a silent
     * supervisor stops the rover rather than leaving it free to drive. */
    maximumStateHeartbeatAgeS =
        declare_parameter<double>("maximum_state_heartbeat_age_s", 1.5);

    /* Raw sensor inputs older than this make the driver report itself not
     * ready. */
    readinessMaximumInputAgeS =
        declare_parameter<double>("readiness_maximum_input_age_s", 0.5);
    if (!(maximumStateHeartbeatAgeS > 0.0) ||
        !(readinessMaximumInputAgeS > 0.0))
    {
        throw std::invalid_argument("gate and readiness ages must be positive");
    }

    /* Every gate interface shares the serial group of the command callback,
     * so the gate state is never read and written concurrently. */
    rclcpp::SubscriptionOptions subscriptionOptions;
    subscriptionOptions.callback_group = p_noiseCallbackGroup;
    p_systemStateSubscription =
        create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            systemStateTopic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](
                diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr p_message)
            { handleSystemStateCallBack(*p_message); },
            subscriptionOptions);

    p_diagnosticsPublisher =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnosticsTopic,
            rclcpp::QoS(10));

    /* Simulation-time timers: the gate check runs ten times per simulated
     * second so a lost heartbeat stops the rover promptly. */
    p_gateTimer = create_timer(
        std::chrono::milliseconds(100),
        [this]() { enforceCommandGateCallBack(); },
        p_noiseCallbackGroup);
    p_diagnosticsTimer = create_timer(
        std::chrono::seconds(1),
        [this]() { publishDiagnosticsCallBack(); },
        p_noiseCallbackGroup);
}

} /* namespace systems::alpha::alpha_drivers */
