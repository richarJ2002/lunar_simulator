/*!
 * @file            configureWheelCommandInterface.cc
 *
 * @brief           Configures the public wheel-command subscription and the
 *                   raw actuator bridge publisher.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::configureWheelCommandInterface(
    const std::string &systemName_in)
{
    /* Public input topic: low-level six-wheel joint command. */
    const std::string wheelCommandTopic = declare_parameter<std::string>(
        "wheel_joint_states_topic",
        "/" + systemName_in + "/control/cmd/wheel_joint_states");

    /* Output topic: the raw Gazebo actuator bridge, matching this system's
     * config/alpha_ros_gz_bridge.yaml and alpha_model/model.sdf. */
    const std::string rawWheelCommandTopic = declare_parameter<std::string>(
        "raw_wheel_joint_states_topic",
        "/" + systemName_in + "/drivers/cmd/wheel_joint_states");

    /* Publisher forwarding onto the raw actuator bridge topic. */
    p_rawWheelCommandPublisher =
        create_publisher<actuator_msgs::msg::Actuators>(rawWheelCommandTopic,
                                                        rclcpp::QoS(10));

    /*!
     * Shares the same serial callback group as the other interfaces (see
     * configureImuInterface.cc), since publishNoisyWheelCommandCallBack() also
     * draws from the shared random engine.
     */
    rclcpp::SubscriptionOptions subscriptionOptions;

    /* Assign that shared callback group to this subscription. */
    subscriptionOptions.callback_group = p_noiseCallbackGroup;

    /* Every incoming public command triggers
     * publishNoisyWheelCommandCallBack(). */
    p_wheelCommandSubscription =
        create_subscription<actuator_msgs::msg::Actuators>(
            wheelCommandTopic,
            rclcpp::QoS(10),
            [this](actuator_msgs::msg::Actuators::ConstSharedPtr p_message)
            { publishNoisyWheelCommandCallBack(*p_message); },
            subscriptionOptions);

    /* Record the resolved topic names once at start-up for operators
     * inspecting the node's log. */
    RCLCPP_INFO(get_logger(),
                "Alpha driver: wheel command %s -> %s",
                wheelCommandTopic.c_str(),
                rawWheelCommandTopic.c_str());
}

} /* namespace systems::alpha::alpha_drivers */
