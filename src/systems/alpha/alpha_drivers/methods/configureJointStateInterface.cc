/*!
 * @file            configureJointStateInterface.cc
 *
 * @brief           Configures the raw and public joint-state topics.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::configureJointStateInterface(
    const std::string &systemName_in)
{
    /* Input topic: raw, noise-free joint states from Gazebo. */
    const std::string rawJointStateTopic = declare_parameter<std::string>(
        "raw_joint_state_topic",
        "/" + systemName_in + "/drivers/joint_states");

    /* Output topic: the noisy joint states every consumer subscribes to. */
    const std::string jointStateTopic =
        declare_parameter<std::string>("joint_state_topic",
                                       "/" + systemName_in + "/joint_states");

    /* Advertise the noisy public joint-state topic. */
    p_jointStatePublisher =
        create_publisher<sensor_msgs::msg::JointState>(jointStateTopic,
                                                       rclcpp::SensorDataQoS());

    /*!
     * Shares the same serial callback group as the other interfaces; see
     * configureImuInterface.cc for why.
     */
    rclcpp::SubscriptionOptions subscriptionOptions;

    /* Assign that shared callback group to this subscription. */
    subscriptionOptions.callback_group = p_noiseCallbackGroup;

    /* Every raw joint-state sample triggers publishNoisyJointStateCallBack().
     */
    p_rawJointStateSubscription =
        create_subscription<sensor_msgs::msg::JointState>(
            rawJointStateTopic,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr p_message)
            { publishNoisyJointStateCallBack(*p_message); },
            subscriptionOptions);
}

} /* namespace systems::alpha::alpha_drivers */
