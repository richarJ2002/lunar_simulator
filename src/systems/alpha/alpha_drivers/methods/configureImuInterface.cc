/*!
 * @file            configureImuInterface.cc
 *
 * @brief           Configures the raw and public IMU topics.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::configureImuInterface(const std::string &systemName_in)
{
    /* Input topic: raw, noise-free IMU bridged straight from Gazebo. */
    const std::string rawImuTopic = declare_parameter<std::string>(
        "raw_imu_topic", "/" + systemName_in + "/raw/imu");

    /* Output topic: the noisy IMU every consumer actually subscribes to. */
    const std::string imuTopic = declare_parameter<std::string>(
        "imu_topic", "/" + systemName_in + "/imu");

    /* Advertise the noisy public IMU topic. */
    p_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(
        imuTopic, rclcpp::SensorDataQoS());

    /*!
     * Every interface below shares one mutually-exclusive callback group so
     * noise sampling from the shared random engine is never re-entered
     * concurrently, while still allowing this node's streams to run on
     * separate executor threads relative to other nodes' callback groups.
     */
    rclcpp::SubscriptionOptions subscriptionOptions;

    /* Assign that shared callback group to this subscription. */
    subscriptionOptions.callback_group = p_noiseCallbackGroup;

    /* Every raw IMU sample triggers publishNoisyImuCallBack() on that group. */
    p_rawImuSubscription = create_subscription<sensor_msgs::msg::Imu>(
        rawImuTopic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr p_message)
        { publishNoisyImuCallBack(*p_message); }, subscriptionOptions);
}

} /* namespace systems::alpha::alpha_drivers */
