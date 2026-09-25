/*!
 * @file            EstimateLowPassFilterNode.h
 *
 * @brief           Declares the node that publishes a low-pass-filtered
 *                  copy of the fused estimate for controllers.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_NODE_H
#define LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_NODE_H

/* C++ Standard Library Includes */
#include <stdexcept>
#include <string>

/* External Library Includes */
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "estimate_filter/public_functions.h"

/* Object Includes */
#include "estimate_filter/objects/EstimateLowPassFilter.h"
#include "estimate_filter/objects/LowPassConfiguration.h"

namespace control::estimate_filter
{

/*!
 * @brief           Republishes the fused estimate through
 *                  EstimateLowPassFilter for controllers.
 *
 * Input and output are nav_msgs/Odometry with the same stamp and frames;
 * the pose and twist are filtered, while the covariance is passed through
 * unchanged and so describes the unfiltered estimate. The output is never
 * fed back into the estimator. Callbacks run in the node's mutually
 * exclusive default callback group, which is the only synchronization the
 * filter needs.
 */
class EstimateLowPassFilterNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters and wires the estimate input and
     *                  filtered output.
     *
     * @throws          std::invalid_argument if a cutoff or the gap limit is
     *                  not a finite positive number.
     */
    EstimateLowPassFilterNode() :
        Node("estimate_low_pass_filter"),
        filter(declareConfiguration())
    {
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: the fused estimate. */
        const std::string inputTopic = declare_parameter<std::string>(
            "input_odometry_topic",
            "/" + systemName + "/localisation/kalman_filter/odometry");

        /* Output topic: the filtered estimate for controllers. */
        const std::string outputTopic = declare_parameter<std::string>(
            "output_odometry_topic",
            "/" + systemName + "/control/filtered_odometry");

        p_outputPublisher =
            create_publisher<nav_msgs::msg::Odometry>(outputTopic,
                                                      rclcpp::QoS(10));
        p_inputSubscription = create_subscription<nav_msgs::msg::Odometry>(
            inputTopic,
            rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleOdometryCallBack(*p_message); });

        LUNAR_LOG_DEBUG(get_logger(),
                        "Estimate LPF: %s -> %s",
                        inputTopic.c_str(),
                        outputTopic.c_str());
    }

    /*!
     * @brief           Releases the node's ROS interfaces.
     */
    ~EstimateLowPassFilterNode() override = default;

    EstimateLowPassFilterNode(const EstimateLowPassFilterNode &otherNode_in) =
        delete;
    EstimateLowPassFilterNode &
        operator=(const EstimateLowPassFilterNode &otherNode_in) = delete;
    EstimateLowPassFilterNode(EstimateLowPassFilterNode &&otherNode_in) =
        delete;
    EstimateLowPassFilterNode &
        operator=(EstimateLowPassFilterNode &&otherNode_in) = delete;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Filters one fused estimate and publishes the result
     *                  unless the sample was skipped.
     *
     * @param[in]       message_in
     *                  Fused estimate; a change of its frame ids resets the
     *                  filter.
     */
    void handleOdometryCallBack(const nav_msgs::msg::Odometry &message_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Declares and validates the filter tuning parameters.
     *
     * @return          The validated configuration.
     *
     * @throws          std::invalid_argument if a value is not a finite
     *                  positive number.
     */
    LowPassConfiguration declareConfiguration();

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           The owned low-pass filter.
     *
     * @frame           Mixed; see EstimateSample
     * @units           Mixed; see EstimateSample
     */
    EstimateLowPassFilter filter;

    /*!
     * @brief           Parent frame of the previous input, to detect a
     *                  frame change.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string previousFrameId;

    /*!
     * @brief           Child frame of the previous input, to detect a frame
     *                  change.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string previousChildFrameId;

    /*!
     * @brief           Publishes the filtered estimate.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_outputPublisher;

    /*!
     * @brief           Receives the fused estimate.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_inputSubscription;
};

} /* namespace control::estimate_filter */

#endif /* LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_NODE_H \
        */
