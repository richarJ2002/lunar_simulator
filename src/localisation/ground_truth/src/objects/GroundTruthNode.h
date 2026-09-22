/*!
 * @File:         GroundTruthNode.h
 *
 * @Brief:        Declares the Alpha ground-truth path/TF publishing node.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_GROUND_TRUTH_NODE_H
#define LUNAR_SIMULATOR_LOCALISATION_GROUND_TRUTH_NODE_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

/* Generic Libraries */
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace localisation::ground_truth
{

/*!
 * @brief           Converts simulator truth odometry into comparison outputs.
 *
 * The first valid input pose defines `alpha/startup_fixed`. The node publishes
 * that full initial position and attitude as a static transform from `map`,
 * then expresses all comparison odometry below the fixed frame. No estimator
 * consumes this private truth-derived transform. It is intended for a
 * single-threaded executor.
 */
class GroundTruthNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Configures ground-truth path and TF publication.
     * @throws          std::invalid_argument if a path limit is invalid.
     */
    GroundTruthNode() :
        Node("ground_truth")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: the raw ground-truth odometry bridged from Gazebo. */
        const std::string odometryTopic = declare_parameter<std::string>(
            "odometry_topic",
            "/" + systemName + "/drivers/ground_truth/odometry");

        /* Output topic: validated ground-truth odometry for ROS consumers. */
        const std::string outputOdometryTopic = declare_parameter<std::string>(
            "output_odometry_topic",
            "/" + systemName + "/localisation/ground_truth/odometry");

        /* Output topic: the retained, rate-limited comparison path. */
        const std::string pathTopic = declare_parameter<std::string>(
            "path_topic",
            "/" + systemName + "/localisation/ground_truth/path");

        /* Fixed world frame shared with Gazebo and every other node. */
        mapFrame = declare_parameter<std::string>("map_frame", "map");

        /* Local comparison frame placed at the rover's first valid pose. */
        startupFixedFrame = declare_parameter<std::string>(
            "startup_fixed_frame", systemName + "/startup_fixed");

        /* This node's own child frame, distinct from the estimator's. */
        groundTruthBaseFrame =
            declare_parameter<std::string>("ground_truth_base_frame",
                                           "alpha/ground_truth_base_link");

        /* Read the raw path-length limit before validating it below. */
        const int configuredMaximumPoses =
            declare_parameter<int>("path_maximum_poses", 5000);

        /* Read the raw sample period before validating it below. */
        const double pathSamplePeriodS =
            declare_parameter<double>("path_sample_period_s", 0.1);

        /* Reject a configuration that could never retain or sample a
         * path. */
        if (configuredMaximumPoses <= 0 || pathSamplePeriodS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "Ground-truth path limits must be greater than zero");
        }

        /* Narrow the validated parameter to the member's storage type. */
        pathMaximumPoses = static_cast<std::size_t>(configuredMaximumPoses);

        /* Convert the configured period from seconds to nanoseconds so it
         * can be compared directly against message timestamps. */
        pathSamplePeriodNs = static_cast<std::int64_t>(pathSamplePeriodS *
                                                       NANOSECONDS_PER_SECOND);

        /* Own the TF broadcaster used by publishTransform(). */
        p_transformBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        /* Own the latched broadcaster for map -> startup-fixed. */
        p_staticTransformBroadcaster =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        /* Latch the path so a newly opened RViz view sees the current
         * history immediately, rather than waiting for the next sample. */
        p_pathPublisher = create_publisher<nav_msgs::msg::Path>(
            pathTopic,
            rclcpp::QoS(1).reliable().transient_local());

        /* Relay validated simulator truth outside the private driver boundary.
         */
        p_odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputOdometryTopic,
            rclcpp::QoS(10).reliable());

        /* Every incoming odometry message triggers handleOdometryCallBack(). */
        p_odometrySubscription = create_subscription<nav_msgs::msg::Odometry>(
            odometryTopic,
            rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleOdometryCallBack(*p_message); });

        /* Record the resolved topic names once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(),
                    "Ground truth: %s -> %s, %s in %s",
                     odometryTopic.c_str(),
                     outputOdometryTopic.c_str(),
                     pathTopic.c_str(),
                    startupFixedFrame.c_str());
    }

    /*!
     * @brief           Rebases one map-frame body transform into fixed.
     *
     * @param[in]       mapFromFixed_in
     *                  Full startup-fixed pose in map.
     * @param[in]       mapFromBody_in
     *                  Body pose in map at the sample epoch.
     *
     * @return          Body pose in startup-fixed, equal to
     *                  inverse(mapFromFixed_in) * mapFromBody_in.
     */
    static tf2::Transform calculateFixedFromBody(
        const tf2::Transform &mapFromFixed_in,
        const tf2::Transform &mapFromBody_in);

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Publishes one truth transform and sampled path point.
     * @param[in]       odometry_in Gazebo truth odometry in the map frame.
     */
    void handleOdometryCallBack(const nav_msgs::msg::Odometry &odometry_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Converts a ROS timestamp to nanoseconds.
     * @param[in]       stamp_in Timestamp to convert.
     * @return          Timestamp in nanoseconds.
     */
    static std::int64_t
        stampToNanoseconds(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Tests whether a pose can be published safely.
     * @param[in]       pose_in Pose expressed in the map frame.
     * @return          True when position and orientation are finite and the
     *                  quaternion has a nonzero norm.
     */
    static bool isPoseValid(const geometry_msgs::msg::Pose &pose_in);

    /*!
     * @brief           Adds a time-sampled truth pose to the bounded path.
     * @param[in]       odometry_in Valid truth odometry in the map frame.
     */
    void appendPathPose(const nav_msgs::msg::Odometry &odometry_in);

    /*!
     * @brief           Publishes map to ground-truth body transform.
     * @param[in]       odometry_in Valid truth odometry in the map frame.
     */
    void publishTransform(const nav_msgs::msg::Odometry &odometry_in);

    /*!
     * @brief           Publishes the latched map-to-startup-fixed transform.
     *
     * @param[in]       stamp_in
     *                  Timestamp of the first valid truth sample.
     */
    void publishStartupFixedTransform(
        const builtin_interfaces::msg::Time &stamp_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Nanoseconds in one second.
     */
    static constexpr double NANOSECONDS_PER_SECOND = 1.0e9;

    /*!
     * @brief       Receives simulator truth odometry.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_odometrySubscription;

    /*!
     * @brief       Publishes validated simulator truth for ROS consumers.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odometryPublisher;

    /*!
     * @brief       Publishes a retained, bounded ground-truth path.
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pathPublisher;

    /*!
     * @brief       Publishes the ground-truth map-to-body transform.
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> p_transformBroadcaster;

    /*!
     * @brief       Publishes the static map-to-startup-fixed transform.
     */
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
        p_staticTransformBroadcaster;

    /*!
     * @brief       Accumulated truth poses in startupFixedFrame.
     */
    nav_msgs::msg::Path groundTruthPath;

    /*!
     * @brief       Fixed world frame shared with Gazebo.
     */
    std::string mapFrame;

    /*!
     * @brief       Local fixed frame coincident with the first valid pose.
     */
    std::string startupFixedFrame;

    /*!
     * @brief       Separate comparison child frame for the true rover body.
     */
    std::string groundTruthBaseFrame;

    /*!
     * @brief       Maximum number of poses retained in the path.
     */
    std::size_t pathMaximumPoses{5000U};

    /*!
     * @brief       Minimum time between retained path samples in nanoseconds.
     */
    std::int64_t pathSamplePeriodNs{100000000};

    /*!
     * @brief       Timestamp of the most recently retained path sample.
     */
    std::int64_t lastPathStampNs{-1};

    /*!
     * @brief       Full first body pose mapping startup-fixed into map.
     */
    tf2::Transform mapFromStartupFixed{tf2::Transform::getIdentity()};

    /*!
     * @brief       True after the first valid truth pose defines the frame.
     */
    bool hasStartupFixedFrame{false};
};

} /* namespace localisation::ground_truth */

#endif /* LUNAR_SIMULATOR_LOCALISATION_GROUND_TRUTH_NODE_H */
