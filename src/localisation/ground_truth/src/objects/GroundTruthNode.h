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
#include <tf2_ros/transform_broadcaster.h>

namespace localisation::ground_truth
{

/*!
 * @brief           Converts simulator truth odometry into comparison outputs.
 *
 * The input pose is already expressed in the Gazebo `map` world frame. The
 * node does not rebase or otherwise alter that pose. It is intended for a
 * single-threaded executor.
 */
class GroundTruthNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Configures ground-truth path and TF publication.
     * @throws          std::invalid_argument if a path limit is invalid.
     */
    GroundTruthNode() : Node("ground_truth")
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
            "/" + systemName + "/localisation/ground_truth/odometry");

        /* Output topic: the retained, rate-limited comparison path. */
        const std::string pathTopic = declare_parameter<std::string>(
            "path_topic",
            "/" + systemName + "/localisation/ground_truth/path");

        /*!
         * Output topic: this node's own first valid odometry sample,
         * published exactly once, latched. This is the rover's actual
         * settled resting pose (see alpha_node/main.cpp's startup delay,
         * which guarantees physics settling has already finished by the
         * time this node's first message arrives) -- continuous_ekf and
         * wheel_odometry both seed their own map-frame origin from it
         * instead of a manually re-measured, terrain-specific constant.
         */
        const std::string initialPoseTopic = declare_parameter<std::string>(
            "initial_pose_topic",
            "/" + systemName + "/localisation/ground_truth/initial_pose");

        /* Fixed world frame shared with Gazebo and every other node. */
        mapFrame = declare_parameter<std::string>("map_frame", "map");

        /* This node's own child frame, distinct from the estimator's. */
        groundTruthBaseFrame = declare_parameter<std::string>(
            "ground_truth_base_frame", "alpha/ground_truth_base_link");

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

        /* Latch the path so a newly opened RViz view sees the current
         * history immediately, rather than waiting for the next sample. */
        p_pathPublisher = create_publisher<nav_msgs::msg::Path>(
            pathTopic, rclcpp::QoS(1).reliable().transient_local());

        /* Latched the same way, so continuous_ekf/wheel_odometry receive
         * this one-time message regardless of when they subscribe relative
         * to when it is published (see handleOdometryCallBack()). */
        p_initialPosePublisher = create_publisher<nav_msgs::msg::Odometry>(
            initialPoseTopic, rclcpp::QoS(1).reliable().transient_local());

        /* Every incoming odometry message triggers handleOdometryCallBack(). */
        p_odometrySubscription = create_subscription<nav_msgs::msg::Odometry>(
            odometryTopic, rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleOdometryCallBack(*p_message); });

        /* Record the resolved topic names once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(), "Ground truth: %s -> %s, %s",
                    odometryTopic.c_str(), pathTopic.c_str(),
                    initialPoseTopic.c_str());
    }

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
     * @brief       Publishes a retained, bounded ground-truth path.
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pathPublisher;

    /*!
     * @brief       Publishes this node's first valid odometry sample,
     *              exactly once (see handleOdometryCallBack()).
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        p_initialPosePublisher;

    /*!
     * @brief       Publishes the ground-truth map-to-body transform.
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> p_transformBroadcaster;

    /*!
     * @brief       Accumulated ground-truth poses, all expressed in map.
     */
    nav_msgs::msg::Path groundTruthPath;

    /*!
     * @brief       Fixed world frame shared with Gazebo.
     */
    std::string mapFrame;

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
     * @brief       True once p_initialPosePublisher has published its one
     *              and only message (see handleOdometryCallBack()).
     */
    bool hasPublishedInitialPose{false};
};

} /* namespace localisation::ground_truth */

#endif /* LUNAR_SIMULATOR_LOCALISATION_GROUND_TRUTH_NODE_H */
