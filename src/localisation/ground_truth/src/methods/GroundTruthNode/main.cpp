/*!
 * @File:         ground_truth_node.cpp
 *
 * @Brief:        Publishes the Alpha ground-truth path and TF.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

/* Generic Libraries */
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace lunar_simulator::localisation
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
        const std::string odometryTopic = declare_parameter<std::string>(
            "odometry_topic", "/localisation/ground_truth/odometry");
        const std::string pathTopic = declare_parameter<std::string>(
            "path_topic", "/localisation/ground_truth/path");
        mapFrame = declare_parameter<std::string>("map_frame", "map");
        groundTruthBaseFrame = declare_parameter<std::string>(
            "ground_truth_base_frame", "alpha/ground_truth_base_link");
        const int configuredMaximumPoses =
            declare_parameter<int>("path_maximum_poses", 5000);
        const double pathSamplePeriodS =
            declare_parameter<double>("path_sample_period_s", 0.1);
        if (configuredMaximumPoses <= 0 || pathSamplePeriodS <= 0.0)
        {
            throw std::invalid_argument(
                "Ground-truth path limits must be greater than zero");
        }
        pathMaximumPoses = static_cast<std::size_t>(configuredMaximumPoses);
        pathSamplePeriodNs = static_cast<std::int64_t>(pathSamplePeriodS *
                                                       NANOSECONDS_PER_SECOND);

        p_transformBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        p_pathPublisher = create_publisher<nav_msgs::msg::Path>(
            pathTopic, rclcpp::QoS(1).reliable().transient_local());
        p_odometrySubscription = create_subscription<nav_msgs::msg::Odometry>(
            odometryTopic, rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleOdometry(*p_message); });

        RCLCPP_INFO(get_logger(), "Ground truth: %s -> %s",
                    odometryTopic.c_str(), pathTopic.c_str());
    }

  private:
    /*! Nanoseconds in one second. */
    static constexpr double NANOSECONDS_PER_SECOND = 1.0e9;

    /*!
     * @brief           Converts a ROS timestamp to nanoseconds.
     * @param[in]       stamp_in Timestamp to convert.
     * @return          Timestamp in nanoseconds.
     */
    static std::int64_t
    stampToNanoseconds(const builtin_interfaces::msg::Time &stamp_in)
    {
        return static_cast<std::int64_t>(stamp_in.sec) *
                   static_cast<std::int64_t>(NANOSECONDS_PER_SECOND) +
               static_cast<std::int64_t>(stamp_in.nanosec);
    }

    /*!
     * @brief           Tests whether a pose can be published safely.
     * @param[in]       pose_in Pose expressed in the map frame.
     * @return          True when position and orientation are finite and the
     *                  quaternion has a nonzero norm.
     */
    static bool isPoseValid(const geometry_msgs::msg::Pose &pose_in)
    {
        const double orientationNormSquared =
            pose_in.orientation.x * pose_in.orientation.x +
            pose_in.orientation.y * pose_in.orientation.y +
            pose_in.orientation.z * pose_in.orientation.z +
            pose_in.orientation.w * pose_in.orientation.w;
        return std::isfinite(pose_in.position.x) &&
               std::isfinite(pose_in.position.y) &&
               std::isfinite(pose_in.position.z) &&
               std::isfinite(orientationNormSquared) &&
               orientationNormSquared > 1.0e-12;
    }

    /*!
     * @brief           Publishes one truth transform and sampled path point.
     * @param[in]       odometry_in Gazebo truth odometry in the map frame.
     */
    void handleOdometry(const nav_msgs::msg::Odometry &odometry_in)
    {
        if (!isPoseValid(odometry_in.pose.pose))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Ignoring an invalid ground-truth pose");
            return;
        }

        nav_msgs::msg::Odometry truthOdometry = odometry_in;
        truthOdometry.header.frame_id = mapFrame;
        truthOdometry.child_frame_id = groundTruthBaseFrame;
        publishTransform(truthOdometry);
        appendPathPose(truthOdometry);
    }

    /*!
     * @brief           Adds a time-sampled truth pose to the bounded path.
     * @param[in]       odometry_in Valid truth odometry in the map frame.
     */
    void appendPathPose(const nav_msgs::msg::Odometry &odometry_in)
    {
        const std::int64_t currentStampNs =
            stampToNanoseconds(odometry_in.header.stamp);
        if (lastPathStampNs >= 0 && currentStampNs < lastPathStampNs)
        {
            groundTruthPath.poses.clear();
            lastPathStampNs = -1;
        }
        if (lastPathStampNs >= 0 &&
            currentStampNs - lastPathStampNs < pathSamplePeriodNs)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odometry_in.header;
        pose.pose = odometry_in.pose.pose;
        groundTruthPath.header = odometry_in.header;
        groundTruthPath.poses.push_back(pose);
        if (groundTruthPath.poses.size() > pathMaximumPoses)
        {
            groundTruthPath.poses.erase(groundTruthPath.poses.begin());
        }
        lastPathStampNs = currentStampNs;
        p_pathPublisher->publish(groundTruthPath);
    }

    /*!
     * @brief           Publishes map to ground-truth body transform.
     * @param[in]       odometry_in Valid truth odometry in the map frame.
     */
    void publishTransform(const nav_msgs::msg::Odometry &odometry_in)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odometry_in.header;
        transform.child_frame_id = odometry_in.child_frame_id;
        transform.transform.translation.x = odometry_in.pose.pose.position.x;
        transform.transform.translation.y = odometry_in.pose.pose.position.y;
        transform.transform.translation.z = odometry_in.pose.pose.position.z;
        transform.transform.rotation = odometry_in.pose.pose.orientation;
        p_transformBroadcaster->sendTransform(transform);
    }

    /*! Receives simulator truth odometry. */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_odometrySubscription;

    /*! Publishes a retained, bounded ground-truth path. */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pathPublisher;

    /*! Publishes the ground-truth map-to-body transform. */
    std::unique_ptr<tf2_ros::TransformBroadcaster> p_transformBroadcaster;

    /*! Accumulated ground-truth poses, all expressed in map. */
    nav_msgs::msg::Path groundTruthPath;

    /*! Fixed world frame shared with Gazebo. */
    std::string mapFrame;

    /*! Separate comparison child frame for the true rover body. */
    std::string groundTruthBaseFrame;

    /*! Maximum number of poses retained in the path. */
    std::size_t pathMaximumPoses{5000U};

    /*! Minimum time between retained path samples in nanoseconds. */
    std::int64_t pathSamplePeriodNs{100000000};

    /*! Timestamp of the most recently retained path sample. */
    std::int64_t lastPathStampNs{-1};
};

} /* namespace lunar_simulator::localisation */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<lunar_simulator::localisation::GroundTruthNode>());
    rclcpp::shutdown();
    return 0;
}
