/*!
 * @File:         alpha_node.cpp
 *
 * @Brief:        Publishes the estimated Alpha path and TF.
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

namespace lunar_simulator::alpha_system
{

/*!
 * @brief           Exposes the estimated rover trajectory.
 *
 * Ground-truth processing belongs to the localisation ground-truth node. This
 * node is intended for a single-threaded executor.
 */
class AlphaSystemNode final : public rclcpp::Node
{
  public:
    AlphaSystemNode() : Node("alpha_system_node")
    {
        const std::string estimateTopic = declare_parameter<std::string>(
            "estimate_topic", "/localisation/kalman_filter/odometry");
        const std::string estimatedPathTopic = declare_parameter<std::string>(
            "estimated_path_topic", "/localisation/kalman_filter/path");
        mapFrame = declare_parameter<std::string>("map_frame", "map");
        estimatedBaseFrame = declare_parameter<std::string>(
            "estimated_base_frame", "alpha/base_link");
        const int configuredMaximumPoses =
            declare_parameter<int>("path_maximum_poses", 5000);
        const double pathSamplePeriodS =
            declare_parameter<double>("path_sample_period_s", 0.1);
        if (configuredMaximumPoses <= 0 || pathSamplePeriodS <= 0.0)
        {
            throw std::invalid_argument(
                "Alpha path limits must be greater than zero");
        }
        pathMaximumPoses = static_cast<std::size_t>(configuredMaximumPoses);
        pathSamplePeriodNs = static_cast<std::int64_t>(pathSamplePeriodS *
                                                       NANOSECONDS_PER_SECOND);

        p_transformBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        p_estimatedPathPublisher = create_publisher<nav_msgs::msg::Path>(
            estimatedPathTopic, rclcpp::QoS(1).reliable().transient_local());
        p_estimateSubscription = create_subscription<nav_msgs::msg::Odometry>(
            estimateTopic, rclcpp::QoS(10).reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleEstimate(*p_message); });
        RCLCPP_INFO(get_logger(), "Alpha estimate: %s -> %s",
                    estimateTopic.c_str(), estimatedPathTopic.c_str());
    }

  private:
    static constexpr double NANOSECONDS_PER_SECOND = 1.0e9;

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

    static std::int64_t
    stampToNanoseconds(const builtin_interfaces::msg::Time &stamp_in)
    {
        return static_cast<std::int64_t>(stamp_in.sec) *
                   static_cast<std::int64_t>(NANOSECONDS_PER_SECOND) +
               static_cast<std::int64_t>(stamp_in.nanosec);
    }

    bool prepareOdometry(const nav_msgs::msg::Odometry &odometry_in,
                         const std::string &parentFrame_in,
                         const std::string &childFrame_in,
                         nav_msgs::msg::Odometry &odometry_out)
    {
        if (!isPoseValid(odometry_in.pose.pose))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Ignoring an invalid Alpha pose");
            return false;
        }

        odometry_out = odometry_in;
        odometry_out.header.frame_id = parentFrame_in;
        odometry_out.child_frame_id = childFrame_in;
        return true;
    }

    void handleEstimate(const nav_msgs::msg::Odometry &odometry_in)
    {
        nav_msgs::msg::Odometry preparedOdometry;
        if (!prepareOdometry(odometry_in, mapFrame, estimatedBaseFrame,
                             preparedOdometry))
        {
            return;
        }
        publishTransform(preparedOdometry);
        appendPathPose(preparedOdometry, estimatedPath,
                       lastEstimatedPathStampNs, *p_estimatedPathPublisher);
    }

    void appendPathPose(const nav_msgs::msg::Odometry &odometry_in,
                        nav_msgs::msg::Path &path_inout,
                        std::int64_t &lastStampNs_inout,
                        rclcpp::Publisher<nav_msgs::msg::Path> &publisher_inout)
    {
        const std::int64_t currentStampNs =
            stampToNanoseconds(odometry_in.header.stamp);
        if (lastStampNs_inout >= 0 && currentStampNs < lastStampNs_inout)
        {
            path_inout.poses.clear();
            lastStampNs_inout = -1;
        }
        if (lastStampNs_inout >= 0 &&
            currentStampNs - lastStampNs_inout < pathSamplePeriodNs)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odometry_in.header;
        pose.pose = odometry_in.pose.pose;
        path_inout.header = odometry_in.header;
        path_inout.poses.push_back(pose);
        if (path_inout.poses.size() > pathMaximumPoses)
        {
            path_inout.poses.erase(path_inout.poses.begin());
        }
        lastStampNs_inout = currentStampNs;
        publisher_inout.publish(path_inout);
    }

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_estimateSubscription;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_estimatedPathPublisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> p_transformBroadcaster;
    nav_msgs::msg::Path estimatedPath;
    std::string mapFrame;
    std::string estimatedBaseFrame;
    std::size_t pathMaximumPoses{5000U};
    std::int64_t pathSamplePeriodNs{100000000};
    std::int64_t lastEstimatedPathStampNs{-1};
};

} /* namespace lunar_simulator::alpha_system */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<lunar_simulator::alpha_system::AlphaSystemNode>());
    rclcpp::shutdown();
    return 0;
}
