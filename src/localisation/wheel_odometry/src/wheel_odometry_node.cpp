/*!
 * @File:         wheel_odometry_node.cpp
 *
 * @Brief:        Computes slip-adjusted six-wheel steering odometry.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace lunar_simulator::localisation
{

class WheelOdometryNode final : public rclcpp::Node
{
  public:
    WheelOdometryNode() : Node("wheel_odometry")
    {
        const std::string inputTopic = declare_parameter<std::string>(
            "joint_state_topic", "/alpha/joint_states");
        const std::string outputTopic = declare_parameter<std::string>(
            "odometry_topic", "/localisation/wheel/odometry");
        const std::string visualOdometryTopic = declare_parameter<std::string>(
            "visual_odometry_topic", "/localisation/visual/odometry");
        const std::string slipRatioTopic = declare_parameter<std::string>(
            "slip_ratio_topic", "/localisation/wheel/slip_ratios");
        odomFrame = declare_parameter<std::string>("odom_frame", "map");
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");
        wheelRadiusM = declare_parameter<double>("wheel_radius_m", 0.1425);
        maximumIntegrationDtS =
            declare_parameter<double>("maximum_integration_dt_s", 5.0);
        shouldEstimateSlip =
            declare_parameter<bool>("estimate_slip_from_visual", true);
        slipEstimationGain =
            declare_parameter<double>("slip_estimation_gain", 0.15);
        slipRecoveryGain =
            declare_parameter<double>("slip_recovery_gain", 0.05);
        slipRatioDeadband =
            declare_parameter<double>("slip_ratio_deadband", 0.05);
        minimumWheelSpeedMps =
            declare_parameter<double>("minimum_wheel_speed_mps", 0.005);
        visualOdometryTimeoutS =
            declare_parameter<double>("visual_odometry_timeout_s", 0.25);
        maximumVisualTwistVariance =
            declare_parameter<double>("maximum_visual_twist_variance", 0.08);
        maximumSlipRatio =
            declare_parameter<double>("maximum_slip_ratio", 0.30);
        if (wheelRadiusM <= 0.0 || maximumIntegrationDtS <= 0.0 ||
            slipEstimationGain < 0.0 ||
            slipEstimationGain > 1.0 || slipRecoveryGain < 0.0 ||
            slipRecoveryGain > 1.0 || minimumWheelSpeedMps < 0.0 ||
            visualOdometryTimeoutS <= 0.0 || maximumVisualTwistVariance < 0.0 ||
            maximumSlipRatio < 0.0 || maximumSlipRatio > 0.99 ||
            slipRatioDeadband < 0.0 || slipRatioDeadband >= 1.0)
        {
            throw std::invalid_argument(
                "Wheel and visual-slip parameters are outside valid bounds");
        }

        loadSixValues("slip_ratios", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      slipRatios);
        loadSixValues("drive_direction_multipliers",
                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                      driveDirectionMultipliers);
        loadSixValues("wheel_x_m", {0.64, 0.64, 0.0, 0.0, -0.72, -0.72},
                      wheelXM);
        loadSixValues("wheel_y_m", {0.60, -0.60, 0.60, -0.60, 0.60, -0.60},
                      wheelYM);
        for (double &slipRatio : slipRatios)
        {
            slipRatio = std::clamp(slipRatio, 0.0, maximumSlipRatio);
        }

        driveJointNames = {
            "alpha/front_left_drive_joint",  "alpha/front_right_drive_joint",
            "alpha/centre_left_drive_joint", "alpha/centre_right_drive_joint",
            "alpha/rear_left_drive_joint",   "alpha/rear_right_drive_joint"};
        steerJointNames = {
            "alpha/front_left_steer_joint",  "alpha/front_right_steer_joint",
            "alpha/centre_left_steer_joint", "alpha/centre_right_steer_joint",
            "alpha/rear_left_steer_joint",   "alpha/rear_right_steer_joint"};

        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputTopic, rclcpp::QoS(10));
        slipRatioPublisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            slipRatioTopic, rclcpp::QoS(1).reliable().transient_local());
        jointSubscription = create_subscription<sensor_msgs::msg::JointState>(
            inputTopic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr p_message)
            { handleJointState(*p_message); });
        visualOdometrySubscription =
            create_subscription<nav_msgs::msg::Odometry>(
                visualOdometryTopic, rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
                { handleVisualOdometry(*p_message); });
        RCLCPP_INFO(get_logger(),
                    "Wheel odometry: %s -> %s; visual slip reference: %s -> %s",
                    inputTopic.c_str(), outputTopic.c_str(),
                    visualOdometryTopic.c_str(), slipRatioTopic.c_str());
    }

  private:
    void loadSixValues(const std::string &name,
                       const std::vector<double> &defaults,
                       std::array<double, 6> &values)
    {
        std::vector<double> configured =
            declare_parameter<std::vector<double>>(name, defaults);
        if (configured.size() != values.size())
        {
            RCLCPP_WARN(get_logger(),
                        "Parameter %s needs six values; using defaults",
                        name.c_str());
            configured = defaults;
        }
        std::copy(configured.begin(), configured.end(), values.begin());
    }

    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(stamp.sec) +
               1.0e-9 * static_cast<double>(stamp.nanosec);
    }

    static bool findJoint(const sensor_msgs::msg::JointState &message,
                          const std::string &name, double &position,
                          double &velocity)
    {
        for (std::size_t index = 0U; index < message.name.size(); ++index)
        {
            if (message.name[index] == name)
            {
                if (index >= message.position.size() ||
                    index >= message.velocity.size())
                {
                    return false;
                }
                position = message.position[index];
                velocity = message.velocity[index];
                return true;
            }
        }
        return false;
    }

    void handleVisualOdometry(const nav_msgs::msg::Odometry &message_in)
    {
        const double maximumTwistVariance = std::max(
            {message_in.twist.covariance[0], message_in.twist.covariance[7],
             message_in.twist.covariance[35]});
        const Eigen::Vector3d visualTwistBody(message_in.twist.twist.linear.x,
                                              message_in.twist.twist.linear.y,
                                              message_in.twist.twist.angular.z);
        if (!visualTwistBody.allFinite() ||
            !std::isfinite(maximumTwistVariance) ||
            maximumTwistVariance > maximumVisualTwistVariance)
        {
            return;
        }

        latestVisualTwistBody = visualTwistBody;
        latestVisualStampS = stampToSeconds(message_in.header.stamp);
        hasVisualReference = true;
    }

    void updateSlipRatios(double jointStampS_in,
                          const std::array<double, 6> &rawWheelSpeedMps_in,
                          const std::array<double, 6> &steerAngleRad_in)
    {
        if (!shouldEstimateSlip || !hasVisualReference ||
            std::abs(jointStampS_in - latestVisualStampS) >
                visualOdometryTimeoutS ||
            latestVisualStampS <= lastSlipUpdateStampS)
        {
            return;
        }

        const double boundedGain = std::clamp(slipEstimationGain, 0.0, 1.0);
        const double boundedMaximumSlip =
            std::clamp(maximumSlipRatio, 0.0, 0.99);
        std::vector<double> observedSlipRatios;
        observedSlipRatios.reserve(slipRatios.size());
        for (std::size_t wheel = 0U; wheel < slipRatios.size(); ++wheel)
        {
            const double rawWheelSpeedMps = rawWheelSpeedMps_in[wheel];
            if (std::abs(rawWheelSpeedMps) < minimumWheelSpeedMps)
            {
                continue;
            }

            const double cosine = std::cos(steerAngleRad_in[wheel]);
            const double sine = std::sin(steerAngleRad_in[wheel]);
            const double expectedRollingSpeedMps =
                cosine * (latestVisualTwistBody.x() -
                          wheelYM[wheel] * latestVisualTwistBody.z()) +
                sine * (latestVisualTwistBody.y() +
                        wheelXM[wheel] * latestVisualTwistBody.z());
            if (rawWheelSpeedMps * expectedRollingSpeedMps < 0.0)
            {
                continue;
            }

            const double observedSlipRatioUnbounded =
                1.0 - expectedRollingSpeedMps / rawWheelSpeedMps;
            if (observedSlipRatioUnbounded < 0.0 ||
                observedSlipRatioUnbounded > boundedMaximumSlip)
            {
                continue;
            }
            observedSlipRatios.push_back(observedSlipRatioUnbounded);
        }
        if (observedSlipRatios.empty())
        {
            return;
        }

        std::sort(observedSlipRatios.begin(), observedSlipRatios.end());
        const std::size_t middle = observedSlipRatios.size() / 2U;
        double observedSlipRatio = observedSlipRatios[middle];
        if (observedSlipRatios.size() % 2U == 0U)
        {
            observedSlipRatio =
                0.5 * (observedSlipRatios[middle - 1U] + observedSlipRatio);
        }
        if (observedSlipRatio < slipRatioDeadband)
        {
            observedSlipRatio = 0.0;
        }

        /* One robust common ratio prevents VO yaw noise from manufacturing
         * unequal left/right wheel speeds and a false rover turn. The six
         * values remain explicit so a per-wheel estimator can replace this
         * model later without changing the public topic.
         */
        for (double &slipRatio : slipRatios)
        {
            slipRatio = (1.0 - boundedGain) * slipRatio +
                        boundedGain * observedSlipRatio;
        }
        lastSlipUpdateStampS = latestVisualStampS;
        publishSlipRatios();
    }

    void publishSlipRatios()
    {
        std_msgs::msg::Float64MultiArray output;
        output.data.assign(slipRatios.begin(), slipRatios.end());
        slipRatioPublisher->publish(output);
        hasPublishedSlipRatios = true;
    }

    void recoverStaleSlipEstimate(double jointStampS_in)
    {
        if (!hasVisualReference ||
            jointStampS_in - latestVisualStampS <= visualOdometryTimeoutS)
        {
            return;
        }

        bool hasChangedRatio = false;
        for (double &slipRatio : slipRatios)
        {
            const double recoveredRatio = slipRatio * (1.0 - slipRecoveryGain);
            hasChangedRatio =
                hasChangedRatio || std::abs(recoveredRatio - slipRatio) > 1.0e-6;
            slipRatio = recoveredRatio < 1.0e-4 ? 0.0 : recoveredRatio;
        }
        if (hasChangedRatio)
        {
            publishSlipRatios();
        }
    }

    void handleJointState(const sensor_msgs::msg::JointState &message)
    {
        Eigen::Matrix<double, 6, 3> rollingMatrix;
        Eigen::Matrix<double, 6, 1> wheelSpeedMps;
        std::array<double, 6> rawWheelSpeedMps{};
        std::array<double, 6> steerAngleRad{};
        for (std::size_t wheel = 0U; wheel < 6U; ++wheel)
        {
            double unusedDrivePosition = 0.0;
            double driveRateRadps = 0.0;
            double wheelSteerAngleRad = 0.0;
            double unusedSteerRate = 0.0;
            if (!findJoint(message, driveJointNames[wheel], unusedDrivePosition,
                           driveRateRadps) ||
                !findJoint(message, steerJointNames[wheel], wheelSteerAngleRad,
                           unusedSteerRate))
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 3000,
                    "Waiting for all six drive and steering joints");
                return;
            }

            steerAngleRad[wheel] = wheelSteerAngleRad;
            const double cosine = std::cos(wheelSteerAngleRad);
            const double sine = std::sin(wheelSteerAngleRad);
            rollingMatrix(static_cast<Eigen::Index>(wheel), 0) = cosine;
            rollingMatrix(static_cast<Eigen::Index>(wheel), 1) = sine;
            rollingMatrix(static_cast<Eigen::Index>(wheel), 2) =
                -wheelYM[wheel] * cosine + wheelXM[wheel] * sine;
            rawWheelSpeedMps[wheel] = driveDirectionMultipliers[wheel] *
                                      wheelRadiusM * driveRateRadps;
        }

        const double stampS = stampToSeconds(message.header.stamp);
        updateSlipRatios(stampS, rawWheelSpeedMps, steerAngleRad);
        recoverStaleSlipEstimate(stampS);
        if (!hasPublishedSlipRatios)
        {
            publishSlipRatios();
        }
        for (std::size_t wheel = 0U; wheel < slipRatios.size(); ++wheel)
        {
            const double boundedSlip =
                std::clamp(slipRatios[wheel], 0.0, maximumSlipRatio);
            wheelSpeedMps(static_cast<Eigen::Index>(wheel)) =
                rawWheelSpeedMps[wheel] * (1.0 - boundedSlip);
        }

        /* For each wheel i:
         * cos(delta_i) vx + sin(delta_i) vy
         * + (-y_i cos(delta_i) + x_i sin(delta_i)) wz = r omega_i.
         * The overdetermined six-wheel system is solved in minimum-norm least
         * squares. This sets an unobservable lateral component to zero when
         * all wheels are parallel instead of allowing arbitrary sideways
         * drift.
         */
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 3>>
            rollingDecomposition(rollingMatrix);
        rollingDecomposition.setThreshold(1.0e-6);
        const Eigen::Vector3d bodyTwist =
            rollingDecomposition.solve(wheelSpeedMps);
        if (!bodyTwist.allFinite())
        {
            return;
        }

        if (hasPreviousStamp)
        {
            const double dtS = stampS - previousStampS;
            if (dtS > 0.0 && dtS <= maximumIntegrationDtS)
            {
                const double worldVx = std::cos(yawRad) * bodyTwist.x() -
                                       std::sin(yawRad) * bodyTwist.y();
                const double worldVy = std::sin(yawRad) * bodyTwist.x() +
                                       std::cos(yawRad) * bodyTwist.y();
                positionXM += worldVx * dtS;
                positionYM += worldVy * dtS;
                yawRad = wrapAngle(yawRad + bodyTwist.z() * dtS);
            }
            else if (dtS > maximumIntegrationDtS)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skipping %.3f s wheel integration gap (limit %.3f s)",
                    dtS, maximumIntegrationDtS);
            }
        }
        previousStampS = stampS;
        hasPreviousStamp = true;
        publishOdometry(message.header.stamp, bodyTwist);
    }

    static double wrapAngle(double angleRad)
    {
        constexpr double pi = 3.14159265358979323846;
        while (angleRad > pi)
        {
            angleRad -= 2.0 * pi;
        }
        while (angleRad < -pi)
        {
            angleRad += 2.0 * pi;
        }
        return angleRad;
    }

    void publishOdometry(const builtin_interfaces::msg::Time &stamp,
                         const Eigen::Vector3d &bodyTwist)
    {
        nav_msgs::msg::Odometry output;
        output.header.stamp = stamp;
        output.header.frame_id = odomFrame;
        output.child_frame_id = baseFrame;
        output.pose.pose.position.x = positionXM;
        output.pose.pose.position.y = positionYM;
        output.pose.pose.orientation.z = std::sin(0.5 * yawRad);
        output.pose.pose.orientation.w = std::cos(0.5 * yawRad);
        output.twist.twist.linear.x = bodyTwist.x();
        output.twist.twist.linear.y = bodyTwist.y();
        output.twist.twist.angular.z = bodyTwist.z();
        output.pose.covariance[0] = 0.03;
        output.pose.covariance[7] = 0.03;
        output.pose.covariance[14] = 1.0e3;
        output.pose.covariance[21] = 1.0e3;
        output.pose.covariance[28] = 1.0e3;
        output.pose.covariance[35] = 0.04;
        output.twist.covariance[0] = 0.01;
        output.twist.covariance[7] = 0.01;
        output.twist.covariance[14] = 1.0e3;
        output.twist.covariance[21] = 1.0e3;
        output.twist.covariance[28] = 1.0e3;
        output.twist.covariance[35] = 0.02;
        odometryPublisher->publish(output);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        jointSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        visualOdometrySubscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        slipRatioPublisher;
    std::array<std::string, 6> driveJointNames;
    std::array<std::string, 6> steerJointNames;
    std::array<double, 6> slipRatios{};
    std::array<double, 6> driveDirectionMultipliers{};
    std::array<double, 6> wheelXM{};
    std::array<double, 6> wheelYM{};
    std::string odomFrame;
    std::string baseFrame;
    double wheelRadiusM{0.1425};
    double maximumIntegrationDtS{5.0};
    double slipEstimationGain{0.15};
    double slipRecoveryGain{0.05};
    double slipRatioDeadband{0.05};
    double minimumWheelSpeedMps{0.005};
    double visualOdometryTimeoutS{0.25};
    double maximumVisualTwistVariance{0.08};
    double maximumSlipRatio{0.95};
    double latestVisualStampS{0.0};
    double lastSlipUpdateStampS{-1.0};
    Eigen::Vector3d latestVisualTwistBody{Eigen::Vector3d::Zero()};
    bool shouldEstimateSlip{true};
    bool hasVisualReference{false};
    bool hasPublishedSlipRatios{false};
    bool hasPreviousStamp{false};
    double previousStampS{0.0};
    double positionXM{0.0};
    double positionYM{0.0};
    double yawRad{0.0};
};

} // namespace lunar_simulator::localisation

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<lunar_simulator::localisation::WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
