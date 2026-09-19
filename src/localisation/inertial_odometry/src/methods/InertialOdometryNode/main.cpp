/*!
 * @File:         inertial_odometry_node.cpp
 *
 * @Brief:        Filters and integrates Alpha rover IMU measurements.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace lunar_simulator::localisation
{

class InertialOdometryNode final : public rclcpp::Node
{
  public:
    InertialOdometryNode()
        : Node("inertial_odometry"), orientation(0.0, 0.0, 0.0, 1.0)
    {
        const std::string imuTopic =
            declare_parameter<std::string>("imu_topic", "/alpha/imu");
        const std::string odometryTopic = declare_parameter<std::string>(
            "odometry_topic", "/localisation/inertial/odometry");
        const std::string filteredImuTopic = declare_parameter<std::string>(
            "filtered_imu_topic", "/localisation/inertial/filtered_imu");
        odomFrame = declare_parameter<std::string>("odom_frame", "map");
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");
        cutoffHz = declare_parameter<double>("low_pass_cutoff_hz", 5.0);
        gravityMps2 = declare_parameter<double>("gravity_mps2", 1.62);
        removeGravity = declare_parameter<bool>("remove_gravity", true);
        maximumDtS = declare_parameter<double>("maximum_dt_s", 0.25);
        calibrationSampleTarget = static_cast<int>(std::max<std::int64_t>(
            1, declare_parameter<std::int64_t>("calibration_samples", 100)));
        accelerationDeadbandMps2 =
            declare_parameter<double>("acceleration_deadband_mps2", 0.02);
        angularRateDeadbandRadps =
            declare_parameter<double>("angular_rate_deadband_radps", 0.002);

        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            odometryTopic, rclcpp::QoS(10));
        filteredImuPublisher = create_publisher<sensor_msgs::msg::Imu>(
            filteredImuTopic, rclcpp::SensorDataQoS());
        imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr p_message)
            { handleImu(*p_message); });

        RCLCPP_INFO(get_logger(),
                    "IMU odometry: %s -> %s (LPF %.1f Hz, gravity %.2f m/s^2)",
                    imuTopic.c_str(), odometryTopic.c_str(), cutoffHz,
                    gravityMps2);
    }

  private:
    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(stamp.sec) +
               1.0e-9 * static_cast<double>(stamp.nanosec);
    }

    static geometry_msgs::msg::Quaternion
    toMessage(const tf2::Quaternion &quaternion)
    {
        geometry_msgs::msg::Quaternion result;
        result.x = quaternion.x();
        result.y = quaternion.y();
        result.z = quaternion.z();
        result.w = quaternion.w();
        return result;
    }

    void handleImu(const sensor_msgs::msg::Imu &message)
    {
        const double stampS = stampToSeconds(message.header.stamp);
        if (!hasPreviousStamp)
        {
            previousStampS = stampS;
            hasPreviousStamp = true;
            return;
        }

        const double dtS = stampS - previousStampS;
        previousStampS = stampS;
        if (!(dtS > 0.0) || dtS > maximumDtS)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Ignoring IMU sample with invalid dt %.6f s",
                                 dtS);
            return;
        }

        const double timeConstantS =
            cutoffHz > 0.0 ? 1.0 / (2.0 * M_PI * cutoffHz) : 0.0;
        const double gain =
            timeConstantS > 0.0 ? dtS / (timeConstantS + dtS) : 1.0;
        const std::array<double, 3> rawAcceleration{
            message.linear_acceleration.x, message.linear_acceleration.y,
            message.linear_acceleration.z};
        const std::array<double, 3> rawAngularRate{message.angular_velocity.x,
                                                   message.angular_velocity.y,
                                                   message.angular_velocity.z};

        if (calibrationSampleCount < calibrationSampleTarget)
        {
            for (std::size_t index = 0U; index < 3U; ++index)
            {
                accelerationCalibrationSum[index] += rawAcceleration[index];
                angularCalibrationSum[index] += rawAngularRate[index];
            }
            ++calibrationSampleCount;
            if (calibrationSampleCount == calibrationSampleTarget)
            {
                for (std::size_t index = 0U; index < 3U; ++index)
                {
                    accelerationBias[index] =
                        accelerationCalibrationSum[index] /
                        static_cast<double>(calibrationSampleTarget);
                    angularRateBias[index] =
                        angularCalibrationSum[index] /
                        static_cast<double>(calibrationSampleTarget);
                }
                accelerationBias[2] -= gravityMps2;
                for (std::size_t index = 0U; index < 3U; ++index)
                {
                    filteredAcceleration[index] =
                        accelerationCalibrationSum[index] /
                            static_cast<double>(calibrationSampleTarget) -
                        accelerationBias[index];
                    filteredAngularRate[index] = 0.0;
                }
                RCLCPP_INFO(get_logger(),
                            "IMU stationary calibration complete (%d samples)",
                            calibrationSampleTarget);
            }
            return;
        }
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            filteredAcceleration[index] +=
                gain * (rawAcceleration[index] - accelerationBias[index] -
                        filteredAcceleration[index]);
            filteredAngularRate[index] +=
                gain * (rawAngularRate[index] - angularRateBias[index] -
                        filteredAngularRate[index]);
            if (std::abs(filteredAngularRate[index]) < angularRateDeadbandRadps)
            {
                filteredAngularRate[index] = 0.0;
            }
        }

        /* q(k+1) = q(k) * Exp(0.5 * omega_body * dt). */
        const double angularMagnitude =
            std::sqrt(filteredAngularRate[0] * filteredAngularRate[0] +
                      filteredAngularRate[1] * filteredAngularRate[1] +
                      filteredAngularRate[2] * filteredAngularRate[2]);
        tf2::Quaternion increment;
        if (angularMagnitude > 1.0e-12)
        {
            increment.setRotation(
                tf2::Vector3(filteredAngularRate[0] / angularMagnitude,
                             filteredAngularRate[1] / angularMagnitude,
                             filteredAngularRate[2] / angularMagnitude),
                angularMagnitude * dtS);
        }
        else
        {
            increment.setValue(0.0, 0.0, 0.0, 1.0);
        }
        orientation = orientation * increment;
        orientation.normalize();

        const tf2::Vector3 accelerationBody(filteredAcceleration[0],
                                            filteredAcceleration[1],
                                            filteredAcceleration[2]);
        tf2::Vector3 accelerationOdom =
            tf2::Matrix3x3(orientation) * accelerationBody;
        if (removeGravity)
        {
            accelerationOdom.setZ(accelerationOdom.z() - gravityMps2);
        }
        if (std::abs(accelerationOdom.x()) < accelerationDeadbandMps2)
        {
            accelerationOdom.setX(0.0);
        }
        if (std::abs(accelerationOdom.y()) < accelerationDeadbandMps2)
        {
            accelerationOdom.setY(0.0);
        }
        if (std::abs(accelerationOdom.z()) < accelerationDeadbandMps2)
        {
            accelerationOdom.setZ(0.0);
        }

        const std::array<double, 3> previousVelocity = velocityMps;
        velocityMps[0] += accelerationOdom.x() * dtS;
        velocityMps[1] += accelerationOdom.y() * dtS;
        velocityMps[2] += accelerationOdom.z() * dtS;
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            positionM[index] +=
                0.5 * (previousVelocity[index] + velocityMps[index]) * dtS;
        }

        publishFilteredImu(message, accelerationOdom);
        publishOdometry(message.header.stamp);
    }

    void publishFilteredImu(const sensor_msgs::msg::Imu &input,
                            const tf2::Vector3 &accelerationOdom)
    {
        sensor_msgs::msg::Imu output = input;
        output.header.frame_id = baseFrame;
        output.orientation = toMessage(orientation);
        output.angular_velocity.x = filteredAngularRate[0];
        output.angular_velocity.y = filteredAngularRate[1];
        output.angular_velocity.z = filteredAngularRate[2];
        const tf2::Vector3 gravityFreeAccelerationBody =
            tf2::Matrix3x3(orientation).transpose() * accelerationOdom;
        output.linear_acceleration.x = gravityFreeAccelerationBody.x();
        output.linear_acceleration.y = gravityFreeAccelerationBody.y();
        output.linear_acceleration.z = gravityFreeAccelerationBody.z();
        filteredImuPublisher->publish(output);
    }

    void publishOdometry(const builtin_interfaces::msg::Time &stamp)
    {
        nav_msgs::msg::Odometry output;
        output.header.stamp = stamp;
        output.header.frame_id = odomFrame;
        output.child_frame_id = baseFrame;
        output.pose.pose.position.x = positionM[0];
        output.pose.pose.position.y = positionM[1];
        output.pose.pose.position.z = positionM[2];
        output.pose.pose.orientation = toMessage(orientation);
        const tf2::Vector3 velocityOdom(velocityMps[0], velocityMps[1],
                                        velocityMps[2]);
        const tf2::Vector3 velocityBody =
            tf2::Matrix3x3(orientation).transpose() * velocityOdom;
        output.twist.twist.linear.x = velocityBody.x();
        output.twist.twist.linear.y = velocityBody.y();
        output.twist.twist.linear.z = velocityBody.z();
        output.twist.twist.angular.x = filteredAngularRate[0];
        output.twist.twist.angular.y = filteredAngularRate[1];
        output.twist.twist.angular.z = filteredAngularRate[2];
        output.pose.covariance[0] = 0.25;
        output.pose.covariance[7] = 0.25;
        output.pose.covariance[14] = 0.50;
        output.pose.covariance[21] = 0.08;
        output.pose.covariance[28] = 0.08;
        output.pose.covariance[35] = 0.08;
        output.twist.covariance[0] = 0.10;
        output.twist.covariance[7] = 0.10;
        output.twist.covariance[14] = 0.20;
        output.twist.covariance[21] = 0.02;
        output.twist.covariance[28] = 0.02;
        output.twist.covariance[35] = 0.02;
        odometryPublisher->publish(output);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filteredImuPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
    std::string odomFrame;
    std::string baseFrame;
    double cutoffHz{5.0};
    double gravityMps2{1.62};
    double maximumDtS{0.25};
    double accelerationDeadbandMps2{0.02};
    double angularRateDeadbandRadps{0.002};
    bool removeGravity{true};
    bool hasPreviousStamp{false};
    double previousStampS{0.0};
    std::array<double, 3> filteredAcceleration{0.0, 0.0, 0.0};
    std::array<double, 3> filteredAngularRate{0.0, 0.0, 0.0};
    std::array<double, 3> accelerationCalibrationSum{0.0, 0.0, 0.0};
    std::array<double, 3> angularCalibrationSum{0.0, 0.0, 0.0};
    std::array<double, 3> accelerationBias{0.0, 0.0, 0.0};
    std::array<double, 3> angularRateBias{0.0, 0.0, 0.0};
    std::array<double, 3> velocityMps{0.0, 0.0, 0.0};
    std::array<double, 3> positionM{0.0, 0.0, 0.0};
    int calibrationSampleTarget{100};
    int calibrationSampleCount{0};
    tf2::Quaternion orientation;
};

} // namespace lunar_simulator::localisation

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<
                 lunar_simulator::localisation::InertialOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
