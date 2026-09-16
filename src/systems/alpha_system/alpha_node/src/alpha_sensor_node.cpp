/*!
 * @file            alpha_sensor_node.cpp
 *
 * @brief           Applies configurable noise to Alpha simulator sensors.
 *
 * @date            15/09/2026
 */

/* C++ Standard Library Includes */
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

/* External Library Includes */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace lunar_simulator::alpha_system
{

/*!
 * @brief           Models Alpha sensor errors at the simulator hardware
 *                  boundary.
 *
 * Raw Gazebo measurements remain private under `/alpha/raw`. Noisy
 * measurements are published through Alpha's normal public sensor topics.
 * Gazebo applies camera noise natively to avoid a high-bandwidth image relay.
 */
class AlphaSensorNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Creates the Alpha sensor-noise interface.
     *
     * @throws          std::invalid_argument if a noise parameter is invalid.
     */
    AlphaSensorNode()
        : Node("alpha_sensor_node"),
          randomEngine(static_cast<std::mt19937::result_type>(
              declare_parameter<std::int64_t>("random_seed", 7302028)))
    {
        noiseEnabled = declare_parameter<bool>("noise_enabled", true);
        imuAngularVelocityStddev_radPerS = declareNonnegativeParameter(
            "imu_angular_velocity_stddev_radps", 0.008);
        imuLinearAccelerationStddev_mPerS2 = declareNonnegativeParameter(
            "imu_linear_acceleration_stddev_mps2", 0.05);
        imuOrientationStddev_rad =
            declareNonnegativeParameter("imu_orientation_stddev_rad", 0.003);
        wheelPositionStddev_rad =
            declareNonnegativeParameter("wheel_position_stddev_rad", 0.002);
        wheelVelocityStddev_radPerS =
            declareNonnegativeParameter("wheel_velocity_stddev_radps", 0.015);
        imuAngularVelocityBias_radPerS = declareTripletParameter(
            "imu_angular_velocity_bias_radps",
            std::array<double, AXIS_COUNT>{0.002, -0.001, 0.0015});
        imuLinearAccelerationBias_mPerS2 = declareTripletParameter(
            "imu_linear_acceleration_bias_mps2",
            std::array<double, AXIS_COUNT>{0.02, -0.015, 0.025});

        p_inertialCallbackGroup =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        configureImuInterface();
        configureJointStateInterface();

        RCLCPP_INFO(
            get_logger(),
            "Alpha sensor noise %s (seed fixed by random_seed parameter)",
            noiseEnabled ? "enabled" : "disabled");
    }

  private:
    /*! Number of spatial axes in an IMU vector. */
    static constexpr std::size_t AXIS_COUNT = 3U;

    /*! Index of the x axis. */
    static constexpr std::size_t X_AXIS = 0U;

    /*! Index of the y axis. */
    static constexpr std::size_t Y_AXIS = 1U;

    /*! Index of the z axis. */
    static constexpr std::size_t Z_AXIS = 2U;

    /*!
     * @brief           Declares and validates one nonnegative parameter.
     *
     * @param[in]       name_in
     *                  Externally controlled ROS parameter name.
     * @param[in]       defaultValue_in
     *                  Default parameter value.
     *
     * @return          Validated parameter value.
     */
    double declareNonnegativeParameter(const std::string &name_in,
                                       double defaultValue_in)
    {
        const double parameterValue =
            declare_parameter<double>(name_in, defaultValue_in);
        if (!std::isfinite(parameterValue) || parameterValue < 0.0)
        {
            throw std::invalid_argument(name_in +
                                        " must be finite and nonnegative");
        }
        return parameterValue;
    }

    /*!
     * @brief           Declares and validates a three-axis parameter.
     *
     * @param[in]       name_in
     *                  Externally controlled ROS parameter name.
     * @param[in]       defaults_in
     *                  Default x, y and z values.
     *
     * @return          Validated x, y and z values.
     */
    std::array<double, AXIS_COUNT>
    declareTripletParameter(const std::string &name_in,
                            const std::array<double, AXIS_COUNT> &defaults_in)
    {
        const std::vector<double> configuredValues =
            declare_parameter<std::vector<double>>(
                name_in,
                std::vector<double>(defaults_in.begin(), defaults_in.end()));
        if (configuredValues.size() != AXIS_COUNT)
        {
            throw std::invalid_argument(name_in +
                                        " must contain exactly three values");
        }
        for (const double configuredValue : configuredValues)
        {
            if (!std::isfinite(configuredValue))
            {
                throw std::invalid_argument(name_in + " values must be finite");
            }
        }
        return std::array<double, AXIS_COUNT>{configuredValues[X_AXIS],
                                              configuredValues[Y_AXIS],
                                              configuredValues[Z_AXIS]};
    }

    /*! @brief Configures the raw and public IMU topics. */
    void configureImuInterface()
    {
        const std::string rawImuTopic =
            declare_parameter<std::string>("raw_imu_topic", "/alpha/raw/imu");
        const std::string imuTopic =
            declare_parameter<std::string>("imu_topic", "/alpha/imu");

        p_imuPublisher = create_publisher<sensor_msgs::msg::Imu>(
            imuTopic, rclcpp::SensorDataQoS());
        rclcpp::SubscriptionOptions subscriptionOptions;
        subscriptionOptions.callback_group = p_inertialCallbackGroup;
        p_rawImuSubscription = create_subscription<sensor_msgs::msg::Imu>(
            rawImuTopic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr p_message)
            { publishNoisyImu(*p_message); }, subscriptionOptions);
    }

    /*! @brief Configures the raw and public joint-state topics. */
    void configureJointStateInterface()
    {
        const std::string rawJointStateTopic = declare_parameter<std::string>(
            "raw_joint_state_topic", "/alpha/raw/joint_states");
        const std::string jointStateTopic = declare_parameter<std::string>(
            "joint_state_topic", "/alpha/joint_states");

        p_jointStatePublisher = create_publisher<sensor_msgs::msg::JointState>(
            jointStateTopic, rclcpp::SensorDataQoS());
        rclcpp::SubscriptionOptions subscriptionOptions;
        subscriptionOptions.callback_group = p_inertialCallbackGroup;
        p_rawJointStateSubscription =
            create_subscription<sensor_msgs::msg::JointState>(
                rawJointStateTopic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::JointState::ConstSharedPtr
                           p_message) { publishNoisyJointState(*p_message); },
                subscriptionOptions);
    }

    /*!
     * @brief           Samples zero-mean Gaussian noise.
     *
     * @param[in]       standardDeviation_in
     *                  Standard deviation in the caller's physical unit.
     *
     * @return          One noise sample.
     */
    double sampleGaussian(double standardDeviation_in)
    {
        if (!noiseEnabled || standardDeviation_in == 0.0)
        {
            return 0.0;
        }
        std::normal_distribution<double> distribution(0.0,
                                                      standardDeviation_in);
        return distribution(randomEngine);
    }

    /*!
     * @brief           Adds configured error to one IMU measurement.
     *
     * @param[in]       message_in
     *                  Raw IMU measurement in the sensor frame.
     */
    void publishNoisyImu(const sensor_msgs::msg::Imu &message_in)
    {
        sensor_msgs::msg::Imu message = message_in;
        if (noiseEnabled)
        {
            message.angular_velocity.x +=
                imuAngularVelocityBias_radPerS[X_AXIS] +
                sampleGaussian(imuAngularVelocityStddev_radPerS);
            message.angular_velocity.y +=
                imuAngularVelocityBias_radPerS[Y_AXIS] +
                sampleGaussian(imuAngularVelocityStddev_radPerS);
            message.angular_velocity.z +=
                imuAngularVelocityBias_radPerS[Z_AXIS] +
                sampleGaussian(imuAngularVelocityStddev_radPerS);
            message.linear_acceleration.x +=
                imuLinearAccelerationBias_mPerS2[X_AXIS] +
                sampleGaussian(imuLinearAccelerationStddev_mPerS2);
            message.linear_acceleration.y +=
                imuLinearAccelerationBias_mPerS2[Y_AXIS] +
                sampleGaussian(imuLinearAccelerationStddev_mPerS2);
            message.linear_acceleration.z +=
                imuLinearAccelerationBias_mPerS2[Z_AXIS] +
                sampleGaussian(imuLinearAccelerationStddev_mPerS2);
            perturbOrientation(message.orientation);
        }

        if (noiseEnabled)
        {
            addVarianceToCovariance(imuOrientationStddev_rad *
                                        imuOrientationStddev_rad,
                                    message.orientation_covariance);
            addVarianceToCovariance(imuAngularVelocityStddev_radPerS *
                                        imuAngularVelocityStddev_radPerS,
                                    message.angular_velocity_covariance);
            addVarianceToCovariance(imuLinearAccelerationStddev_mPerS2 *
                                        imuLinearAccelerationStddev_mPerS2,
                                    message.linear_acceleration_covariance);
        }
        p_imuPublisher->publish(message);
    }

    /*!
     * @brief           Adds a small body-frame rotation to an orientation.
     *
     * @param[in,out]   orientation_inout
     *                  IMU quaternion rotated in place.
     */
    void perturbOrientation(geometry_msgs::msg::Quaternion &orientation_inout)
    {
        tf2::Quaternion orientation(orientation_inout.x, orientation_inout.y,
                                    orientation_inout.z, orientation_inout.w);
        tf2::Quaternion perturbation;
        perturbation.setRPY(sampleGaussian(imuOrientationStddev_rad),
                            sampleGaussian(imuOrientationStddev_rad),
                            sampleGaussian(imuOrientationStddev_rad));
        orientation *= perturbation;
        orientation.normalize();
        orientation_inout.x = orientation.x();
        orientation_inout.y = orientation.y();
        orientation_inout.z = orientation.z();
        orientation_inout.w = orientation.w();
    }

    /*!
     * @brief           Adds a variance to each covariance diagonal.
     *
     * @param[in]       variance_in
     *                  Added variance in the measurement's squared unit.
     * @param[in,out]   covariance_inout
     *                  ROS three-axis covariance matrix.
     */
    static void
    addVarianceToCovariance(double variance_in,
                            std::array<double, 9U> &covariance_inout)
    {
        if (covariance_inout[0U] < 0.0)
        {
            covariance_inout.fill(0.0);
        }
        covariance_inout[0U] += variance_in;
        covariance_inout[4U] += variance_in;
        covariance_inout[8U] += variance_in;
    }

    /*!
     * @brief           Adds configured noise to measured joint states.
     *
     * @param[in]       message_in
     *                  Raw joint measurement.
     */
    void publishNoisyJointState(const sensor_msgs::msg::JointState &message_in)
    {
        sensor_msgs::msg::JointState message = message_in;
        for (double &position_rad : message.position)
        {
            position_rad += sampleGaussian(wheelPositionStddev_rad);
        }
        for (double &velocity_radPerS : message.velocity)
        {
            velocity_radPerS += sampleGaussian(wheelVelocityStddev_radPerS);
        }
        p_jointStatePublisher->publish(message);
    }

    /*! Whether configured stochastic errors are applied. */
    bool noiseEnabled{true};

    /*! IMU angular-rate white-noise standard deviation in rad/s. */
    double imuAngularVelocityStddev_radPerS{0.008};

    /*! IMU acceleration white-noise standard deviation in m/s^2. */
    double imuLinearAccelerationStddev_mPerS2{0.05};

    /*! IMU orientation white-noise standard deviation in rad. */
    double imuOrientationStddev_rad{0.003};

    /*! Wheel joint-position white-noise standard deviation in rad. */
    double wheelPositionStddev_rad{0.002};

    /*! Wheel joint-velocity white-noise standard deviation in rad/s. */
    double wheelVelocityStddev_radPerS{0.015};

    /*! Constant body-frame IMU angular-rate bias in rad/s. */
    std::array<double, AXIS_COUNT> imuAngularVelocityBias_radPerS{};

    /*! Constant body-frame IMU acceleration bias in m/s^2. */
    std::array<double, AXIS_COUNT> imuLinearAccelerationBias_mPerS2{};

    /*! Deterministic random source selected by the Alpha random seed. */
    std::mt19937 randomEngine;

    /*! Serial callback group for IMU and joint sensors. */
    rclcpp::CallbackGroup::SharedPtr p_inertialCallbackGroup;

    /*! Noisy public IMU publisher. */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr p_imuPublisher;

    /*! Raw simulator IMU subscription. */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr p_rawImuSubscription;

    /*! Noisy public joint-state publisher. */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        p_jointStatePublisher;

    /*! Raw simulator joint-state subscription. */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        p_rawJointStateSubscription;
};

} /* namespace lunar_simulator::alpha_system */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    const std::shared_ptr<lunar_simulator::alpha_system::AlphaSensorNode>
        p_alphaSensorNode =
            std::make_shared<lunar_simulator::alpha_system::AlphaSensorNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2U);
    executor.add_node(p_alphaSensorNode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
