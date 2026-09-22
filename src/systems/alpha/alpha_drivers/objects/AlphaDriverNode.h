/*!
 * @file            AlphaDriverNode.h
 *
 * @brief           Declares the node that interfaces Alpha's stack with
 *                   Gazebo: raw sensor reads and raw actuator writes both
 *                   pass through here.
 *
 * @date            17/09/2026
 */

#ifndef LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVER_NODE_H
#define LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVER_NODE_H

/* C++ Standard Library Includes */
#include <array>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

/* External Library Includes */
#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace systems::alpha::alpha_drivers
{

/*!
 * @brief           Models Alpha's simulator hardware boundary in both
 *                   directions.
 *
 * Raw Gazebo measurements remain private under `/alpha/drivers`; noisy
 * measurements are published through Alpha's normal public sensor topics.
 * The public wheel command topic is the only actuator input; it is perturbed
 * the same way before being forwarded to the raw Gazebo actuator bridge, then
 * clamped to Alpha's physical maximum_wheel_speed_radps (see its
 * declare_parameter call). Gazebo applies camera noise natively to avoid a
 * high-bandwidth image relay, so this node never touches camera topics.
 */
class AlphaDriverNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Creates Alpha's hardware-interface node.
     *
     * @throws          std::invalid_argument if a noise parameter is invalid.
     */
    AlphaDriverNode() :
        Node("alpha_driver_node"),
        randomEngine(static_cast<std::mt19937::result_type>(
            declare_parameter<std::int64_t>("random_seed", 7302028)))
    {
        /*!
         * The owning system's namespace is declared first so every topic
         * declared below can default to a name rooted at it (e.g.
         * "/alpha/drivers/imu"); a future second system launched with a
         * different system_name gets its own topic namespace for free.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Global switch: when false, every sample below stays zero. */
        noiseEnabled = declare_parameter<bool>("noise_enabled", true);

        /* Angular-rate white-noise standard deviation, shared by x/y/z. */
        imuAngularVelocityStddev_radPerS =
            declareNonnegativeParameter("imu_angular_velocity_stddev_radps",
                                        0.008);

        /*!
         * Linear-acceleration white-noise standard deviation, shared by
         * x/y/z.
         */
        imuLinearAccelerationStddev_mPerS2 =
            declareNonnegativeParameter("imu_linear_acceleration_stddev_mps2",
                                        0.05);

        /* Small-rotation orientation-noise standard deviation. */
        imuOrientationStddev_rad =
            declareNonnegativeParameter("imu_orientation_stddev_rad", 0.003);

        /* Steering-joint position white-noise standard deviation. */
        wheelPositionStddev_rad =
            declareNonnegativeParameter("wheel_position_stddev_rad", 0.002);

        /* Drive-joint velocity white-noise standard deviation. */
        wheelVelocityStddev_radPerS =
            declareNonnegativeParameter("wheel_velocity_stddev_radps", 0.015);

        /* Bound public encoder traffic independently of Gazebo's physics
         * iteration rate. */
        const double jointStatePublishRateHz = declareNonnegativeParameter(
            "joint_state_publish_rate_hz", 50.0);
        if (!(jointStatePublishRateHz > 0.0) ||
            jointStatePublishRateHz > 1.0e9)
        {
            throw std::invalid_argument(
                "joint_state_publish_rate_hz must be in (0, 1e9]");
        }
        jointStateMinimumPeriod_ns = static_cast<std::int64_t>(
            1.0e9 / jointStatePublishRateHz);

        /* Steering-command position white-noise standard deviation, applied
         * to the outgoing actuator command rather than a sensor reading. */
        wheelCommandPositionStddev_rad =
            declareNonnegativeParameter("wheel_command_position_stddev_rad",
                                        0.001);

        /* Drive-command velocity white-noise standard deviation. */
        wheelCommandVelocityStddev_radPerS =
            declareNonnegativeParameter("wheel_command_velocity_stddev_radps",
                                        0.01);

        /*!
         * Alpha's real-hardware maximum drive-wheel speed (matching the
         * ExoMars rover's own physical limit, ~0.02 m/s at Alpha's wheel
         * radius) -- the second of three layers enforcing this same limit:
         * ackermann_controller proportionally scales a commanded twist down
         * before it ever reaches this node (so this rarely binds), this
         * node clamps the final per-wheel command (catching noise pushing
         * an already-borderline command over the limit, or any other
         * commander that does not itself respect the limit), and
         * alpha_model/model.sdf's drive-joint velocity limits enforce it a
         * third time at the physics level.
         */
        maximumWheelSpeedRadps =
            declareNonnegativeParameter("maximum_wheel_speed_radps", 0.14);

        /* Constant per-axis IMU angular-rate bias. */
        imuAngularVelocityBias_radPerS = declareTripletParameter(
            "imu_angular_velocity_bias_radps",
            std::array<double, AXIS_COUNT>{0.002, -0.001, 0.0015});

        /* Constant per-axis IMU linear-acceleration bias. */
        imuLinearAccelerationBias_mPerS2 = declareTripletParameter(
            "imu_linear_acceleration_bias_mps2",
            std::array<double, AXIS_COUNT>{0.02, -0.015, 0.025});

        /*!
         * One shared, mutually-exclusive group keeps every interface below
         * from ever re-entering the shared random engine concurrently.
         */
        p_noiseCallbackGroup =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        /* Wire up the IMU raw-subscription/noisy-publisher pair. */
        configureImuInterface(systemName);

        /* Wire up the joint-state raw-subscription/noisy-publisher pair. */
        configureJointStateInterface(systemName);

        /* Wire up the wheel-command subscription/raw-publisher pair. */
        configureWheelCommandInterface(systemName);

        /*!
         * Record whether noise is active for operators inspecting the log;
         * the seed itself is fixed by the random_seed parameter.
         */
        RCLCPP_INFO(
            get_logger(),
            "Alpha driver noise %s (seed fixed by random_seed parameter)",
            noiseEnabled ? "enabled" : "disabled");
    }

    /*!
     * @brief       Releases the node's ROS interfaces.
     */
    ~AlphaDriverNode() override = default;

    AlphaDriverNode(const AlphaDriverNode &otherNode_in)            = delete;
    AlphaDriverNode &operator=(const AlphaDriverNode &otherNode_in) = delete;
    AlphaDriverNode(AlphaDriverNode &&otherNode_in)                 = delete;
    AlphaDriverNode &operator=(AlphaDriverNode &&otherNode_in)      = delete;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Adds configured error to one IMU measurement.
     *
     * @param[in]       message_in
     *                  Raw IMU measurement in the sensor frame.
     */
    void publishNoisyImuCallBack(const sensor_msgs::msg::Imu &message_in);

    /*!
     * @brief           Adds configured noise to measured joint states.
     *
     * @param[in]       message_in
     *                  Raw joint measurement.
     */
    void publishNoisyJointStateCallBack(
        const sensor_msgs::msg::JointState &message_in);

    /*!
     * @brief           Adds configured noise to one wheel command and
     *                   forwards it onto the raw actuator bridge topic.
     *
     * @param[in]       message_in
     *                  Command received on the public wheel command topic.
     */
    void publishNoisyWheelCommandCallBack(
        const actuator_msgs::msg::Actuators &message_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Declares and validates one nonnegative parameter.
     *
     * @param[in]       name_in
     *                  Externally controlled ROS parameter name.
     * @param[in]       defaultValue_in
     *                  Default parameter value.
     *
     * @return          Validated parameter value.
     *
     * @throws          std::invalid_argument if the configured value is not
     *                  finite and nonnegative.
     */
    double declareNonnegativeParameter(const std::string &name_in,
                                       double             defaultValue_in);

    /*!
     * @brief           Declares and validates a three-axis parameter.
     *
     * @param[in]       name_in
     *                  Externally controlled ROS parameter name.
     *
     * @param[in]       defaults_in
     *                  Default x, y and z values.
     *
     * @return          Validated x, y and z values.
     *
     * @throws          std::invalid_argument if the configured value does not
     *                  contain exactly three finite values.
     */
    /*!
     * A member function's own declared parameter/return types (unlike its
     * body) are not a complete-class context, so they cannot forward-refer
     * to AXIS_COUNT, which this class declares later under PRIVATE MEMBERS;
     * the literal 3U below is that same value spelled out instead.
     */
    std::array<double, 3U>
        declareTripletParameter(const std::string            &name_in,
                                const std::array<double, 3U> &defaults_in);

    /*!
     * @brief           Configures the raw and public IMU topics.
     *
     * @param[in]       systemName_in
     *                  Owning system's namespace, used to build this node's
     *                  default topic names.
     */
    void configureImuInterface(const std::string &systemName_in);

    /*!
     * @brief           Configures the raw and public joint-state topics.
     *
     * @param[in]       systemName_in
     *                  Owning system's namespace, used to build this node's
     *                  default topic names.
     */
    void configureJointStateInterface(const std::string &systemName_in);

    /*!
     * @brief           Configures the public wheel-command subscription and
     *                   the raw actuator bridge publisher.
     *
     * @param[in]       systemName_in
     *                  Owning system's namespace, used to build this node's
     *                  default topic names.
     */
    void configureWheelCommandInterface(const std::string &systemName_in);

    /*!
     * @brief           Samples zero-mean Gaussian noise.
     *
     * @param[in]       standardDeviation_in
     *                  Standard deviation in the caller's physical unit.
     *
     * @return          One noise sample, or exactly zero when noise is
     *                  disabled or the standard deviation is zero.
     */
    double sampleGaussian(double standardDeviation_in);

    /*!
     * @brief           Adds a small body-frame rotation to an orientation.
     *
     * @param[in,out]   orientation_inout
     *                  IMU quaternion rotated in place.
     */
    void perturbOrientation(geometry_msgs::msg::Quaternion &orientation_inout);

    /*!
     * @brief           Adds a variance to each covariance diagonal.
     *
     * @param[in]       variance_in
     *                  Added variance in the measurement's squared unit.
     *
     * @param[in,out]   covariance_inout
     *                  ROS three-axis covariance matrix.
     */
    static void
        addVarianceToCovariance(double                  variance_in,
                                std::array<double, 9U> &covariance_inout);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Number of spatial axes in an IMU vector.
     */
    static constexpr std::size_t AXIS_COUNT = 3U;

    /*!
     * @brief       Index of the x axis.
     */
    static constexpr std::size_t X_AXIS = 0U;

    /*!
     * @brief       Index of the y axis.
     */
    static constexpr std::size_t Y_AXIS = 1U;

    /*!
     * @brief       Index of the z axis.
     */
    static constexpr std::size_t Z_AXIS = 2U;

    /*!
     * @brief       Whether configured stochastic errors are applied.
     */
    bool noiseEnabled{true};

    /*!
     * @brief       IMU angular-rate white-noise standard deviation in rad/s.
     */
    double imuAngularVelocityStddev_radPerS{0.008};

    /*!
     * @brief       IMU acceleration white-noise standard deviation in m/s^2.
     */
    double imuLinearAccelerationStddev_mPerS2{0.05};

    /*!
     * @brief       IMU orientation white-noise standard deviation in rad.
     */
    double imuOrientationStddev_rad{0.003};

    /*!
     * @brief       Wheel joint-position white-noise standard deviation in rad.
     */
    double wheelPositionStddev_rad{0.002};

    /*!
     * @brief       Wheel joint-velocity white-noise standard deviation in
     *              rad/s.
     */
    double wheelVelocityStddev_radPerS{0.015};

    /*! @brief Minimum simulation-time interval between public joint states. */
    std::int64_t jointStateMinimumPeriod_ns{20000000};

    /*! @brief Timestamp of the most recently published joint state. */
    std::int64_t previousJointStateStamp_ns{0};

    /*! @brief Whether a public joint-state timestamp has been recorded. */
    bool hasPreviousJointStateStamp{false};

    /*!
     * @brief       Wheel steering-command white-noise standard deviation in
     *              rad, applied to the outgoing actuator command.
     */
    double wheelCommandPositionStddev_rad{0.001};

    /*!
     * @brief       Wheel drive-command white-noise standard deviation in
     *              rad/s, applied to the outgoing actuator command.
     */
    double wheelCommandVelocityStddev_radPerS{0.01};

    /*!
     * @brief       Alpha's real-hardware maximum drive-wheel speed in
     *              rad/s; see its declare_parameter call for the three
     *              layers this is enforced at.
     */
    double maximumWheelSpeedRadps{0.14};

    /*!
     * @brief       Constant body-frame IMU angular-rate bias in rad/s.
     */
    std::array<double, AXIS_COUNT> imuAngularVelocityBias_radPerS{};

    /*!
     * @brief       Constant body-frame IMU acceleration bias in m/s^2.
     */
    std::array<double, AXIS_COUNT> imuLinearAccelerationBias_mPerS2{};

    /*!
     * @brief       Deterministic random source selected by the Alpha random
     *              seed.
     */
    std::mt19937 randomEngine;

    /*!
     * @brief       Serial callback group shared by every interface below, so
     *              none of them can re-enter randomEngine concurrently.
     */
    rclcpp::CallbackGroup::SharedPtr p_noiseCallbackGroup;

    /*!
     * @brief       Noisy public IMU publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr p_imuPublisher;

    /*!
     * @brief       Raw simulator IMU subscription.
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr p_rawImuSubscription;

    /*!
     * @brief       Noisy public joint-state publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        p_jointStatePublisher;

    /*!
     * @brief       Raw simulator joint-state subscription.
     */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        p_rawJointStateSubscription;

    /*!
     * @brief       Raw Gazebo actuator bridge publisher.
     */
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
        p_rawWheelCommandPublisher;

    /*!
     * @brief       Public wheel command subscription.
     */
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr
        p_wheelCommandSubscription;
};

} /* namespace systems::alpha::alpha_drivers */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVER_NODE_H */
