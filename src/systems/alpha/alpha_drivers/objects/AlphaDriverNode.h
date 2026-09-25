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
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

/* External Library Includes */
#include <actuator_msgs/msg/actuators.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/* Other Project Module Includes */
#include "alpha_supervisor/public_functions.h"
#include "console/console.h"

/* Object Includes */
#include "alpha_supervisor/objects/SystemState.h"
#include "objects/CommandGateDecision.h"

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
 *
 * Every public command first passes the command gate: it is forwarded only
 * while the start-up supervisor's latched system state is READY and its
 * heartbeat is fresh (see evaluateCommandGate()). Anything else is dropped,
 * counted and never replayed, and a gate that closes after being open sends
 * one zero-velocity command. Because this node is the only path from the
 * public command topic to the raw actuator bridge, the gate covers both the
 * Ackermann controller and direct wheel commands; something publishing
 * straight to the private raw topic still bypasses it.
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

        /* Wire up the system-state gate and readiness diagnostics. */
        configureCommandGate(systemName);

        /*!
         * Record whether noise is active for operators inspecting the log;
         * the seed itself is fixed by the random_seed parameter.
         */
        LUNAR_LOG_INFO(get_logger(),
                       "Driver noise %s",
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

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Decides whether wheel commands may reach the
     *                  actuators.
     *
     *                  The gate is open only when a system state has been
     *                  received, it is READY, and it is no older than the
     *                  heartbeat limit. The function is pure and
     *                  deterministic.
     *
     * @param[in]       systemState_in
     *                  Latest received system state; no value when none
     *                  has been received or it could not be parsed.
     *
     * @param[in]       heartbeatAge_s_in
     *                  Time since that state was received, seconds.
     *
     * @param[in]       maximumHeartbeatAge_s_in
     *                  Oldest state that still opens the gate, seconds.
     *
     * @return          The decision and, when closed, a short reason.
     */
    static CommandGateDecision evaluateCommandGate(
        const std::optional<alpha_supervisor::SystemState> &systemState_in,
        double                                              heartbeatAge_s_in,
        double maximumHeartbeatAge_s_in);

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Records the latest system state and its receipt
     *                  time.
     *
     * @param[in]       message_in
     *                  Latched system state; only its "system_state"
     *                  status's "state" value is used.
     */
    void handleSystemStateCallBack(
        const diagnostic_msgs::msg::DiagnosticArray &message_in);

    /*!
     * @brief           Re-evaluates the gate between commands and sends one
     *                  zero-velocity command when an open gate closes.
     *
     *                  Runs every 100 ms of simulation time in the shared
     *                  serial callback group, so it never interleaves with
     *                  publishNoisyWheelCommandCallBack().
     */
    void enforceCommandGateCallBack();

    /*!
     * @brief           Publishes this node's readiness and gate counters
     *                  on the diagnostics topic.
     *
     *                  Runs once per simulated second in the shared serial
     *                  callback group.
     */
    void publishDiagnosticsCallBack();

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
     * @brief           Declares the gate and readiness parameters and wires
     *                  the system-state subscription, the diagnostics
     *                  publisher and both gate timers.
     *
     * @param[in]       systemName_in
     *                  Owning system's namespace, used to build this node's
     *                  default topic names.
     *
     * @throws          std::invalid_argument if a time limit is not
     *                  positive.
     */
    void configureCommandGate(const std::string &systemName_in);

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

    /*!
     * @brief           Minimum simulation-time interval between public
     *                  joint states.
     *
     * @frame           N/A
     * @units           nanoseconds
     */
    std::int64_t jointStateMinimumPeriod_ns{20000000};

    /*!
     * @brief           Timestamp of the most recently published joint state.
     *
     * @frame           N/A
     * @units           nanoseconds
     */
    std::int64_t previousJointStateStamp_ns{0};

    /*!
     * @brief           Whether a public joint-state timestamp has been
     *                  recorded.
     *
     * @frame           N/A
     * @units           N/A
     */
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

    /*!
     * @brief           Receives the supervisor's latched system state.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_systemStateSubscription;

    /*!
     * @brief           Publishes this node's readiness and gate counters.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsPublisher;

    /*!
     * @brief           Simulation-time timer driving
     *                  enforceCommandGateCallBack().
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_gateTimer;

    /*!
     * @brief           Simulation-time timer driving
     *                  publishDiagnosticsCallBack().
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;

    /*!
     * @brief           Oldest system state that still opens the gate.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumStateHeartbeatAgeS{1.5};

    /*!
     * @brief           Oldest raw IMU or joint-state sample for which this
     *                  node reports itself ready.
     *
     * @frame           N/A
     * @units           seconds
     */
    double readinessMaximumInputAgeS{0.5};

    /*!
     * @brief           Latest parsed system state; no value before the
     *                  first one or after an unparsable one.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::optional<alpha_supervisor::SystemState> latestSystemState;

    /*!
     * @brief           Receipt time of the latest system state.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double latestSystemStateReceipt_s{0.0};

    /*!
     * @brief           Whether the gate was open at its previous
     *                  evaluation, to detect the open-to-closed edge.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool wasGateOpen{false};

    /*!
     * @brief           Steering positions of the latest forwarded command,
     *                  held by the stop command so a stop never re-steers
     *                  the wheels; empty before the first command.
     *
     * @frame           Per-wheel steering joint
     * @units           radians
     */
    std::vector<double> latestForwardedSteering_rad;

    /*!
     * @brief           Number of commands forwarded to the actuators.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t forwardedCommandCount{0U};

    /*!
     * @brief           Number of commands dropped by the closed gate.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t blockedCommandCount{0U};

    /*!
     * @brief           Number of zero-velocity commands sent when the gate
     *                  closed.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t stopCommandCount{0U};

    /*!
     * @brief           Receipt time of the latest raw IMU sample; negative
     *                  before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double latestRawImuReceipt_s{-1.0};

    /*!
     * @brief           Receipt time of the latest raw joint state; negative
     *                  before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double latestRawJointStateReceipt_s{-1.0};
};

} /* namespace systems::alpha::alpha_drivers */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_DRIVER_NODE_H */
