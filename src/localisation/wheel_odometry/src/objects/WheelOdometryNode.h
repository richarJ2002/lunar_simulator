/*!
 * @File:         WheelOdometryNode.h
 *
 * @Brief:        Declares the slip-adjusted six-wheel steering odometry node.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_NODE_H
#define LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_NODE_H

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
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace localisation::wheel_odometry
{

/*!
 * @brief           Fuses six independently-steered wheel encoders into a
 *                  slip-adjusted planar body twist and integrates it into an
 *                  odometry pose.
 *
 * Wheel order is fixed throughout this node and its configuration
 * (`slip_ratios`, `drive_direction_multipliers`, `wheel_x_m`, `wheel_y_m`) as
 * front-left, front-right, centre-left, centre-right, rear-left, rear-right.
 * All wheel positions are expressed in the rover body frame in metres, and
 * all wheel and body rates are expressed in the body frame. The rolling
 * constraint for wheel i with steering angle delta_i, body-frame planar
 * velocity (vx, vy) and yaw rate wz is:
 *
 *   cos(delta_i) * vx + sin(delta_i) * vy
 *     + (-y_i * cos(delta_i) + x_i * sin(delta_i)) * wz = r * omega_i
 *
 * where r is the common wheel radius in metres and omega_i is that wheel's
 * slip-adjusted angular rate in rad/s. The six such constraints are solved
 * in a minimum-norm least-squares sense every joint-state callback.
 *
 * The slip ratio applied above is not estimated locally: this node computes
 * one raw slip *observation* per wheel per cycle (that wheel's own raw
 * speed compared against the wheel-projection of the latest
 * visual-odometry body twist for that wheel specifically, NaN for a wheel
 * that fails this cycle's gating -- a single shared value cannot represent
 * rotational slip, where inner/outer wheels genuinely slip by different
 * amounts during a turn) and publishes all six, in wheel order, on
 * slip_observation_topic for continuous_ekf (alpha_kalman_filter) to fuse
 * as six of its own independent EKF states alongside pose and twist. This
 * node then reads back whatever continuous_ekf last published on
 * wheel_slip_estimate_topic and applies each wheel's own fused value to
 * that same wheel before solving the rolling-constraint system above --
 * Kalman-weighted fusion (accounting for how much the EKF currently trusts
 * each wheel's own observation) replaces what used to be this node's own
 * fixed-gain exponential blend and decay. The node is intended for a
 * single-threaded executor.
 */
class WheelOdometryNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters, validates them and subscribes to
     *                  joint-state and visual-odometry input topics.
     *
     * @throws          std::invalid_argument if a wheel, slip or visual-slip
     *                  parameter lies outside its valid range.
     */
    WheelOdometryNode() : Node("wheel_odometry")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: joint states carrying every wheel's drive/steer
         * position and rate. */
        const std::string inputTopic = declare_parameter<std::string>(
            "joint_state_topic", "/" + systemName + "/joint_states");

        /* Output topic: the integrated slip-adjusted odometry pose. */
        const std::string outputTopic = declare_parameter<std::string>(
            "odometry_topic",
            "/" + systemName + "/localisation/wheel/odometry");

        /* Input topic: visual odometry, used as the slip-estimation
         * reference. */
        const std::string visualOdometryTopic = declare_parameter<std::string>(
            "visual_odometry_topic",
            "/" + systemName + "/localisation/visual/odometry");

        /* Output topic: the current shared per-wheel slip ratio, as
         * applied to the rolling-constraint solve. */
        const std::string slipRatioTopic = declare_parameter<std::string>(
            "slip_ratio_topic",
            "/" + systemName + "/localisation/wheel/slip_ratios");

        /* Output topic: this node's raw per-cycle slip observation, fused
         * by continuous_ekf rather than blended locally. */
        const std::string slipObservationTopic = declare_parameter<
            std::string>(
            "slip_observation_topic",
            "/" + systemName + "/localisation/wheel/slip_observation");

        /* Input topic: continuous_ekf's fused slip estimate, applied to
         * all six wheels below. */
        const std::string wheelSlipEstimateTopic = declare_parameter<
            std::string>(
            "wheel_slip_estimate_topic",
            "/" + systemName + "/localisation/kalman_filter/wheel_slip_ratio");

        /*!
         * Input topic: continuous_ekf's fused pose/twist estimate, read
         * only for its roll/pitch (see handleKalmanFilterCallBack()) so
         * each integration step can be projected through the rover's
         * actual current tilt instead of assuming a level body frame.
         */
        const std::string kalmanFilterOdometryTopic = declare_parameter<
            std::string>(
            "kalman_filter_odometry_topic",
            "/" + systemName + "/localisation/kalman_filter/odometry");

        /*!
         * Input topic: ground_truth's own settled resting pose, published
         * once, latched (see GroundTruthNode::handleOdometryCallBack()).
         * Seeds this node's own integrated pose (position, yaw) and the
         * live roll/pitch cache above (see handleInitialPoseCallBack())
         * instead of a manually re-measured, terrain-specific constant.
         * Subscribed independently of kalmanFilterOdometryTopic above
         * (rather than waiting for continuous_ekf's own first fused
         * output) to avoid a circular startup dependency: continuous_ekf's
         * own seeding can itself depend on this node's odometry arriving
         * first.
         */
        const std::string initialPoseTopic = declare_parameter<std::string>(
            "initial_pose_topic",
            "/" + systemName + "/localisation/ground_truth/initial_pose");

        /* Fixed world frame in which the integrated pose is published. */
        odomFrame = declare_parameter<std::string>("odom_frame", "map");

        /* Child body frame of the integrated pose and twist. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Common wheel radius used to convert drive rate to speed. */
        wheelRadiusM = declare_parameter<double>("wheel_radius_m", 0.1425);

        /* Largest joint-state gap still integrated as continuous motion. */
        maximumIntegrationDtS =
            declare_parameter<double>("maximum_integration_dt_s", 5.0);

        /* Whether visual odometry is used to produce a slip observation at
         * all. */
        shouldEstimateSlip =
            declare_parameter<bool>("estimate_slip_from_visual", true);

        /* Slip ratios below this threshold are treated as zero. */
        slipRatioDeadband =
            declare_parameter<double>("slip_ratio_deadband", 0.05);

        /* Wheel speeds below this magnitude are excluded from slip
         * observation. */
        minimumWheelSpeedMps =
            declare_parameter<double>("minimum_wheel_speed_mps", 0.005);

        /* Maximum age of a visual-odometry reference still usable for
         * slip estimation. */
        visualOdometryTimeoutS =
            declare_parameter<double>("visual_odometry_timeout_s", 0.25);

        /* Visual-odometry twist variance above this is untrustworthy. */
        maximumVisualTwistVariance =
            declare_parameter<double>("maximum_visual_twist_variance", 0.08);

        /* Upper bound applied to any slip ratio. */
        maximumSlipRatio =
            declare_parameter<double>("maximum_slip_ratio", 0.30);

        /*!
         * Minimum singular value of the six-wheel rolling-constraint
         * matrix still treated as observing lateral (vy) motion; see
         * handleJointStateCallBack()'s use of this value below for why it
         * must exceed encoder-noise-driven steering angles rather than
         * just numerical zero.
         */
        lateralObservabilityThreshold = declare_parameter<double>(
            "lateral_observability_threshold", 0.02);

        /* Reject a configuration whose wheel or slip parameters could
         * never produce a physically meaningful result. */
        if (wheelRadiusM <= 0.0 || maximumIntegrationDtS <= 0.0 ||
            minimumWheelSpeedMps < 0.0 ||
            visualOdometryTimeoutS <= 0.0 || maximumVisualTwistVariance < 0.0 ||
            maximumSlipRatio < 0.0 || maximumSlipRatio > 0.99 ||
            slipRatioDeadband < 0.0 || slipRatioDeadband >= 1.0 ||
            lateralObservabilityThreshold <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "Wheel and visual-slip parameters are outside valid bounds");
        }

        /*!
         * Six-value parameters are loaded (and range-clamped, for slip
         * ratios) before the joint name tables so that an invalid override
         * falls back to the physically-measured Alpha defaults rather than
         * leaving the node in a partially configured state.
         */
        loadSixValues("slip_ratios", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      slipRatios);

        /* Load each wheel's fixed drive-direction sign convention. */
        loadSixValues("drive_direction_multipliers",
                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                      driveDirectionMultipliers);

        /* Load each wheel's fixed body-frame x position. */
        loadSixValues("wheel_x_m", {0.64, 0.64, 0.0, 0.0, -0.72, -0.72},
                      wheelXM);

        /* Load each wheel's fixed body-frame y position. */
        loadSixValues("wheel_y_m", {0.60, -0.60, 0.60, -0.60, 0.60, -0.60},
                      wheelYM);

        /* Clamp every configured slip ratio into its valid range. */
        for (double &slipRatio : slipRatios)
        {
            /* A slip ratio outside [0, maximumSlipRatio) is not
             * physically meaningful; clamp it into range. */
            slipRatio = std::clamp(slipRatio, 0.0, maximumSlipRatio);
        }

        /* Fixed drive-joint names in front-left..rear-right order. */
        driveJointNames = {
            "alpha/front_left_drive_joint",  "alpha/front_right_drive_joint",
            "alpha/centre_left_drive_joint", "alpha/centre_right_drive_joint",
            "alpha/rear_left_drive_joint",   "alpha/rear_right_drive_joint"};

        /* Fixed steering-joint names in front-left..rear-right order. */
        steerJointNames = {
            "alpha/front_left_steer_joint",  "alpha/front_right_steer_joint",
            "alpha/centre_left_steer_joint", "alpha/centre_right_steer_joint",
            "alpha/rear_left_steer_joint",   "alpha/rear_right_steer_joint"};

        /* Publisher for the integrated slip-adjusted odometry pose. */
        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputTopic, rclcpp::QoS(10));

        /* Latch the slip-ratio topic so a newly opened subscriber sees
         * the current value immediately. */
        slipRatioPublisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            slipRatioTopic, rclcpp::QoS(1).reliable().transient_local());

        /* Publisher for this node's raw per-cycle per-wheel slip
         * observations, consumed by continuous_ekf. */
        slipObservationPublisher =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                slipObservationTopic, rclcpp::SensorDataQoS());

        /* Every incoming joint-state message triggers
         * handleJointStateCallBack(). */
        jointSubscription = create_subscription<sensor_msgs::msg::JointState>(
            inputTopic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::ConstSharedPtr p_message)
            { handleJointStateCallBack(*p_message); });

        /* Every incoming visual-odometry message triggers
         * handleVisualOdometryCallBack(). */
        visualOdometrySubscription =
            create_subscription<nav_msgs::msg::Odometry>(
                visualOdometryTopic, rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
                { handleVisualOdometryCallBack(*p_message); });

        /* Every fused per-wheel slip estimate from continuous_ekf triggers
         * handleWheelSlipEstimateCallBack(). */
        wheelSlipEstimateSubscription =
            create_subscription<std_msgs::msg::Float64MultiArray>(
                wheelSlipEstimateTopic, rclcpp::SensorDataQoS(),
                [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr p_message)
                { handleWheelSlipEstimateCallBack(*p_message); });

        /* Every fused estimate from continuous_ekf refreshes the live
         * roll/pitch cache; matches continuous_ekf's own output QoS
         * (rclcpp::QoS(10)) rather than SensorDataQoS since that is what
         * AlphaKalmanFilterNode actually publishes with. */
        kalmanFilterSubscription = create_subscription<nav_msgs::msg::Odometry>(
            kalmanFilterOdometryTopic, rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleKalmanFilterCallBack(*p_message); });

        /* ground_truth's one-time settled resting pose; latched, so this
         * arrives regardless of subscribe order relative to when
         * ground_truth publishes it. */
        initialPoseSubscription = create_subscription<nav_msgs::msg::Odometry>(
            initialPoseTopic, rclcpp::QoS(1).reliable().transient_local(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleInitialPoseCallBack(*p_message); });

        /* Record the resolved topic names once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(),
                    "Wheel odometry: %s -> %s; visual slip reference: %s -> %s",
                    inputTopic.c_str(), outputTopic.c_str(),
                    visualOdometryTopic.c_str(), slipRatioTopic.c_str());
    }

    /*! @brief Releases the node without external side effects. */
    ~WheelOdometryNode() override = default;

    WheelOdometryNode(const WheelOdometryNode &otherNode_in) = delete;
    WheelOdometryNode &operator=(
        const WheelOdometryNode &otherNode_in) = delete;
    WheelOdometryNode(WheelOdometryNode &&otherNode_in) = delete;
    WheelOdometryNode &operator=(WheelOdometryNode &&otherNode_in) = delete;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Records the latest visual-odometry body twist as the
     *                  slip-estimation reference when it is trustworthy.
     *
     * @param[in]       message_in
     *                  Visual-odometry estimate. Only the planar body
     *                  twist (linear x, y and angular z) and its diagonal
     *                  twist covariance are used.
     */
    void handleVisualOdometryCallBack(const nav_msgs::msg::Odometry &message_in);

    /*!
     * @brief           Solves the six-wheel rolling-constraint system for
     *                  the current body twist and integrates it into the
     *                  odometry pose.
     *
     * @param[in]       message_in
     *                  Joint-state message containing all six drive and
     *                  steering joints.
     */
    void handleJointStateCallBack(const sensor_msgs::msg::JointState &message_in);

    /*!
     * @brief           Applies continuous_ekf's latest fused per-wheel slip
     *                   estimate, each to its own wheel.
     *
     * @param[in]       message_in
     *                  Fused per-wheel slip ratios, six elements in wheel
     *                  order, each dimensionless; clamped into
     *                  [0, maximumSlipRatio) defensively before use.
     */
    void handleWheelSlipEstimateCallBack(
        const std_msgs::msg::Float64MultiArray &message_in);

    /*!
     * @brief           Refreshes the live roll/pitch cache from
     *                   continuous_ekf's latest fused estimate.
     *
     * This node cannot sense roll or pitch itself; it only borrows
     * continuous_ekf's best current estimate of them to project each
     * integration step (see handleJointStateCallBack()) through the
     * rover's actual current tilt instead of assuming a level body frame.
     *
     * @param[in]       message_in
     *                  continuous_ekf's fused pose/twist estimate; only
     *                  its orientation is used.
     */
    void handleKalmanFilterCallBack(const nav_msgs::msg::Odometry &message_in);

    /*!
     * @brief           Seeds this node's integrated pose and live
     *                   roll/pitch cache from ground_truth's one-time
     *                   settled resting pose.
     *
     * Fires exactly once in practice (ground_truth only ever publishes one
     * message on this topic), but is not itself latched against being
     * called again -- harmless, since it would simply re-derive the same
     * value from the same retained message. handleJointStateCallBack()
     * withholds integration and publication entirely until this has run,
     * so continuous_ekf can never fuse a not-yet-seeded (zero) wheel pose.
     *
     * @param[in]       message_in
     *                  ground_truth's settled resting pose, in the map
     *                  frame.
     */
    void handleInitialPoseCallBack(const nav_msgs::msg::Odometry &message_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Declares a six-element parameter, falling back to the
     *                  provided defaults when the configured value does not
     *                  contain exactly six entries.
     *
     * @param[in]       name_in
     *                  Externally controlled ROS parameter name.
     * @param[in]       defaults_in
     *                  Default six values used verbatim when validation
     *                  fails.
     * @param[out]      values_out
     *                  Six validated (or default) values in wheel order.
     */
    void loadSixValues(const std::string &name_in,
                       const std::vector<double> &defaults_in,
                       std::array<double, 6> &values_out);

    /*!
     * @brief           Converts a ROS timestamp to seconds.
     *
     * @param[in]       stamp_in
     *                  Timestamp to convert.
     *
     * @return          Timestamp in seconds.
     */
    static double
    stampToSeconds(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Finds one named joint's position and velocity in a
     *                  joint-state message.
     *
     * @param[in]       message_in
     *                  Joint-state message to search.
     * @param[in]       name_in
     *                  Exact joint name to find.
     * @param[out]      position_out
     *                  Joint position in the message's unit when found.
     * @param[out]      velocity_out
     *                  Joint velocity in the message's unit when found.
     *
     * @return          True when the named joint was found with both a
     *                  position and a velocity entry.
     */
    static bool findJoint(const sensor_msgs::msg::JointState &message_in,
                          const std::string &name_in, double &position_out,
                          double &velocity_out);

    /*!
     * @brief           Computes and publishes one raw per-wheel slip
     *                  observation from the latest visual-odometry
     *                  reference, when one is available and fresh enough
     *                  to trust.
     *
     * The per-wheel gating here decides whether/what to observe for each
     * wheel independently (NaN for a wheel that fails it this cycle);
     * fusing each wheel's observation over time into an authoritative
     * estimate is continuous_ekf's job (see
     * handleWheelSlipEstimateCallBack()), not this node's.
     *
     * @param[in]       jointStampS_in
     *                  Timestamp in seconds of the joint-state message that
     *                  triggered this update.
     * @param[in]       rawWheelSpeedMps_in
     *                  Un-slip-adjusted per-wheel circumferential speed in
     *                  m/s, in wheel order.
     * @param[in]       steerAngleRad_in
     *                  Per-wheel steering angle in radians, in wheel order.
     */
    void publishSlipObservation(double jointStampS_in,
                                const std::array<double, 6> &rawWheelSpeedMps_in,
                                const std::array<double, 6> &steerAngleRad_in);

    /*! @brief Publishes the current six per-wheel slip ratios. */
    void publishSlipRatios();

    /*!
     * @brief           Wraps an angle to the range [-pi, pi].
     *
     * @param[in]       angleRad_in
     *                  Angle in radians.
     *
     * @return          Wrapped angle in radians.
     */
    static double wrapAngle(double angleRad_in);

    /*!
     * @brief           Publishes the integrated pose and current body twist
     *                  as one odometry message.
     *
     * @param[in]       stamp_in
     *                  Timestamp applied to the published message, matching
     *                  the triggering joint-state message.
     * @param[in]       bodyTwist_in
     *                  Slip-adjusted planar body twist (vx, vy, wz) in the
     *                  body frame, in m/s and rad/s.
     */
    void publishOdometry(const builtin_interfaces::msg::Time &stamp_in,
                         const Eigen::Vector3d &bodyTwist_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Publishes integrated wheel odometry.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

    /*!
     * @brief       Receives raw drive and steering joint states.
     */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        jointSubscription;

    /*!
     * @brief       Receives visual odometry used only as a slip-estimation
     *              reference.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        visualOdometrySubscription;

    /*!
     * @brief       Publishes the six independent per-wheel slip-ratio
     *              estimates currently applied.
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        slipRatioPublisher;

    /*!
     * @brief       Publishes this node's raw per-cycle per-wheel slip
     *              observations for continuous_ekf to fuse.
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        slipObservationPublisher;

    /*!
     * @brief       Receives continuous_ekf's fused per-wheel slip
     *              estimate.
     */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        wheelSlipEstimateSubscription;

    /*!
     * @brief       Receives continuous_ekf's fused pose/twist estimate,
     *              read only for its roll/pitch.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        kalmanFilterSubscription;

    /*!
     * @brief       Subscribes to ground_truth's one-time settled resting
     *              pose.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        initialPoseSubscription;

    /*!
     * @brief       Drive-joint names in front-left..rear-right wheel order.
     */
    std::array<std::string, 6> driveJointNames;

    /*!
     * @brief       Steering-joint names in front-left..rear-right wheel order.
     */
    std::array<std::string, 6> steerJointNames;

    /*!
     * @brief       Slip ratio currently applied to all six wheels,
     *              dimensionless in [0, 1). Sourced from continuous_ekf's
     *              fused estimate (see handleWheelSlipEstimateCallBack());
     *              starts from the configured slip_ratios default until the
     *              first fused estimate arrives.
     */
    std::array<double, 6> slipRatios{};

    /*!
     * @brief       Per-wheel drive-direction sign convention, dimensionless.
     */
    std::array<double, 6> driveDirectionMultipliers{};

    /*!
     * @brief       Per-wheel body-frame x position in metres.
     */
    std::array<double, 6> wheelXM{};

    /*!
     * @brief       Per-wheel body-frame y position in metres.
     */
    std::array<double, 6> wheelYM{};

    /*!
     * @brief       Frame in which the integrated pose is expressed.
     */
    std::string odomFrame;

    /*!
     * @brief       Child frame of the integrated pose and twist.
     */
    std::string baseFrame;

    /*!
     * @brief       Common wheel radius in metres.
     */
    double wheelRadiusM{0.1425};

    /*!
     * @brief       Largest joint-state time gap integrated as continuous
     *              motion, in seconds.
     */
    double maximumIntegrationDtS{5.0};

    /*!
     * @brief       Observed slip ratios below this dimensionless threshold
     *              are treated as zero slip.
     */
    double slipRatioDeadband{0.05};

    /*!
     * @brief       Wheel speeds below this magnitude in m/s are excluded
     *              from slip observation because their sign and ratio are
     *              unreliable near zero.
     */
    double minimumWheelSpeedMps{0.005};

    /*!
     * @brief       Maximum age in seconds of a visual-odometry reference
     *              that may still be used for slip estimation.
     */
    double visualOdometryTimeoutS{0.25};

    /*!
     * @brief       Visual-odometry twist variance above this threshold is
     *              treated as untrustworthy and excluded from slip
     *              estimation.
     */
    double maximumVisualTwistVariance{0.08};

    /*!
     * @brief       Upper bound applied to any slip ratio, dimensionless in
     *              [0, 1). Always overwritten by the `maximum_slip_ratio`
     *              parameter (default 0.30) in the constructor; this
     *              in-class initializer only matters if a future code path
     *              reads it before that parameter is declared.
     */
    double maximumSlipRatio{0.30};

    /*!
     * @brief       Minimum singular value of the rolling-constraint matrix
     *              still treated as observing lateral (vy) motion,
     *              dimensionless (same units as the matrix's cos/sin
     *              entries). Below this, vy is set to zero instead of
     *              amplifying steering-encoder noise; see
     *              handleJointStateCallBack().
     */
    double lateralObservabilityThreshold{0.02};

    /*!
     * @brief       Timestamp in seconds of the latest accepted
     *              visual-odometry reference.
     */
    double latestVisualStampS{0.0};

    /*!
     * @brief       Timestamp in seconds of the visual-odometry reference
     *              last used to update the slip ratio; -1 before the first
     *              update.
     */
    double lastSlipUpdateStampS{-1.0};

    /*!
     * @brief       Latest accepted visual-odometry body twist (vx, vy, wz)
     *              in the body frame, m/s and rad/s.
     */
    Eigen::Vector3d latestVisualTwistBody{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Whether visual-odometry-based slip estimation is enabled.
     */
    bool shouldEstimateSlip{true};

    /*!
     * @brief       True once at least one visual-odometry reference has been
     *              accepted.
     */
    bool hasVisualReference{false};

    /*!
     * @brief       True once the slip-ratio topic has published at least once.
     */
    bool hasPublishedSlipRatios{false};

    /*!
     * @brief       True once at least one joint-state message has been
     *              integrated.
     */
    bool hasPreviousStamp{false};

    /*!
     * @brief       Timestamp in seconds of the previously integrated
     *              joint-state message.
     */
    double previousStampS{0.0};

    /*!
     * @brief       Integrated rover position x in the odom frame, in metres.
     */
    double positionXM{0.0};

    /*!
     * @brief       Integrated rover position y in the odom frame, in metres.
     */
    double positionYM{0.0};

    /*!
     * @brief       Integrated rover position z in the odom frame, in
     *              metres. Unlike x/y, this is projected through the live
     *              roll/pitch cache below at every integration step (see
     *              handleJointStateCallBack()), not assumed level.
     */
    double positionZM{0.0};

    /*!
     * @brief       Integrated rover yaw in the odom frame, in radians.
     */
    double yawRad{0.0};

    /*!
     * @brief       Live roll estimate borrowed from continuous_ekf's
     *              latest fused output (see handleKalmanFilterCallBack()),
     *              radians. This node cannot sense roll itself; seeded
     *              from ground_truth's one-time initial pose (see
     *              handleInitialPoseCallBack()) until the first fused
     *              estimate arrives.
     */
    double latestRollRad{0.0};

    /*!
     * @brief       Live pitch estimate borrowed from continuous_ekf's
     *              latest fused output, radians, same caveats as
     *              latestRollRad above.
     */
    double latestPitchRad{0.0};

    /*!
     * @brief       True once positionXM/YM/ZM, yawRad, latestRollRad and
     *              latestPitchRad have been seeded from ground_truth's
     *              one-time initial pose (see handleInitialPoseCallBack()).
     *              handleJointStateCallBack() withholds integration and
     *              publication entirely until this is true.
     */
    bool hasReceivedInitialPose{false};
};

} /* namespace localisation::wheel_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_NODE_H */
