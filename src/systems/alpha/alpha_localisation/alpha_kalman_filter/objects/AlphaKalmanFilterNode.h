/*!
 * @File:         AlphaKalmanFilterNode.h
 *
 * @Brief:        Declares the ROS wrapper that fuses Alpha odometry through
 *                the reusable continuous-discrete EKF engine.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_ALPHA_ALPHA_KALMAN_FILTER_NODE_H
#define LUNAR_SIMULATOR_ALPHA_ALPHA_KALMAN_FILTER_NODE_H

/* Function Includes */
/* None */

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"
#include "objects/FilterStatus.h"
#include "objects/MeasurementKind.h"

/* Data include */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/* Generic Libraries */
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Fuses inertial, visual and wheel odometry, plus six
 *                  independent per-wheel slip observations, into one
 *                  eighteen-state pose/twist/slip estimate using the
 *                  reusable, model-agnostic continuous EKF engine, and
 *                  exposes the result as odometry, a retained path, TF, and
 *                  a fused per-wheel slip ratio.
 *
 * This node owns Alpha's specific eighteen-state model -- the process model
 * (see computeProcessModel()), each source's observation model (see
 * handleMeasurementCallBack() and handleSlipObservationCallBack()) and the
 * state layout itself ([position(3), roll/pitch/yaw(3), linear_velocity(3),
 * angular_rate(3), wheel_slip_ratio(6), front-left..rear-right wheel order
 * matching WheelOdometryNode's own]) -- and evaluates that model at the
 * engine's current state (Filter::getState()) to build the matrices the
 * engine's predict()/update() take. The engine itself knows none of this: it
 * only executes the linear algebra. The node subscribes to three
 * independently produced odometry topics (inertial_odometry, visual_odometry,
 * wheel_odometry), maps each into this state representation, and applies a
 * measurement-specific observation matrix and variance so each source only
 * corrects the states it can actually observe. It also subscribes to
 * wheel_odometry's raw per-cycle per-wheel slip observations (each wheel's
 * own raw speed compared against visual-odometry-projected rolling speed for
 * that wheel specifically, NaN for a wheel not observable this cycle -- see
 * WheelOdometryNode::publishSlipObservation()) and fuses whichever wheels
 * were observed directly as their own states, six independent
 * mean-reverting-toward-zero-absent-fresh-evidence states rather than one
 * shared value (a single shared ratio cannot represent rotational slip,
 * where inner/outer wheels genuinely slip by different amounts during a
 * turn -- see computeProcessModel()), then republishes the fused six-wheel
 * result on wheel_slip_estimate_topic for wheel_odometry to read back and
 * apply per-wheel to its own rolling-constraint solve -- replacing
 * wheel_odometry's own local ad hoc exponential blend and stale-decay with
 * genuine Kalman-weighted fusion. A wall timer
 * separately drives continuous prediction between measurements at
 * prediction_rate_hz. Every fused pose/twist estimate is published on
 * output_topic as nav_msgs::msg::Odometry, then validated and
 * unconditionally broadcast as the odom_frame -> base_frame transform and
 * appended to a retained, rate-limited path on estimated_path_topic (see
 * publishEstimatedPath()). This node owns the ContinuousExtendedKalmanFilter
 * instance exclusively and is intended for a single-threaded executor; it is
 * not thread-safe.
 */
class AlphaKalmanFilterNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters and wires the odometry
     *                  subscriptions, output publisher, and prediction
     *                  timer.
     *
     * @throws          std::invalid_argument if maximum_visual_measurement_age_s
     *                  is not positive.
     */
    AlphaKalmanFilterNode() : Node("continuous_ekf")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: inertial_odometry's odometry output. */
        const std::string inertialTopic = declare_parameter<std::string>(
            "inertial_odometry_topic",
            "/" + systemName + "/localisation/inertial/odometry");

        /* Input topic: visual_odometry's odometry output. */
        const std::string visualTopic = declare_parameter<std::string>(
            "visual_odometry_topic",
            "/" + systemName + "/localisation/visual/odometry");

        /* Input topic: wheel_odometry's odometry output. */
        const std::string wheelTopic = declare_parameter<std::string>(
            "wheel_odometry_topic",
            "/" + systemName + "/localisation/wheel/odometry");

        /* Output topic: this node's fused pose/twist/slip estimate. */
        const std::string outputTopic = declare_parameter<std::string>(
            "output_topic",
            "/" + systemName + "/localisation/kalman_filter/odometry");

        /* Output topic: the retained, rate-limited estimated path. */
        const std::string estimatedPathTopic = declare_parameter<std::string>(
            "estimated_path_topic",
            "/" + systemName + "/localisation/kalman_filter/path");

        /*!
         * Input topic: wheel_odometry's raw per-cycle per-wheel slip
         * observations (std_msgs::msg::Float64MultiArray, WHEEL_COUNT
         * elements, NaN for a wheel not observable this cycle), fused as
         * six of this node's own states.
         */
        const std::string wheelSlipObservationTopic =
            declare_parameter<std::string>(
                "wheel_slip_observation_topic",
                "/" + systemName + "/localisation/wheel/slip_observation");

        /* Output topic: this node's fused per-wheel slip estimate
         * (std_msgs::msg::Float64MultiArray, WHEEL_COUNT elements), read
         * back by wheel_odometry. */
        const std::string wheelSlipEstimateTopic = declare_parameter<
            std::string>(
            "wheel_slip_estimate_topic",
            "/" + systemName + "/localisation/kalman_filter/wheel_slip_ratio");

        /* Parent frame published with the fused estimate. */
        odomFrame = declare_parameter<std::string>("odom_frame", "map");

        /* Child frame published with the fused estimate. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Fixed world frame the estimated path/TF are published under;
         * kept distinct from odom_frame so either can be overridden
         * independently even though both default to "map". */
        mapFrame = declare_parameter<std::string>("map_frame", "map");

        /* This node's own path/TF child frame, distinct from ground
         * truth's; kept distinct from base_frame for the same reason. */
        estimatedBaseFrame =
            declare_parameter<std::string>("estimated_base_frame",
                                           "alpha/base_link");

        /* Read the raw path-length limit before validating it below. */
        const int configuredMaximumPoses =
            declare_parameter<int>("path_maximum_poses", 5000);

        /* Read the raw sample period before validating it below. */
        const double pathSamplePeriodS =
            declare_parameter<double>("path_sample_period_s", 0.1);

        /* Reject a configuration that could never retain or sample a path. */
        if (configuredMaximumPoses <= 0 || pathSamplePeriodS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "Alpha path limits must be greater than zero");
        }

        /* Narrow the validated parameter to the member's storage type. */
        pathMaximumPoses = static_cast<std::size_t>(configuredMaximumPoses);

        /*!
         * Convert the configured period from seconds to nanoseconds so it
         * can be compared directly against message timestamps.
         */
        pathSamplePeriodNs = static_cast<std::int64_t>(pathSamplePeriodS *
                                                       NANOSECONDS_PER_SECOND);

        /*!
         * Input topic: ground_truth's own settled resting pose, published
         * once, latched (see GroundTruthNode::handleOdometryCallBack()).
         * initialPositionMapM/initialOrientation are seeded from it (see
         * handleInitialPoseCallBack()) instead of a manually re-measured,
         * terrain-specific constant.
         */
        const std::string initialPoseTopic = declare_parameter<std::string>(
            "initial_pose_topic",
            "/" + systemName + "/localisation/ground_truth/initial_pose");

        /* Configured prediction-timer rate, validated below. */
        const double predictionRateHz =
            declare_parameter<double>("prediction_rate_hz", 100.0);

        /* Maximum age a visual-odometry measurement may have before it is
         * dropped instead of fused; validated immediately below. */
        maximumVisualMeasurementAgeS = declare_parameter<double>(
            "maximum_visual_measurement_age_s", 0.5);

        /* A non-positive age limit would make every visual measurement
         * either always or never stale, which is never a useful
         * configuration. */
        if (maximumVisualMeasurementAgeS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "maximum_visual_measurement_age_s must be positive");
        }

        /* Variance floor applied to inertial measurements. */
        inertialVariance =
            declare_parameter<double>("inertial_measurement_variance", 0.20);

        /* Variance floor applied to visual-odometry measurements. */
        visualVariance =
            declare_parameter<double>("visual_measurement_variance", 0.03);

        /*!
         * Deliberately loose variance applied to visual odometry's
         * roll/pitch correction (see handleMeasurementCallBack()) so it
         * anchors long-run IMU attitude drift without overriding the IMU's
         * better short-term roll/pitch estimate.
         */
        visualAttitudeVariance = declare_parameter<double>(
            "visual_attitude_measurement_variance", 0.5);

        /* Variance floor applied to wheel-odometry measurements. */
        wheelVariance =
            declare_parameter<double>("wheel_measurement_variance", 0.08);

        /*!
         * Wheel odometry's own reported yaw variance (~0.04, see
         * publishOdometry.cc) assumes it can observe yaw as reliably as
         * position/velocity, but a single shared linear slip ratio cannot
         * represent rotational slip: under sustained turning, wheel's own
         * yaw can run well ahead of the truth with no internal signal that
         * anything is wrong (confirmed live: ~1.68x too fast at 0.15 rad/s
         * commanded). These two gains inflate wheel's yaw variance by
         * additional, independent evidence that this failure mode is
         * active, rather than trusting wheel's yaw unconditionally.
         */

        /* Scales yaw-variance inflation by the square of wheel's own
         * reported yaw rate: faster observed turning means slip is more
         * likely, so trust wheel's absolute yaw less. Zero at rest,
         * matching every stationary-divergence fix verified so far. */
        wheelYawRateVarianceGain =
            declare_parameter<double>("wheel_yaw_rate_variance_gain", 25.0);

        /* Scales yaw-variance inflation by the square of wheel's
         * disagreement with gyroOnlyYawRad -- an independent, slip-immune
         * yaw estimate integrated purely from the IMU's own gyro reading
         * (see gyroOnlyYawRad's doc comment). A large disagreement is
         * direct evidence wheel's yaw has drifted from the truth. */
        wheelYawGyroDisagreementGain = declare_parameter<double>(
            "wheel_yaw_gyro_disagreement_gain", 10.0);

        /*!
         * Hard cap on the combined inflation above. Under this exact
         * failure mode wheel's own yaw is often close to uncorrelated
         * noise, not just "off by a factor", so the disagreement term
         * routinely approaches its maximum (pi rad) -- confirmed live to
         * reach the point of catastrophic, unbounded position runaway
         * (inflating this one of six observed wheel axes by 100x+
         * relative to the others, O(0.01-0.08), made the measurement-
         * noise matrix ill-conditioned enough that Eigen::LDLT still
         * reported success while producing an inaccurate Kalman gain that
         * corrupted the cross-covariant velocity states rather than just
         * de-weighting yaw). This keeps wheel's yaw de-weighted far below
         * any other observed axis without letting one entry's scale run
         * away within the same small system.
         */
        maximumWheelYawVarianceInflation = declare_parameter<double>(
            "maximum_wheel_yaw_variance_inflation", 3.0);

        /* Measurement variance applied to wheel_odometry's raw wheel-slip
         * observation. */
        slipMeasurementVariance =
            declare_parameter<double>("slip_measurement_variance", 0.02);

        /* Upper bound applied to the fused slip-ratio state, matching
         * wheel_odometry's own physically-meaningful ceiling. */
        maximumSlipRatio =
            declare_parameter<double>("maximum_slip_ratio", 0.30);

        /* A slip ratio can never reach or exceed 1 (the wheel would be
         * spinning with no rolling contribution at all), and a
         * non-positive ceiling would make every observation invalid. */
        if (maximumSlipRatio <= 0.0 || maximumSlipRatio >= 1.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "maximum_slip_ratio must lie in (0, 1)");
        }

        /*!
         * Rate at which the slip state mean-reverts toward zero absent
         * fresh evidence (see computeProcessModel()'s doc comment). Read
         * as a time constant rather than a raw rate since that is the
         * more intuitive unit to tune: "recovers to within 1/e of zero in
         * about this many seconds without a supporting observation."
         */
        const double slipDecayTimeConstantS =
            declare_parameter<double>("slip_decay_time_constant_s", 2.0);

        /* A non-positive time constant has no meaningful decay rate. */
        if (slipDecayTimeConstantS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "slip_decay_time_constant_s must be positive");
        }

        /* Convert the configured time constant to the rate
         * computeProcessModel() actually applies. */
        slipDecayRateHz = 1.0 / slipDecayTimeConstantS;

        /*!
         * Process noise is diagonal and grouped by state block (position,
         * orientation, linear velocity, angular velocity); each block gets
         * its own configured variance applied to all three of its axes.
         * Stored on this node (not the engine) since predict() now takes
         * process noise as a per-call argument.
         */
        processNoise = StateMatrix::Zero();

        /* Position block (states 0-2). */
        setNoiseTriplet(
            processNoise, 0,
            declare_parameter<double>("process_noise_position", 0.02));

        /* Orientation block (states 3-5). */
        setNoiseTriplet(
            processNoise, 3,
            declare_parameter<double>("process_noise_orientation", 0.02));

        /* Linear-velocity block (states 6-8). */
        setNoiseTriplet(
            processNoise, 6,
            declare_parameter<double>("process_noise_linear_velocity", 0.20));

        /* Angular-velocity block (states 9-11). */
        setNoiseTriplet(
            processNoise, 9,
            declare_parameter<double>("process_noise_angular_velocity", 0.10));

        /*!
         * The six wheel-slip states (12-17, one per wheel) each mean-revert
         * toward zero at slipDecayRateHz rather than following pure
         * Euler-rate kinematics (see computeProcessModel()), so these
         * diagonal entries are set directly, one shared configured
         * variance applied to all six, rather than through
         * setNoiseTriplet(), which sets exactly three axes of one physical
         * block at once.
         */
        const double processNoiseSlipRatio =
            declare_parameter<double>("process_noise_slip_ratio", 0.0005);

        for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
        {
            processNoise(SLIP_STATE_START_INDEX + wheel,
                        SLIP_STATE_START_INDEX + wheel) = processNoiseSlipRatio;
        }

        /* Publish the fused estimate as ordinary odometry. */
        p_outputPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputTopic, rclcpp::QoS(10));

        /* Owned exclusively by this node; used by publishTransform(). */
        p_transformBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        /*!
         * Latch the path so a newly opened RViz view sees the current
         * history immediately, rather than waiting for the next sample.
         */
        p_estimatedPathPublisher = create_publisher<nav_msgs::msg::Path>(
            estimatedPathTopic,
            rclcpp::QoS(1).reliable().transient_local());

        /*!
         * Latch the fused slip estimate the same way so wheel_odometry
         * (whose own joint-state callback may fire before this node's
         * first slip observation is fused) always has a value to read,
         * starting from the "no slip" prior published immediately below.
         */
        p_wheelSlipPublisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            wheelSlipEstimateTopic,
            rclcpp::QoS(1).reliable().transient_local());

        /* Publish the initial "no slip assumed" prior immediately, before
         * any subscription can fire, so a subscriber never waits for the
         * first fused observation to see a value. */
        publishWheelSlip();

        /* ground_truth's one-time settled resting pose; latched, so this
         * arrives regardless of subscribe order relative to when
         * ground_truth publishes it. */
        p_initialPoseSubscription = create_subscription<nav_msgs::msg::Odometry>(
            initialPoseTopic, rclcpp::QoS(1).reliable().transient_local(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleInitialPoseCallBack(*p_message); });

        /* Every inertial-odometry message is fused as an inertial
         * measurement. */
        p_inertialSubscription = create_subscription<nav_msgs::msg::Odometry>(
            inertialTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the inertial-specific observation model and variance. */
                handleMeasurementCallBack(*p_message,
                                  MeasurementKind::MEASUREMENT_KIND_INERTIAL);
            });

        /* Every visual-odometry message is fused as a visual measurement. */
        p_visualSubscription = create_subscription<nav_msgs::msg::Odometry>(
            visualTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the visual-specific observation model, variance and age
                 * check. */
                handleMeasurementCallBack(*p_message,
                                  MeasurementKind::MEASUREMENT_KIND_VISUAL);
            });

        /* Every wheel-odometry message is fused as a wheel measurement. */
        p_wheelSubscription = create_subscription<nav_msgs::msg::Odometry>(
            wheelTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the wheel-specific observation model and variance. */
                handleMeasurementCallBack(*p_message,
                                  MeasurementKind::MEASUREMENT_KIND_WHEEL);
            });

        /* Every per-wheel slip observation array is fused directly into
         * the six slip states it carries an observation for. */
        p_wheelSlipObservationSubscription =
            create_subscription<std_msgs::msg::Float64MultiArray>(
                wheelSlipObservationTopic, rclcpp::SensorDataQoS(),
                [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr p_message)
                { handleSlipObservationCallBack(*p_message); });

        /*!
         * A non-positive configured rate would make the wall-timer period
         * zero or negative, so the effective rate is floored at 1 Hz.
         */
        const double safeRateHz = std::max(1.0, predictionRateHz);

        /* Drive continuous prediction between measurements at the
         * configured (or floored) rate. */
        p_predictionTimer = create_wall_timer(
            std::chrono::duration<double>(1.0 / safeRateHz),
            [this]()
            {
                /*!
                 * Prediction has nothing to propagate from until the first
                 * measurement has established an initial state (see
                 * handleMeasurementCallBack()), so skip silently until then.
                 */
                if (!hasInitialState)
                {
                    /* No state exists yet to predict forward from. */
                    return;
                }

                /* Read the current time once for this prediction step. */
                const rclcpp::Time stamp = now();

                /* Advance the filter to now without applying a
                 * measurement. */
                const FilterStatus status = predictTo(stamp.seconds());

                /* Only publish, and only mark an estimate as available,
                 * once the engine actually reports success. */
                if (status == FilterStatus::FILTER_STATUS_SUCCESS)
                {
                    /* Refresh this node's cached copy of the engine's
                     * state/covariance for publishEstimate() to read. */
                    latestState = filter.getState();
                    latestCovariance = filter.getCovariance();

                    /* Record that at least one valid estimate now exists. */
                    hasEstimate = true;

                    /* Publish the freshly predicted estimate immediately. */
                    publishEstimate(stamp);
                }
                else
                {
                    /* Report the rejected step without crashing the timer. */
                    logStepFailure(status);
                }
            });

        /* Record the resolved fusion rate once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(),
                    "Continuous EKF fusing inertial, visual and wheel odometry "
                    "at %.1f Hz",
                    safeRateHz);
    }

    /*!
     * @brief           Terminates the owned EKF and logs a failure if that
     *                  lifecycle transition is rejected.
     */
    ~AlphaKalmanFilterNode() noexcept override
    {
        /* Return the owned engine to its uninitialized lifecycle state. */
        const FilterStatus status = filter.terminate();

        /* Termination only fails if the engine was never successfully
         * initialized in the first place; report that rather than hide
         * it, even though there is nothing left to do about it here. */
        if (status != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            /* Log at error severity since this indicates a lifecycle bug. */
            RCLCPP_ERROR(get_logger(), "Continuous EKF termination failed");
        }
    }

    AlphaKalmanFilterNode(const AlphaKalmanFilterNode &otherNode_in) = delete;
    AlphaKalmanFilterNode &operator=(
        const AlphaKalmanFilterNode &otherNode_in) = delete;
    AlphaKalmanFilterNode(AlphaKalmanFilterNode &&otherNode_in) = delete;
    AlphaKalmanFilterNode &operator=(
        AlphaKalmanFilterNode &&otherNode_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Reusable continuous-discrete EKF engine type.
     */
    using Filter = localisation::kalman_filter::ekf_continuous_kalman_filter::
        ContinuousExtendedKalmanFilter;

    /*!
     * @brief       Filter lifecycle/operation status type.
     */
    using FilterStatus =
        localisation::kalman_filter::ekf_continuous_kalman_filter::FilterStatus;

    /*!
     * @brief       This node's fixed-size state vector, converted to/from
     *              the engine's dynamically-sized Eigen::VectorXd at each
     *              call.
     */
    using StateVector = Eigen::Matrix<double, 18, 1>;

    /*!
     * @brief       This node's fixed-size state covariance/process-noise
     *              type, converted to/from the engine's dynamically-sized
     *              Eigen::MatrixXd at each call.
     */
    using StateMatrix = Eigen::Matrix<double, 18, 18>;

    /*!
     * @brief       Number of scalar states in Alpha's EKF model.
     */
    static constexpr Eigen::Index STATE_SIZE = 18;

    /*!
     * @brief       Number of independently steered/driven wheels, and the
     *              number of per-wheel slip states starting at
     *              SLIP_STATE_START_INDEX. Matches WheelOdometryNode's own
     *              wheel count and front-left..rear-right ordering.
     */
    static constexpr Eigen::Index WHEEL_COUNT = 6;

    /*!
     * @brief       State index of the first of WHEEL_COUNT independent,
     *              dimensionless per-wheel slip states, each in
     *              [0, maximumSlipRatio); wheel w's slip state is at
     *              SLIP_STATE_START_INDEX + w. A single shared value
     *              cannot represent rotational slip (inner/outer wheels
     *              genuinely slip by different amounts during a turn), so
     *              each wheel is fused independently -- see
     *              handleSlipObservationCallBack().
     */
    static constexpr Eigen::Index SLIP_STATE_START_INDEX = 12;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Applies one odometry measurement to the EKF, using
     *                   the observation matrix that source can actually
     *                   observe.
     *
     * On the very first measurement of any kind, this also seeds the
     * filter's initial position and attitude from the configured initial
     * pose before applying the measurement itself, establishing the time
     * origin used by predictTo().
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source.
     * @param[in]       kind_in
     *                  Selects the observation matrix, minimum variance, and
     *                  (for visual odometry) age-compensation behaviour.
     */
    void handleMeasurementCallBack(const nav_msgs::msg::Odometry &message_in,
                           MeasurementKind kind_in);

    /*!
     * @brief           Seeds initialPositionMapM/initialOrientation from
     *                   ground_truth's one-time settled resting pose.
     *
     * Fires exactly once in practice (ground_truth only ever publishes one
     * message on this topic), but is not itself latched against being
     * called again -- harmless, since it would simply re-derive the same
     * value from the same retained message.
     *
     * @param[in]       message_in
     *                  ground_truth's settled resting pose, in the map
     *                  frame.
     */
    void handleInitialPoseCallBack(const nav_msgs::msg::Odometry &message_in);

    /*!
     * @brief           Fuses whichever wheels' raw slip observations
     *                   arrived this cycle directly into their own slip
     *                   states.
     *
     * Unlike handleMeasurementCallBack(), this does not seed the filter's
     * initial state (a slip observation carries no pose information) and is
     * silently dropped until an odometry measurement has already
     * established one. A wheel whose observation is NaN this cycle (see
     * WheelOdometryNode::publishSlipObservation()) is simply excluded from
     * this update's observed indices, not fused as zero.
     *
     * @param[in]       message_in
     *                  Raw per-cycle per-wheel slip observations from
     *                  wheel_odometry, WHEEL_COUNT elements, each
     *                  dimensionless in [0, 1) or NaN.
     */
    void handleSlipObservationCallBack(
        const std_msgs::msg::Float64MultiArray &message_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Sets three consecutive diagonal process-noise entries
     *                  to one variance, floored above zero for numerical
     *                  conditioning.
     *
     * @param[in,out]   noise_inout
     *                  Process-noise matrix whose diagonal is updated.
     * @param[in]       firstIndex_in
     *                  First of three consecutive state indices to set.
     * @param[in]       variance_in
     *                  Configured variance in squared state units per second.
     */
    static void setNoiseTriplet(StateMatrix &noise_inout,
                                Eigen::Index firstIndex_in, double variance_in);

    /*!
     * @brief           Evaluates Alpha's Euler-rate process model and its
     *                   Jacobian at one state, for the engine's predict().
     *
     * The six per-wheel slip states (12-17) are the one exception to pure
     * Euler-rate kinematics: each mean-reverts toward zero at
     * slipDecayRateHz absent fresh evidence (an Ornstein-Uhlenbeck-style
     * term), rather than following a pure random walk, so a stale or
     * spuriously-fused slip observation relaxes back toward "no slip
     * assumed" over time instead of persisting indefinitely -- this is not
     * a static method for that reason, since slipDecayRateHz is configured
     * per-node.
     *
     * @param[in]       state_in
     *                  State at which to linearize.
     * @param[out]      stateDerivative_out
     *                  f(x) evaluated at state_in.
     * @param[out]      processJacobian_out
     *                  F = df/dx evaluated at state_in.
     */
    void computeProcessModel(const StateVector &state_in,
                             StateVector &stateDerivative_out,
                             StateMatrix &processJacobian_out) const;

    /*!
     * @brief           Propagates the owned filter to a requested monotonic
     *                   timestamp, re-linearizing computeProcessModel() at
     *                   the engine's current state every bounded substep.
     *
     * @param[in]       targetTimestampS_in
     *                  Monotonic target time in seconds; must not precede
     *                  the filter's current time.
     * @return          Lifecycle or numerical status.
     */
    FilterStatus predictTo(double targetTimestampS_in);

    /*!
     * @brief           Builds the one-hot observation matrix selecting the
     *                   given state indices out of Alpha's eighteen-state
     *                   model.
     *
     * @param[in]       observedIndices_in
     *                  State indices this observation matrix selects, in
     *                  row order.
     * @param[in]       stateSize_in
     *                  Number of columns (Alpha's STATE_SIZE).
     * @return          observedIndices_in.size() x stateSize_in observation
     *                  matrix.
     */
    static Eigen::MatrixXd
    buildObservationMatrix(const std::vector<Eigen::Index> &observedIndices_in,
                           Eigen::Index stateSize_in);

    /*!
     * @brief           Converts one odometry message into an absolute EKF
     *                   state vector expressed in the map frame.
     *
     * Every source except wheel_odometry reports pose relative to the
     * configured initial pose, rebased here by rotating through
     * initialOrientation and translating by initialPositionMapM.
     * wheel_odometry is the one exception: it integrates and publishes
     * directly in absolute map-frame terms (its own message's position
     * already includes the initial pose, and its own message's orientation
     * already carries the live roll/pitch it borrowed from this node's own
     * previous fused output for projection -- see
     * WheelOdometryNode::handleJointStateCallBack()), so rotating or
     * translating it again here would double-apply that same offset; for
     * MEASUREMENT_KIND_WHEEL, the message's pose/orientation is used
     * verbatim as the map-frame value instead.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source, with
     *                  twist expressed in the child (body) frame per ROS
     *                  convention.
     * @param[in]       kind_in
     *                  Selects whether message_in's pose is rebased
     *                  (every source except wheel) or used verbatim (wheel).
     * @return          State vector with position (map, m), roll/pitch/yaw
     *                  (map, rad), linear velocity (map, m/s) and angular
     *                  rate (body, rad/s) populated; the slip state (12) is
     *                  left at zero, since no odometry message carries it.
     */
    StateVector odometryToState(const nav_msgs::msg::Odometry &message_in,
                                MeasurementKind kind_in) const;

    /*!
     * @brief           Wraps an angle into the canonical [-pi, pi] range.
     *
     * @param[in]       angleRad_in
     *                  Angle in radians.
     * @return          Equivalent angle in radians within [-pi, pi].
     */
    static double wrapAngle(double angleRad_in);

    /*!
     * @brief           Advances a state vector by a simple constant-rate
     *                   extrapolation to compensate for measurement latency.
     *
     * Used only for visual odometry, whose measurement stamp can lag the
     * filter's current time by up to maximum_visual_measurement_age_s.
     * Position is advanced by the current linear-velocity state; roll,
     * pitch and yaw are advanced by the same Euler-rate kinematics
     * computeProcessModel() evaluates for the EKF's own process model.
     *
     * @param[in,out]   state_inout
     *                  State vector advanced in place by ageS_in.
     * @param[in]       ageS_in
     *                  Non-negative measurement age in seconds.
     */
    static void compensateMeasurementAge(StateVector &state_inout,
                                         double ageS_in);

    /*!
     * @brief           Derives per-state measurement variances from a
     *                   message's reported covariance, falling back to a
     *                   minimum floor for unreported or non-positive
     *                   entries.
     *
     * @param[in]       message_in
     *                  Odometry message whose diagonal pose/twist covariance
     *                  entries are read.
     * @param[in]       minimumVariance_in
     *                  Variance floor applied when a covariance entry is
     *                  missing, non-finite, or non-positive.
     * @return          Twelve-element variance vector in the EKF's state
     *                  units squared.
     */
    static StateVector measurementVariances(
        const nav_msgs::msg::Odometry &message_in, double minimumVariance_in);

    /*!
     * @brief           Logs a throttled warning describing a rejected EKF
     *                   step.
     *
     * @param[in]       status_in
     *                  Non-success status returned by the filter or by
     *                  predictTo().
     */
    void logStepFailure(FilterStatus status_in);

    /*!
     * @brief           Publishes the latest fused estimate as odometry, then
     *                   hands it to publishEstimatedPath() for TF/path.
     *
     * @param[in]       stamp_in
     *                  Timestamp applied to the published message and
     *                  transform header.
     */
    void publishEstimate(const rclcpp::Time &stamp_in);

    /*!
     * @brief           Tests whether a pose can be published safely.
     *
     * @param[in]       pose_in
     *                  Pose expressed in the map frame.
     *
     * @return          True when position and orientation are finite and the
     *                  quaternion has a nonzero norm.
     */
    static bool isPoseValid(const geometry_msgs::msg::Pose &pose_in);

    /*!
     * @brief           Converts a ROS timestamp to nanoseconds.
     *
     * @param[in]       stamp_in
     *                  Timestamp to convert.
     *
     * @return          Timestamp in nanoseconds.
     */
    static std::int64_t
        stampToNanoseconds(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Validates and reframes one estimate odometry message.
     *
     * @param[in]       odometry_in
     *                  Odometry to validate and reframe.
     *
     * @param[in]       parentFrame_in
     *                  Frame id written into the output header.
     *
     * @param[in]       childFrame_in
     *                  Frame id written into the output child frame.
     *
     * @param[out]      odometry_out
     *                  Reframed odometry, valid only when this method returns
     *                  true.
     *
     * @return          True when the input pose was valid and odometry_out
     *                  was written.
     */
    bool prepareOdometry(const nav_msgs::msg::Odometry &odometry_in,
                         const std::string             &parentFrame_in,
                         const std::string             &childFrame_in,
                         nav_msgs::msg::Odometry       &odometry_out);

    /*!
     * @brief           Appends a time-sampled pose to a bounded path.
     *
     * @param[in]       odometry_in
     *                  Valid odometry already expressed in the target
     *                  frames.
     *
     * @param[in,out]   path_inout
     *                  Path accumulator updated in place.
     *
     * @param[in,out]   lastStampNs_inout
     *                  Timestamp of the most recently retained sample,
     *                  updated in place.
     *
     * @param[in,out]   publisher_inout
     *                  Publisher used to republish the updated path.
     */
    void
        appendPathPose(const nav_msgs::msg::Odometry &odometry_in,
                       nav_msgs::msg::Path           &path_inout,
                       std::int64_t                  &lastStampNs_inout,
                       rclcpp::Publisher<nav_msgs::msg::Path> &publisher_inout);

    /*!
     * @brief           Publishes the estimated map-to-body transform.
     *
     * @param[in]       odometry_in
     *                  Valid odometry already expressed in the target
     *                  frames.
     */
    void publishTransform(const nav_msgs::msg::Odometry &odometry_in);

    /*!
     * @brief           Validates one fused estimate and, if valid,
     *                   broadcasts its TF transform and appends it to the
     *                   retained estimated path.
     *
     * @param[in]       odometry_in
     *                  Just-published fused estimate, in odomFrame/
     *                  baseFrame.
     */
    void publishEstimatedPath(const nav_msgs::msg::Odometry &odometry_in);

    /*!
     * @brief           Publishes the fused wheel-slip state for
     *                   wheel_odometry to read back.
     */
    void publishWheelSlip();

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Nanoseconds in one second, used to convert path-sample
     *              period.
     */
    static constexpr double NANOSECONDS_PER_SECOND = 1.0e9;

    /*!
     * @brief       Owned continuous-discrete EKF instance; exclusively
     *              owned and accessed only from this node's single
     *              executor thread.
     */
    Filter filter;

    /*!
     * @brief       Most recent posterior state; refreshed from
     *              filter.getState() after every successful predict/update.
     */
    StateVector latestState{StateVector::Zero()};

    /*!
     * @brief       Most recent posterior covariance; refreshed from
     *              filter.getCovariance() after every successful
     *              predict/update.
     */
    StateMatrix latestCovariance{StateMatrix::Identity()};

    /*!
     * @brief       Continuous process-noise density, passed to the engine's
     *              predict() on every call.
     */
    StateMatrix processNoise{StateMatrix::Zero()};

    /*!
     * @brief       Monotonic time in seconds the owned filter's state is
     *              currently valid at; only meaningful once hasInitialState
     *              is true.
     */
    double filterTimestampS{0.0};

    /*!
     * @brief       Publishes the fused odometry estimate.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_outputPublisher;

    /*!
     * @brief       Subscribes to inertial_odometry's odometry output.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_inertialSubscription;

    /*!
     * @brief       Subscribes to ground_truth's one-time settled resting
     *              pose.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_initialPoseSubscription;

    /*!
     * @brief       Subscribes to visual_odometry's odometry output.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_visualSubscription;

    /*!
     * @brief       Subscribes to wheel_odometry's odometry output.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        p_wheelSubscription;

    /*!
     * @brief       Subscribes to wheel_odometry's raw per-cycle per-wheel
     *              slip observations.
     */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        p_wheelSlipObservationSubscription;

    /*!
     * @brief       Drives EKF prediction at prediction_rate_hz between
     *              measurements.
     */
    rclcpp::TimerBase::SharedPtr p_predictionTimer;

    /*!
     * @brief       Publishes the estimated map-to-body transform; used by
     *              publishTransform().
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> p_transformBroadcaster;

    /*!
     * @brief       Publishes a retained, bounded estimated path.
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_estimatedPathPublisher;

    /*!
     * @brief       Publishes the fused per-wheel slip state; used by
     *              publishWheelSlip().
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        p_wheelSlipPublisher;

    /*!
     * @brief       Accumulated estimated poses, all expressed in mapFrame.
     */
    nav_msgs::msg::Path estimatedPath;

    /*!
     * @brief       Rover's settled resting attitude in the map frame,
     *              seeded from ground_truth's one-time initial pose (see
     *              handleInitialPoseCallBack()); used to rebase every
     *              incoming relative odometry measurement. Meaningless
     *              (identity) until hasReceivedInitialPose is true.
     */
    tf2::Quaternion initialOrientation{0.0, 0.0, 0.0, 1.0};

    /*!
     * @brief       Rover's settled resting position in the map frame,
     *              metres; see initialOrientation's doc comment above.
     */
    tf2::Vector3 initialPositionMapM{0.0, 0.0, 0.0};

    /*!
     * @brief       Frame published as the parent of the fused estimate.
     */
    std::string odomFrame;

    /*!
     * @brief       Frame published as the child of the fused estimate.
     */
    std::string baseFrame;

    /*!
     * @brief       Fixed world frame the estimated path/TF are published
     *              under.
     */
    std::string mapFrame;

    /*!
     * @brief       This node's own path/TF child frame, distinct from
     *              ground truth's.
     */
    std::string estimatedBaseFrame;

    /*!
     * @brief       Maximum number of poses retained in the estimated path.
     */
    std::size_t pathMaximumPoses{5000U};

    /*!
     * @brief       Minimum time between retained path samples in
     *              nanoseconds.
     */
    std::int64_t pathSamplePeriodNs{100000000};

    /*!
     * @brief       Timestamp of the most recently retained path sample.
     */
    std::int64_t lastEstimatedPathStampNs{-1};

    /*!
     * @brief       Minimum inertial measurement variance floor.
     */
    double inertialVariance{0.20};

    /*!
     * @brief       Minimum visual measurement variance floor.
     */
    double visualVariance{0.03};

    /*!
     * @brief       Deliberately loose variance applied to visual
     *              odometry's roll/pitch correction, so it anchors
     *              long-run IMU attitude drift without overriding the
     *              IMU's own better short-term estimate.
     */
    double visualAttitudeVariance{0.5};

    /*!
     * @brief       Minimum wheel measurement variance floor.
     */
    double wheelVariance{0.08};

    /*!
     * @brief       Scales wheel yaw-variance inflation by the square of
     *              wheel's own reported yaw rate; see its declare_parameter
     *              call for the rationale.
     */
    double wheelYawRateVarianceGain{25.0};

    /*!
     * @brief       Scales wheel yaw-variance inflation by the square of
     *              wheel's disagreement with gyroOnlyYawRad; see its
     *              declare_parameter call for the rationale.
     */
    double wheelYawGyroDisagreementGain{10.0};

    /*!
     * @brief       Hard cap on wheel's combined yaw-variance inflation;
     *              see its declare_parameter call for the rationale.
     */
    double maximumWheelYawVarianceInflation{3.0};

    /*!
     * @brief       Yaw estimate integrated purely from the IMU's own gyro
     *              reading (latestGyroYawRateRadps), independent of the
     *              fused EKF state -- wheel/visual corrections never touch
     *              it directly. Reset to the fused yaw whenever a visual
     *              update succeeds (the trustworthy absolute reference
     *              when available), so it rides out, un-drifted over the
     *              short term, exactly the periods where visual is stale
     *              and wheel's own yaw is least trustworthy (see
     *              handleMeasurementCallBack()'s wheel-yaw variance
     *              inflation). Not a replacement for the fused yaw state:
     *              gyro integration alone would drift over long
     *              stationary or visual-starved periods, which periodic
     *              visual resets are relied on to bound.
     */
    double gyroOnlyYawRad{0.0};

    /*!
     * @brief       Latest yaw rate reported by inertial_odometry, body
     *              frame, rad/s -- the IMU's own gyro reading, not the
     *              fused angular-rate state (which wheel also corrects,
     *              making it unsuitable as a slip-immune cross-check).
     *              Updated only by inertial measurements; integrated into
     *              gyroOnlyYawRad by predictTo().
     */
    double latestGyroYawRateRadps{0.0};

    /*!
     * @brief       Measurement variance applied to wheel_odometry's raw
     *              wheel-slip observation.
     */
    double slipMeasurementVariance{0.02};

    /*!
     * @brief       Upper bound applied to the fused slip-ratio state,
     *              dimensionless in (0, 1).
     */
    double maximumSlipRatio{0.30};

    /*!
     * @brief       Rate (1 / time constant, Hz) at which
     *              computeProcessModel() mean-reverts the slip state
     *              toward zero absent fresh evidence.
     */
    double slipDecayRateHz{0.5};

    /*!
     * @brief       Maximum accepted visual-odometry measurement age in seconds.
     */
    double maximumVisualMeasurementAgeS{0.5};

    /*!
     * @brief       True once filter.predict()/update() has produced at
     *              least one posterior state.
     */
    bool hasEstimate{false};

    /*!
     * @brief       True once the filter's initial state and time origin are
     *              seeded.
     */
    bool hasInitialState{false};

    /*!
     * @brief       True once initialPositionMapM/initialOrientation have
     *              been seeded from ground_truth's one-time initial pose
     *              (see handleInitialPoseCallBack()). handleMeasurementCallBack()
     *              drops every measurement until this is true, since there
     *              is nothing sensible to rebase against or seed the
     *              filter from before then.
     */
    bool hasReceivedInitialPose{false};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_KALMAN_FILTER_NODE_H */
