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
#include "console/console.h"

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"
#include "objects/ErrorStateIndex.h"
#include "objects/FilterStatus.h"
#include "objects/FusedMeasurement.h"
#include "objects/ImuRingBuffer.h"
#include "objects/MeasurementFusionResult.h"
#include "objects/MeasurementHistory.h"
#include "objects/MeasurementKind.h"
#include "objects/SourceDiagnostics.h"
#include "objects/StateIndex.h"

/* Data include */
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/* Generic Libraries */
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
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
 * @brief           Fuses raw IMU, visual pose, and wheel velocity measurements
 *                  into a bias-aware rover estimate.
 *
 * This node owns Alpha's 16-component nominal state (fixed-frame position and
 * velocity, body-to-fixed quaternion, accelerometer bias, and gyroscope bias)
 * and its 15-component multiplicative error state. Raw IMU samples drive
 * timestamp-ordered prediction; visual odometry (as an absolute pose, or
 * as a per-interval velocity increment; see visual_fusion_mode) and wheel
 * body velocity apply source-specific corrections through the reusable EKF
 * engine. The estimate is published once per IMU-driven propagation,
 * stamped with the state's own timestamp and capped at prediction_rate_hz
 * of simulation time, so no two messages share a stamp. Every
 * fused pose/twist estimate is published on
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
     * @brief           Declares parameters and wires sensor subscriptions,
     *                  output publishers, and the output timer.
     *
     * @throws          std::invalid_argument if a configured measurement-age,
     *                  path, slip, or variance constraint is invalid.
     */
    AlphaKalmanFilterNode() :
        Node("continuous_ekf")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Raw specific force and angular rate are corrected by the ESKF's own
         * bias states before propagation. */
        const std::string rawImuTopic =
            declare_parameter<std::string>("raw_imu_topic",
                                           "/" + systemName + "/imu");

        /* Input topic: visual_odometry's odometry output. */
        const std::string visualTopic = declare_parameter<std::string>(
            "visual_odometry_topic",
            "/" + systemName + "/localisation/visual/odometry");

        /* Input topic: wheel_odometry's odometry output. */
        const std::string wheelTopic = declare_parameter<std::string>(
            "wheel_odometry_topic",
            "/" + systemName + "/localisation/wheel/odometry");

        /* Output topic: this node's fused pose and twist estimate. */
        const std::string outputTopic = declare_parameter<std::string>(
            "output_topic",
            "/" + systemName + "/localisation/kalman_filter/odometry");

        /* Output topic: the retained, rate-limited estimated path. */
        const std::string estimatedPathTopic = declare_parameter<std::string>(
            "estimated_path_topic",
            "/" + systemName + "/localisation/kalman_filter/path");

        /* Output topic: periodic estimator diagnostics, shared by every
         * node of the owning system and recorded with every run. */
        const std::string diagnosticsTopic =
            declare_parameter<std::string>("diagnostics_topic",
                                           "/" + systemName + "/diagnostics");

        /* Compatibility output retained as a neutral no-slip diagnostic. */
        const std::string wheelSlipEstimateTopic =
            declare_parameter<std::string>(
                "wheel_slip_estimate_topic",
                "/" + systemName +
                    "/localisation/kalman_filter/wheel_slip_ratio");

        /* Parent frame published with the fused estimate. */
        odomFrame =
            declare_parameter<std::string>("odom_frame",
                                           systemName + "/startup_fixed");

        /* Child frame published with the fused estimate. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Fixed world frame the estimated path/TF are published under;
         * kept distinct from odom_frame so either can be overridden
         * independently even though both default to "map". */
        mapFrame =
            declare_parameter<std::string>("map_frame",
                                           systemName + "/startup_fixed");

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

        /* Configured prediction-timer rate, validated below. */
        const double predictionRateHz =
            declare_parameter<double>("prediction_rate_hz", 100.0);

        /* Maximum age a visual-odometry measurement may have before it is
         * dropped instead of fused; validated immediately below. */
        maximumVisualMeasurementAgeS =
            declare_parameter<double>("maximum_visual_measurement_age_s", 0.75);

        /* Maximum interval over which the latest filtered IMU sample may be
         * held constant when prediction reaches beyond its timestamp. */
        maximumImuMeasurementAgeS =
            declare_parameter<double>("maximum_imu_measurement_age_s", 0.25);

        /* Source ablations preserve the independent measurement roles. */
        shouldFuseVisualPose =
            declare_parameter<bool>("fuse_visual_pose", true);
        shouldFuseWheelTwist =
            declare_parameter<bool>("fuse_wheel_twist", true);

        /* Zero selects the 99% chi-square limit for the update's actual
         * active dimension; a positive value explicitly overrides it. */
        visualNisThreshold =
            declare_parameter<double>("visual_nis_threshold", 0.0);
        wheelNisThreshold =
            declare_parameter<double>("wheel_nis_threshold", 0.0);
        if (visualNisThreshold < 0.0 || wheelNisThreshold < 0.0)
        {
            throw std::invalid_argument(
                "NIS thresholds must be zero (automatic) or positive");
        }

        /* Largest amount a measurement stamp may lead this node's clock
         * and still be fused. The measurement path (bridge, driver, wheel
         * odometry) can deliver a message a few milliseconds before the
         * corresponding 1 kHz /clock update reaches this node; before this
         * tolerance every such message was rejected as negative age
         * (LOC-4). */
        maximumFutureStampS =
            declare_parameter<double>("maximum_future_stamp_s", 0.02);
        if (!(maximumFutureStampS >= 0.0))
        {
            throw std::invalid_argument(
                "maximum_future_stamp_s must not be negative");
        }

        /* A non-positive age limit would make every visual measurement
         * either always or never stale, which is never a useful
         * configuration. */
        if (maximumVisualMeasurementAgeS <= 0.0 ||
            maximumImuMeasurementAgeS <= 0.0)
        {
            /* Fail fast at construction rather than misbehave later. */
            throw std::invalid_argument(
                "measurement age limits must be positive");
        }

        const std::int64_t configuredInitializationSamples =
            declare_parameter<std::int64_t>("imu_initialization_samples", 100);
        gravityMagnitudeMps2 = declare_parameter<double>("gravity_mps2", 1.62);
        if (configuredInitializationSamples <= 0 || gravityMagnitudeMps2 <= 0.0)
        {
            throw std::invalid_argument(
                "IMU initialization samples and gravity must be positive");
        }
        imuInitializationSampleTarget =
            static_cast<std::size_t>(configuredInitializationSamples);

        /* Variance floor applied to visual-odometry measurements. */
        visualVariance =
            declare_parameter<double>("visual_measurement_variance", 0.03);

        /*!
         * How visual odometry is fused. "pose" treats its accumulated pose
         * as an absolute position and attitude measurement. "increment"
         * fuses only the motion between consecutive visual messages: the
         * average body velocity over the interval and the average yaw
         * rate, which observes the gyroscope z bias. Visual odometry has
         * no map, so its accumulated pose error is strongly correlated
         * over time; fusing it as independent absolute fixes makes the
         * filter follow that drift.
         */
        const std::string visualFusionMode =
            declare_parameter<std::string>("visual_fusion_mode", "pose");
        if (visualFusionMode != "pose" && visualFusionMode != "increment")
        {
            throw std::invalid_argument(
                "visual_fusion_mode must be \"pose\" or \"increment\"");
        }
        isVisualIncrementMode = visualFusionMode == "increment";

        /* Variance floors for the increment-mode body velocity and yaw
         * rate; the reported visual twist covariance applies above them. */
        visualIncrementVelocityVariance =
            declare_parameter<double>("visual_increment_velocity_variance",
                                      1.0e-6);
        visualIncrementYawRateVariance =
            declare_parameter<double>("visual_increment_yaw_rate_variance",
                                      1.0e-6);
        if (!(visualIncrementVelocityVariance > 0.0) ||
            !(visualIncrementYawRateVariance > 0.0))
        {
            throw std::invalid_argument(
                "visual increment variance floors must be positive");
        }

        /*!
         * Deliberately loose variance applied to visual odometry's
         * roll/pitch correction (see handleMeasurementCallBack()) so it
         * anchors long-run IMU attitude drift without overriding the IMU's
         * better short-term roll/pitch estimate.
         */
        visualAttitudeVariance =
            declare_parameter<double>("visual_attitude_measurement_variance",
                                      0.5);

        /* Variance floor applied to wheel-odometry measurements. */
        wheelVariance =
            declare_parameter<double>("wheel_measurement_variance", 0.10);

        /* Raw-IMU and bias random-walk noise are continuous-time variances. */
        processNoise = ErrorStateMatrix::Zero();
        setNoiseTriplet(
            processNoise,
            static_cast<Eigen::Index>(
                ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X),
            declare_parameter<double>("gyroscope_noise_variance", 0.000064));
        setNoiseTriplet(
            processNoise,
            static_cast<Eigen::Index>(
                ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X),
            declare_parameter<double>("accelerometer_noise_variance", 0.0025));
        setNoiseTriplet(
            processNoise,
            static_cast<Eigen::Index>(
                ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X),
            declare_parameter<double>("accelerometer_bias_random_walk_variance",
                                      1.0e-6));
        setNoiseTriplet(
            processNoise,
            static_cast<Eigen::Index>(
                ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X),
            declare_parameter<double>("gyroscope_bias_random_walk_variance",
                                      1.0e-8));
        initialAccelerometerBiasVariance =
            declare_parameter<double>("initial_accelerometer_bias_variance",
                                      0.01);
        initialGyroscopeBiasVariance =
            declare_parameter<double>("initial_gyroscope_bias_variance", 0.001);

        /* Readiness reported to the start-up supervisor: initialised, fed
         * enough visual corrections, settled for a while, and not
         * diverged. The variance limit is a divergence guard, not a
         * start-up criterion: visual-odometry covariance legitimately
         * grows while driving. */
        const std::int64_t configuredReadinessVisualUpdates =
            declare_parameter<std::int64_t>("readiness_minimum_visual_updates",
                                            3);
        readinessSettleTimeS =
            declare_parameter<double>("readiness_settle_time_s", 5.0);
        readinessMaximumPositionVarianceM2 =
            declare_parameter<double>("readiness_maximum_position_variance_m2",
                                      1.0);
        if (configuredReadinessVisualUpdates < 0 ||
            !(readinessSettleTimeS >= 0.0) ||
            !(readinessMaximumPositionVarianceM2 > 0.0))
        {
            throw std::invalid_argument(
                "EKF readiness limits are outside valid bounds");
        }
        readinessMinimumVisualUpdates =
            static_cast<std::uint64_t>(configuredReadinessVisualUpdates);

        /* Publish the fused estimate as ordinary odometry. */
        p_outputPublisher =
            create_publisher<nav_msgs::msg::Odometry>(outputTopic,
                                                      rclcpp::QoS(10));

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
        p_wheelSlipPublisher =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                wheelSlipEstimateTopic,
                rclcpp::QoS(1).reliable().transient_local());

        /* Publish the initial "no slip assumed" prior immediately, before
         * any subscription can fire, so a subscriber never waits for the
         * first fused observation to see a value. */
        publishWheelSlip();

        /* Every raw IMU message is retained and processed in timestamp order.
         */
        p_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
            rawImuTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr p_message)
            { handleImuCallBack(*p_message); });

        /* Every visual-odometry message is fused as a visual measurement. */
        p_visualSubscription = create_subscription<nav_msgs::msg::Odometry>(
            visualTopic,
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the visual-specific observation model, variance and age
                 * check. */
                handleMeasurementCallBack(
                    *p_message,
                    MeasurementKind::MEASUREMENT_KIND_VISUAL);
            });

        /* Every wheel-odometry message is fused as a wheel measurement. */
        p_wheelSubscription = create_subscription<nav_msgs::msg::Odometry>(
            wheelTopic,
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            {
                /* Delegate to the shared fusion logic, tagged so it applies
                 * the wheel-specific observation model and variance. */
                handleMeasurementCallBack(
                    *p_message,
                    MeasurementKind::MEASUREMENT_KIND_WHEEL);
            });

        /*!
         * The estimate is published once per IMU-driven state propagation,
         * stamped with the state's own time, so no message repeats a stamp
         * or an unchanged state. prediction_rate_hz caps that output rate
         * in simulation time (a non-positive value is floored at 1 Hz);
         * at the 50 Hz IMU rate and the default 100 Hz cap every
         * propagation is published.
         */
        const double safeRateHz = std::max(1.0, predictionRateHz);
        minimumOutputPeriodS    = 1.0 / safeRateHz;

        /* Report bounded counters separately from the high-rate data path,
         * once per simulated second so the recorded diagnostics share the
         * clock of every other recorded topic. */
        p_diagnosticsPublisher =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnosticsTopic,
                rclcpp::QoS(10));
        p_diagnosticsTimer =
            create_timer(std::chrono::seconds(1),
                         [this]() { publishDiagnosticsCallBack(); });

        /* The resolved configuration is in the run's parameter snapshot, so
         * it is debug detail rather than operator output. */
        LUNAR_LOG_DEBUG(get_logger(),
                        "Bias-aware ESKF fusing raw IMU, visual and wheel "
                        "measurements; publishing at most %.1f Hz",
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
            LUNAR_LOG_ERROR(get_logger(), "EKF termination failed");
        }
    }

    AlphaKalmanFilterNode(const AlphaKalmanFilterNode &otherNode_in) = delete;
    AlphaKalmanFilterNode &
        operator=(const AlphaKalmanFilterNode &otherNode_in)    = delete;
    AlphaKalmanFilterNode(AlphaKalmanFilterNode &&otherNode_in) = delete;
    AlphaKalmanFilterNode &
        operator=(AlphaKalmanFilterNode &&otherNode_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC TYPES AND CONSTANTS
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
     * @brief           Fixed-size nominal state for Alpha's SE(3) estimate.
     *
     * The state stores fixed-frame position, a body-to-fixed unit quaternion,
     * fixed-frame linear velocity, and body-frame accelerometer and gyroscope
     * biases in the order declared by StateIndex.
     */
    using NominalStateVector =
        Eigen::Matrix<double,
                      static_cast<Eigen::Index>(StateIndex::STATE_INDEX_COUNT),
                      1>;

    /*!
     * @brief           Fixed-size Euclidean error state corrected by the EKF.
     *
     * The vector stores fixed-frame position and velocity error, body-frame
     * attitude error, and body-frame accelerometer- and gyroscope-bias errors.
     */
    using ErrorStateVector =
        Eigen::Matrix<double,
                      static_cast<Eigen::Index>(
                          ErrorStateIndex::ERROR_STATE_INDEX_COUNT),
                      1>;

    /*!
     * @brief           Error-state covariance and process-noise matrix type.
     *
     * Row and column order matches ErrorStateVector. Diagonal entries use the
     * squared units of their associated error-state components.
     */
    using ErrorStateMatrix = Eigen::Matrix<
        double,
        static_cast<Eigen::Index>(ErrorStateIndex::ERROR_STATE_INDEX_COUNT),
        static_cast<Eigen::Index>(ErrorStateIndex::ERROR_STATE_INDEX_COUNT)>;

    /*!
     * @brief           Per-axis pose and twist measurement-variance vector.
     *
     * The first six entries follow ROS pose covariance order and the final
     * six entries follow ROS twist covariance order.
     */
    using MeasurementVarianceVector = Eigen::Matrix<double, 12, 1>;

    /*!
     * @brief           Two-axis wheel body-velocity observation Jacobian type.
     */
    using WheelVelocityObservationMatrix =
        Eigen::Matrix<double,
                      2,
                      static_cast<Eigen::Index>(
                          ErrorStateIndex::ERROR_STATE_INDEX_COUNT)>;

    /*!
     * @brief           Number of scalar components in the nominal state.
     */
    static constexpr Eigen::Index NOMINAL_STATE_SIZE =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_COUNT);

    /*!
     * @brief           Number of scalar components in the error state.
     */
    static constexpr Eigen::Index ERROR_STATE_SIZE =
        static_cast<Eigen::Index>(ErrorStateIndex::ERROR_STATE_INDEX_COUNT);

    /*! @brief Number of wheels in the retained neutral compatibility output. */
    static constexpr Eigen::Index WHEEL_COUNT = 6;

    /*!
     * @brief           Evaluates the bias-aware nominal process and Jacobian.
     *
     * @param[in]       state_in
     *                  Nominal state at the beginning of the interval.
     * @param[in]       specificForceBodyMps2_in
     *                  Raw body-frame specific force in metres per second
     *                  squared.
     * @param[in]       angularVelocityBodyRadPerS_in
     *                  Raw body-frame angular velocity in radians per second.
     * @param[in]       gravityAccelerationFixedMps2_in
     *                  Physical gravity vector in startup-fixed, in metres per
     *                  second squared.
     * @param[in]       timeStepS_in
     *                  Positive propagation interval in seconds.
     * @param[out]      predictedState_out
     *                  Nominal state at the end of the interval.
     * @param[out]      processJacobian_out
     *                  Continuous 15-by-15 right-error Jacobian.
     */
    static void calculateProcessModel(
        const NominalStateVector &state_in,
        const Eigen::Vector3d    &specificForceBodyMps2_in,
        const Eigen::Vector3d    &angularVelocityBodyRadPerS_in,
        const Eigen::Vector3d    &gravityAccelerationFixedMps2_in,
        double                    timeStepS_in,
        NominalStateVector       &predictedState_out,
        ErrorStateMatrix         &processJacobian_out);

    /*!
     * @brief           Calculates the wheel body-velocity prediction/Jacobian.
     *
     * @param[in]       state_in
     *                  Nominal state containing fixed velocity and attitude.
     * @param[out]      predictedVelocityBodyMps_out
     *                  Predicted body-frame x/y velocity in metres per second.
     * @param[out]      observationMatrix_out
     *                  Two-by-fifteen right-error observation Jacobian.
     */
    static void calculateWheelVelocityObservation(
        const NominalStateVector       &state_in,
        Eigen::Vector2d                &predictedVelocityBodyMps_out,
        WheelVelocityObservationMatrix &observationMatrix_out);

    /*!
     * @brief           Predicts the visual increment measurement and its
     *                  Jacobian: the body-frame velocity and, when a raw
     *                  gyroscope mean is available, the body yaw rate.
     *
     *                  Body velocity is R^T v with R the body-to-fixed
     *                  rotation, so its right-error Jacobian is R^T for the
     *                  velocity error and [R^T v]x for the attitude error.
     *                  The yaw rate is the mean raw gyroscope z rate over
     *                  the visual interval minus the gyroscope z bias, so
     *                  its only Jacobian entry is -1 on that bias error.
     *
     * @param[in]       state_in
     *                  Nominal state at the measurement time.
     *
     * @param[in]       meanRawYawRateRadPerS_in
     *                  Mean raw body z angular rate over the visual
     *                  interval, radians per second; ignored when
     *                  hasYawRate_in is false.
     *
     * @param[in]       hasYawRate_in
     *                  Whether to include the yaw-rate row.
     *
     * @param[out]      predictedMeasurement_out
     *                  (vx, vy, vz[, wz]) in the body frame, metres per
     *                  second and radians per second.
     *
     * @param[out]      observationMatrix_out
     *                  3- or 4-by-15 right-error observation Jacobian.
     */
    static void calculateVisualIncrementObservation(
        const NominalStateVector &state_in,
        double                    meanRawYawRateRadPerS_in,
        bool                      hasYawRate_in,
        Eigen::VectorXd          &predictedMeasurement_out,
        Eigen::MatrixXd          &observationMatrix_out);

    /*!
     * @brief           Calculates the right-multiplicative attitude reset
     *                  Jacobian.
     *
     * After injecting q+ = q Exp(deltaTheta), this maps perturbations from
     * the old body tangent coordinates into the tangent coordinates about q+.
     *
     * @param[in]       attitudeCorrectionBodyRad_in
     *                  Injected body-frame rotation vector in radians.
     * @return          SO(3) right Jacobian of the injected correction.
     */
    static Eigen::Matrix3d calculateAttitudeResetJacobian(
        const Eigen::Vector3d &attitudeCorrectionBodyRad_in);

    /*!
     * @brief           Computes normalized innovation squared with an LDLT
     *                  solve.
     *
     * @return          True only when the innovation covariance is finite and
     *                  positive definite and nis_out was written.
     */
    static bool calculateNormalizedInnovationSquared(
        const Eigen::VectorXd &innovation_in,
        const Eigen::MatrixXd &observationMatrix_in,
        const Eigen::MatrixXd &stateCovariance_in,
        const Eigen::MatrixXd &measurementNoise_in,
        double                &nis_out);

    /*!
     * @brief           Selects a configured or dimension-dependent NIS gate.
     *
     * @param[in]       configuredThreshold_in
     *                  Positive override, or zero for the 99% chi-square gate.
     * @param[in]       measurementDimension_in
     *                  Number of active scalar measurement channels.
     * @return          Positive NIS rejection threshold.
     */
    static double selectNisThreshold(double       configuredThreshold_in,
                                     Eigen::Index measurementDimension_in);

  private:
    /*!
     * @brief           Complete estimator state retained for bounded-lag
     *                  rollback.
     */
    struct FilterCheckpoint
    {
        /*!
         * @brief           Epoch at which the checkpoint is valid.
         *
         * @frame           N/A
         * @units           ROS seconds
         */
        double timestamp_s{0.0};

        /*!
         * @brief           Nominal state at the epoch.
         *
         * @frame           Mixed; see StateIndex
         * @units           Mixed; see StateIndex
         */
        NominalStateVector nominalState{NominalStateVector::Zero()};

        /*!
         * @brief           Error-state mean at the epoch.
         *
         * @frame           Mixed; see ErrorStateIndex
         * @units           Mixed; see ErrorStateIndex
         */
        ErrorStateVector errorState{ErrorStateVector::Zero()};

        /*!
         * @brief           Error-state covariance at the epoch.
         *
         * @frame           Mixed; see ErrorStateIndex
         * @units           Squared error-state units
         */
        ErrorStateMatrix covariance{ErrorStateMatrix::Identity()};
    };

    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Calibrates from or propagates with one raw IMU sample.
     *
     * Raw specific force and angular rate are corrected by the nominal bias
     * states. The callback executes in the node's mutually exclusive default
     * callback group.
     *
     * @param[in]       message_in
     *                  Raw IMU message. Specific force is in metres per second
     *                  squared and angular velocity is in radians per second,
     *                  both in the body frame.
     */
    void handleImuCallBack(const sensor_msgs::msg::Imu &message_in);

    /*!
     * @brief           Applies one odometry measurement to the EKF, using
     *                   the observation matrix that source can actually
     *                   observe.
     *
     * Measurements arriving before the IMU-seeded initial state exists are
     * rejected. A measurement stamped before the current filter epoch rolls
     * the filter back to its stamp, is fused there, and then every
     * measurement already fused after that stamp is replayed in order (see
     * replayMeasurementsAfter()), so a lagged source never erases newer
     * updates from the estimate.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source.
     * @param[in]       kind_in
     *                  Selects the observation matrix, minimum variance, and
     *                  (for visual odometry) age-compensation behaviour.
     */
    void handleMeasurementCallBack(const nav_msgs::msg::Odometry &message_in,
                                   MeasurementKind                kind_in);

    /*!
     * @brief           Publishes per-source counters, timing and filter
     *                  health on the diagnostics topic and, every fifth
     *                  call, a compact console health line.
     *
     *                  Runs from a simulation-time timer once per second in
     *                  the node's mutually exclusive default callback group,
     *                  so it reads the filter state without locking.
     */
    void publishDiagnosticsCallBack();

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
    static void setNoiseTriplet(ErrorStateMatrix &noise_inout,
                                Eigen::Index      firstIndex_in,
                                double            variance_in);

    /*!
     * @brief           Evaluates Alpha's nominal dynamics and error Jacobian.
     *
     * Position and velocity are expressed in startup-fixed. Raw body-frame
     * specific force and angular rate are corrected by nominal bias states.
     *
     * @param[in]       state_in
     *                  Nominal state at which the model is evaluated.
     * @param[in]       specificForceBodyMps2_in
     *                  Raw body-frame specific force in metres per second
     *                  squared.
     * @param[in]       angularVelocityBodyRadPerS_in
     *                  Raw body-frame angular velocity in radians per second.
     * @param[in]       timeStepS_in
     *                  Integration interval in seconds.
     * @param[out]      predictedState_out
     *                  Nominal state propagated through timeStepS_in.
     * @param[out]      processJacobian_out
     *                  Continuous 15-by-15 error-state Jacobian.
     */
    void computeProcessModel(
        const NominalStateVector &state_in,
        const Eigen::Vector3d    &specificForceBodyMps2_in,
        const Eigen::Vector3d    &angularVelocityBodyRadPerS_in,
        double                    timeStepS_in,
        NominalStateVector       &predictedState_out,
        ErrorStateMatrix         &processJacobian_out) const;

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
    FilterStatus predictTo(double targetTimestampS_in,
                           bool   shouldSaveCheckpoints_in = true);

    /*!
     * @brief           Initializes an identity nominal state and covariance.
     *
     * @param[in]       timestampS_in
     *                  Sensor epoch at which the identity state is valid, in
     *                  ROS seconds.
     *
     * @return          Filter lifecycle status.
     */
    [[nodiscard]] FilterStatus initializeFilter(double timestampS_in);

    /*!
     * @brief           Saves or replaces the checkpoint at the current filter
     *                  epoch.
     */
    void saveFilterCheckpoint();

    /*!
     * @brief           Restores the newest checkpoint no later than a target.
     * @return          True when a suitable valid checkpoint was restored.
     */
    [[nodiscard]] bool
        restoreFilterCheckpointAtOrBefore(double targetTimestampS_in);

    /*!
     * @brief           Discards checkpoints newer than the supplied epoch.
     *
     * @param[in]       targetTimestampS_in
     *                  Epoch after which checkpoints are discarded, ROS
     *                  seconds.
     */
    void discardFilterCheckpointsAfter(double targetTimestampS_in) noexcept;

    /*!
     * @brief           Removes all retained rollback checkpoints.
     */
    void clearFilterCheckpoints() noexcept;

    /*!
     * @brief           Converts one admitted odometry message into a
     *                  replayable measurement record.
     *
     *                  Only state-independent values are captured: the
     *                  measured quantities, their noise variances and, for
     *                  a visual increment, the mean raw gyroscope z over
     *                  the increment interval from the IMU buffer.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source.
     * @param[in]       kind_in
     *                  Source that produced message_in.
     * @param[in]       measurementTimestampS_in
     *                  Header stamp of message_in, ROS seconds.
     * @param[in]       intervalStartS_in
     *                  Previous visual stamp bounding an increment's
     *                  gyroscope window, ROS seconds; ignored unless kind_in
     *                  is visual and increment mode is active.
     * @param[out]      record_out
     *                  Record written only when this method returns true.
     *
     * @return          True when every measured value was finite.
     */
    [[nodiscard]] bool
        buildMeasurementRecord(const nav_msgs::msg::Odometry &message_in,
                               MeasurementKind                kind_in,
                               double            measurementTimestampS_in,
                               double            intervalStartS_in,
                               FusedMeasurement &record_out) const;

    /*!
     * @brief           Gates and applies one measurement record at the
     *                  current filter epoch.
     *
     *                  The innovation and observation matrix are evaluated
     *                  against the current nominal state, so the caller must
     *                  first bring the filter to the record's timestamp.
     *
     * @param[in]       record_in
     *                  Measurement record to apply.
     * @param[out]      nis_out
     *                  Normalized innovation squared, written whenever the
     *                  innovation covariance was valid.
     * @param[out]      correctionNorm_out
     *                  Norm of the posterior error state, written only when
     *                  the update succeeded.
     *
     * @return          Fused, NIS-rejected (filter unchanged) or
     *                  numerically rejected.
     */
    [[nodiscard]] MeasurementFusionResult
        fuseMeasurementRecord(const FusedMeasurement &record_in,
                              double                 &nis_out,
                              double                 &correctionNorm_out);

    /*!
     * @brief           Re-applies every retained measurement stamped after
     *                  a rollback epoch, then propagates to the present.
     *
     *                  Called after a delayed measurement was fused at its
     *                  own epoch. Without this, the measurements already
     *                  fused between that epoch and the present would be
     *                  lost from the estimate until the next update.
     *                  Checkpoints are saved as the replay advances. A
     *                  record that the gate now rejects is skipped but kept
     *                  for later replays.
     *
     * @param[in]       rollbackTimestampS_in
     *                  Epoch of the delayed measurement just fused, ROS
     *                  seconds; only records strictly after it replay.
     * @param[in]       presentTimestampS_in
     *                  Filter epoch before the rollback, ROS seconds.
     *
     * @return          Success, or the first prediction or numerical
     *                  failure; the caller then restores the present state.
     */
    [[nodiscard]] FilterStatus
        replayMeasurementsAfter(double rollbackTimestampS_in,
                                double presentTimestampS_in);

    /*!
     * @brief           Injects one posterior error into the nominal state.
     *
     * Position, velocity and both sensor biases use additive correction. The
     * attitude correction is converted to a quaternion and composed on the
     * right, so its three components are expressed in the body frame.
     * Before the error-state mean is reset, its covariance is transformed into
     * the tangent coordinates about the corrected nominal attitude.
     *
     * @return          Filter status produced while resetting the error mean.
     */
    [[nodiscard]] FilterStatus injectErrorState();

    /*!
     * @brief           Computes the body-frame attitude innovation.
     *
     * @param[in]       measuredQuaternion_in
     *                  Measured unit quaternion rotating body components into
     *                  map components.
     * @param[in]       predictedQuaternion_in
     *                  Predicted unit quaternion rotating body components into
     *                  map components.
     *
     * @return          Three-component logarithmic attitude residual in
     *                  radians, expressed in the body tangent frame.
     */
    static Eigen::Vector3d calculateQuaternionError(
        const Eigen::Quaterniond &measuredQuaternion_in,
        const Eigen::Quaterniond &predictedQuaternion_in);

    /*!
     * @brief           Builds a one-hot error-state observation matrix.
     *
     * @param[in]       observedIndices_in
     *                  State indices this observation matrix selects, in
     *                  row order.
     * @param[in]       stateSize_in
     *                  Number of error-state columns.
     * @return          observedIndices_in.size() x stateSize_in observation
     *                  matrix.
     */
    static Eigen::MatrixXd buildObservationMatrix(
        const std::vector<Eigen::Index> &observedIndices_in,
        Eigen::Index                     stateSize_in);

    /*!
     * @brief           Converts one fixed-frame odometry message to state.
     *
     * Visual and wheel odometry both start at identity and publish directly in
     * startup-fixed. Their poses therefore require no truth-derived rebasing.
     *
     * @param[in]       message_in
     *                  Odometry measurement from one upstream source, with
     *                  twist expressed in the child (body) frame per ROS
     *                  convention.
     * @return          Nominal vector with fixed-frame position, body-to-fixed
     *                  quaternion and fixed-frame position populated. Other
     *                  components remain zero.
     */
    NominalStateVector
        odometryToState(const nav_msgs::msg::Odometry &message_in) const;

    /*!
     * @brief           Wraps an angle into the canonical [-pi, pi] range.
     *
     * @param[in]       angleRad_in
     *                  Angle in radians.
     * @return          Equivalent angle in radians within [-pi, pi].
     */
    static double wrapAngle(double angleRad_in);

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
    static MeasurementVarianceVector
        measurementVariances(const nav_msgs::msg::Odometry &message_in,
                             double                         minimumVariance_in);

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
     * @brief           Publishes the propagated state once, stamped with
     *                  its own timestamp, unless that stamp was already
     *                  published or is closer than the minimum output
     *                  period to the previous publication.
     */
    void publishPropagatedEstimate();

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
     * @brief           Publishes a neutral no-slip compatibility message.
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
     * @brief           Fixed checkpoint capacity, matching the retained IMU
     *                  history.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::size_t FILTER_CHECKPOINT_CAPACITY = 512U;

    /*!
     * @brief           Owned continuous-discrete error-state EKF engine.
     *
     * The engine stores the zero-centred 15-component Euclidean error state
     * and its covariance. The node's mutually exclusive callback group is the
     * sole synchronization mechanism protecting it.
     *
     * @frame           Mixed; see ErrorStateIndex
     * @units           Mixed; see ErrorStateIndex
     */
    Filter filter;

    /*!
     * @brief           Current nominal pose, velocity, and IMU-bias estimate.
     *
     * This state is propagated separately from the Euclidean error state and
     * receives each posterior correction through injectErrorState().
     *
     * @frame           Mixed; see StateIndex
     * @units           Mixed; see StateIndex
     */
    NominalStateVector nominalState{NominalStateVector::Zero()};

    /*!
     * @brief           Most recently cached nominal state for publication.
     *
     * @frame           Mixed; see StateIndex
     * @units           Mixed; see StateIndex
     */
    NominalStateVector latestState{NominalStateVector::Zero()};

    /*!
     * @brief           Most recently cached error-state covariance.
     *
     * @frame           Mixed; see ErrorStateIndex
     * @units           Squared error-state units
     */
    ErrorStateMatrix latestCovariance{ErrorStateMatrix::Identity()};

    /*!
     * @brief           Continuous error-state process-noise density.
     *
     * @frame           Mixed; see ErrorStateIndex
     * @units           Squared error-state units per second
     */
    ErrorStateMatrix processNoise{ErrorStateMatrix::Zero()};

    /*!
     * @brief           ROS timestamp at which the nominal state and error
     *                  covariance are valid.
     *
     * @frame           N/A
     * @units           seconds
     */
    double stateTimestamp_s{0.0};

    /*!
     * @brief           Recent raw IMU samples retained for prediction.
     *
     * Samples are copied from ROS messages and consumed chronologically by
     * predictTo(). The node's mutually exclusive callback group serializes
     * all access, so the buffer requires no internal lock.
     *
     * @frame           body
     * @units           seconds, metres per second squared, radians per second
     */
    ImuRingBuffer imuBuffer;

    /*!
     * @brief           Fixed-capacity chronological rollback checkpoints.
     *
     * @frame           Mixed; see FilterCheckpoint
     * @units           Mixed; see FilterCheckpoint
     */
    std::array<FilterCheckpoint, FILTER_CHECKPOINT_CAPACITY>
        filterCheckpoints{};

    /*!
     * @brief           Storage index where the next checkpoint is written.
     *
     * @frame           N/A
     * @units           index
     */
    std::size_t nextFilterCheckpointIndex{0U};

    /*!
     * @brief           Storage index of the oldest retained checkpoint.
     *
     * @frame           N/A
     * @units           index
     */
    std::size_t oldestFilterCheckpointIndex{0U};

    /*!
     * @brief           Number of valid retained checkpoints.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t filterCheckpointCount{0U};

    /*!
     * @brief           Recently fused visual and wheel measurements, in
     *                  timestamp order, replayed after a rollback.
     *
     * @frame           Mixed; see FusedMeasurement
     * @units           Mixed; see FusedMeasurement
     */
    MeasurementHistory measurementHistory;

    /*!
     * @brief       Publishes the fused odometry estimate.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_outputPublisher;

    /*!
     * @brief           Subscribes to inertial_odometry's filtered IMU output.
     *
     * @frame           body
     * @units           metres per second squared and radians per second
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr p_imuSubscription;

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
     * @brief           Shortest simulated interval between two published
     *                  estimates (the inverse of prediction_rate_hz).
     *
     * @frame           N/A
     * @units           seconds
     */
    double minimumOutputPeriodS{0.01};

    /*!
     * @brief           State timestamp of the latest published estimate;
     *                  negative before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double lastPublishedStateTimestamp_s{-1.0};

    /*!
     * @brief           Simulation-time timer driving
     *                  publishDiagnosticsCallBack() once per second.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;

    /*!
     * @brief           Publishes periodic estimator diagnostics on the
     *                  system's shared diagnostics topic.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsPublisher;

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
    double wheelVariance{0.10};

    /*!
     * @brief           Known lunar gravitational-acceleration magnitude.
     *
     * @frame           N/A
     * @units           metres per second squared
     */
    double gravityMagnitudeMps2{1.62};

    /*!
     * @brief           Physical gravitational acceleration.
     *
     * @frame           startup-fixed
     * @units           metres per second squared
     */
    Eigen::Vector3d gravityAcceleration_fixed_mPerS2{
        Eigen::Vector3d(0.0, 0.0, -1.62)};

    /*!
     * @brief           Sum of stationary raw specific-force initialization
     *                  samples.
     *
     * @frame           body
     * @units           metres per second squared
     */
    Eigen::Vector3d initializationSpecificForceSum_body_mPerS2{
        Eigen::Vector3d::Zero()};

    /*!
     * @brief           Sum of stationary raw angular-rate initialization
     *                  samples.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d initializationAngularVelocitySum_body_radPerS{
        Eigen::Vector3d::Zero()};

    /*!
     * @brief           Required number of stationary initialization samples.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t imuInitializationSampleTarget{100U};

    /*!
     * @brief           Number of stationary initialization samples received.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t imuInitializationSampleCount{0U};

    /*!
     * @brief           Initial accelerometer-bias covariance diagonal.
     *
     * @frame           body
     * @units           (metres per second squared)^2
     */
    double initialAccelerometerBiasVariance{0.01};

    /*!
     * @brief           Initial gyroscope-bias covariance diagonal.
     *
     * @frame           body
     * @units           (radians per second)^2
     */
    double initialGyroscopeBiasVariance{0.001};

    /*!
     * @brief           Latest bias-corrected body angular velocity for
     *                  publication.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d latestAngularVelocity_body_radPerS{Eigen::Vector3d::Zero()};

    /*!
     * @brief       Maximum accepted visual-odometry measurement age in seconds.
     */
    double maximumVisualMeasurementAgeS{0.75};

    /*!
     * @brief           Whether visual odometry is fused as per-interval
     *                  motion increments rather than absolute poses.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isVisualIncrementMode{false};

    /*!
     * @brief           Variance floor of an increment-mode visual body
     *                  velocity.
     *
     * @frame           body
     * @units           (metres per second)^2
     */
    double visualIncrementVelocityVariance{1.0e-6};

    /*!
     * @brief           Variance floor of an increment-mode visual yaw rate.
     *
     * @frame           body
     * @units           (radians per second)^2
     */
    double visualIncrementYawRateVariance{1.0e-6};

    /*!
     * @brief           Stamp of the previous visual message, the start of
     *                  the next visual increment's interval; negative before
     *                  the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double previousVisualStamp_s{-1.0};

    /*!
     * @brief           Largest lead of a measurement stamp over this node's
     *                  clock that is still fused.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumFutureStampS{0.02};

    /*!
     * @brief           Maximum permitted age of a held IMU sample.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumImuMeasurementAgeS{0.25};

    /*!
     * @brief           Visual update NIS override; zero selects the automatic
     *                  limit.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double visualNisThreshold{0.0};

    /*!
     * @brief           Wheel update NIS override; zero selects the automatic
     *                  limit.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double wheelNisThreshold{0.0};

    /*!
     * @brief           Whether visual pose channels are fused.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool shouldFuseVisualPose{true};

    /*!
     * @brief           Whether wheel body-twist channels are fused.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool shouldFuseWheelTwist{true};

    /*!
     * @brief           Diagnostics for raw IMU process input.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics imuDiagnostics;

    /*!
     * @brief           Diagnostics for visual odometry corrections.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics visualDiagnostics;

    /*!
     * @brief           Diagnostics for wheel odometry corrections.
     *
     * @frame           N/A
     * @units           Mixed; see SourceDiagnostics
     */
    SourceDiagnostics wheelDiagnostics;

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
     * @brief           Number of diagnostics records between console health
     *                  lines: one line every five simulated seconds.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::uint64_t CONSOLE_HEALTH_PERIOD_TICKS = 5U;

    /*!
     * @brief           Number of diagnostics records published.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t diagnosticsRecordCount{0U};

    /*!
     * @brief           Visual fused count at the previous console health
     *                  line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleVisualFusedCount{0U};

    /*!
     * @brief           Wheel fused count at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleWheelFusedCount{0U};

    /*!
     * @brief           Visual and wheel rejections (age, NIS and numerical)
     *                  at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleRejectedCount{0U};

    /*!
     * @brief           Visual updates that must be fused after
     *                  initialisation before this node reports itself
     *                  ready.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t readinessMinimumVisualUpdates{3U};

    /*!
     * @brief           Shortest time after initialisation before this node
     *                  reports itself ready.
     *
     * @frame           N/A
     * @units           seconds
     */
    double readinessSettleTimeS{5.0};

    /*!
     * @brief           Largest position-covariance trace for which this
     *                  node still reports itself ready (divergence guard).
     *
     * @frame           startup-fixed
     * @units           square metres
     */
    double readinessMaximumPositionVarianceM2{1.0};

    /*!
     * @brief           Stamp of the IMU sample that completed the latest
     *                  initialisation.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double initializationTimestamp_s{0.0};

    /*!
     * @brief           Visual fused count when the filter was last
     *                  initialised, so readiness counts only updates since.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t visualFusedCountAtInitialization{0U};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_KALMAN_FILTER_NODE_H */
