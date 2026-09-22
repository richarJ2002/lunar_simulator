/*!
 * @File:         InertialOdometryNode.h
 *
 * @Brief:        Declares the IMU-based dead-reckoning odometry node.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_INERTIAL_ODOMETRY_NODE_H
#define LUNAR_SIMULATOR_LOCALISATION_INERTIAL_ODOMETRY_NODE_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

/* Generic Libraries */
#include <array>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace localisation::inertial_odometry
{

/*!
 * @brief           Filters and integrates Alpha rover IMU measurements.
 *
 *                  The node low-pass filters raw angular rate and linear
 *                  acceleration, estimates a stationary accelerometer/gyro bias
 *                  during a startup calibration window, integrates the
 *                  body-frame quaternion, removes lunar gravity, and
 *                  double-integrates velocity and position. Pure inertial
 *                  integration drifts without external correction; bias
 *                  calibration assumes the rover is stationary when the node
 *                  starts. It is intended for a single-threaded executor.
 */
class InertialOdometryNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters and configures IMU I/O.
     */
    InertialOdometryNode() :
        Node("inertial_odometry"),
        orientation(0.0, 0.0, 0.0, 1.0)
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: the noisy IMU republished by the sensor node. */
        const std::string imuTopic =
            declare_parameter<std::string>("imu_topic",
                                           "/" + systemName + "/imu");

        /* Output topic: this node's integrated odometry estimate. */
        const std::string odometryTopic = declare_parameter<std::string>(
            "odometry_topic",
            "/" + systemName + "/localisation/inertial/odometry");

        /* Output topic: the bias-corrected, filtered IMU sample. */
        const std::string filteredImuTopic = declare_parameter<std::string>(
            "filtered_imu_topic",
            "/" + systemName + "/localisation/inertial/filtered_imu");

        /* Fixed world frame shared with Gazebo and every other node. */
        odomFrame = declare_parameter<std::string>(
            "odom_frame", systemName + "/startup_fixed");

        /* Rover body frame this node's odometry describes. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Low-pass filter cutoff frequency in Hz. */
        cutoffHz = declare_parameter<double>("low_pass_cutoff_hz", 5.0);

        /* Local gravity magnitude to remove from acceleration, in m/s^2. */
        gravityMps2 = declare_parameter<double>("gravity_mps2", 1.62);

        /* Whether local gravity is subtracted from acceleration at all. */
        removeGravity = declare_parameter<bool>("remove_gravity", true);

        /* Largest inter-sample dt accepted before a sample is dropped. */
        maximumDtS = declare_parameter<double>("maximum_dt_s", 0.25);

        /* Number of startup samples averaged into the stationary bias;
         * clamp to at least one so calibration can always complete. */
        calibrationSampleTarget = static_cast<int>(std::max<std::int64_t>(
            1,
            declare_parameter<std::int64_t>("calibration_samples", 100)));

        /* Odom-frame acceleration deadband in m/s^2. */
        accelerationDeadbandMps2 =
            declare_parameter<double>("acceleration_deadband_mps2", 0.02);

        /* Body-frame angular-rate deadband in rad/s. */
        angularRateDeadbandRadps =
            declare_parameter<double>("angular_rate_deadband_radps", 0.002);

        /* Publish integrated odometry with default reliable QoS. */
        odometryPublisher =
            create_publisher<nav_msgs::msg::Odometry>(odometryTopic,
                                                      rclcpp::QoS(10));

        /* Publish the filtered IMU sample as best-effort sensor data. */
        filteredImuPublisher =
            create_publisher<sensor_msgs::msg::Imu>(filteredImuTopic,
                                                    rclcpp::SensorDataQoS());

        /* Every incoming raw IMU message triggers handleImuCallBack(). */
        imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr p_message)
            { handleImuCallBack(*p_message); });

        /* Record the resolved topics and tuning once at start-up for
         * operators inspecting the node's log. */
        RCLCPP_INFO(get_logger(),
                    "IMU calibration requires a stationary rover for %d "
                    "samples; filtered samples are temporally correlated",
                    calibrationSampleTarget);
        RCLCPP_INFO(get_logger(),
                    "IMU odometry: %s -> %s (LPF %.1f Hz, gravity %.2f m/s^2)",
                    imuTopic.c_str(),
                    odometryTopic.c_str(),
                    cutoffHz,
                    gravityMps2);
    }

    /*!
     * @brief           Forms the fixed-frame stationary specific-force prior.
     *
     * @param[in]       stationaryMeanBody_in
     *                  Mean stationary accelerometer sample in body axes.
     * @param[in]       gravityMagnitudeMps2_in
     *                  Known local gravity magnitude in metres per second
     *                  squared.
     *
     * @return          Gravity-specific-force vector in startup-fixed. At
     *                  initialization body and fixed axes coincide.
     */
    static tf2::Vector3 calculateGravitySpecificForceFixed(
        const tf2::Vector3 &stationaryMeanBody_in,
        double gravityMagnitudeMps2_in);

    /*!
     * @brief           Removes fixed-frame gravity-specific force.
     *
     * @param[in]       orientationBodyToFixed_in
     *                  Unit quaternion rotating body vectors into fixed.
     * @param[in]       specificForceBody_in
     *                  Bias-corrected accelerometer specific force in body.
     * @param[in]       gravitySpecificForceFixed_in
     *                  Constant stationary specific-force vector in fixed.
     *
     * @return          Gravity-free acceleration in startup-fixed.
     */
    static tf2::Vector3 calculateGravityFreeAccelerationFixed(
        const tf2::Quaternion &orientationBodyToFixed_in,
        const tf2::Vector3 &specificForceBody_in,
        const tf2::Vector3 &gravitySpecificForceFixed_in);

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Filters, integrates and publishes one IMU sample.
     *
     * During the startup calibration window, samples are accumulated into a
     * stationary bias estimate instead of being integrated. Once
     * calibration completes, this filters angular rate and acceleration,
     * integrates orientation and then position/velocity, and publishes the
     * filtered IMU and odometry outputs.
     *
     * @param[in]       message Raw IMU measurement in the body frame.
     */
    void handleImuCallBack(const sensor_msgs::msg::Imu &message);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Converts a ROS timestamp to seconds.
     * @param[in]       stamp Timestamp to convert.
     * @return          Timestamp in seconds.
     */
    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp);

    /*!
     * @brief           Converts a tf2 quaternion to a message quaternion.
     * @param[in]       quaternion Quaternion to convert.
     * @return          Equivalent `geometry_msgs::msg::Quaternion`.
     */
    static geometry_msgs::msg::Quaternion
        toMessage(const tf2::Quaternion &quaternion);

    /*!
     * @brief           Publishes the bias-corrected, filtered IMU sample.
     * @param[in]       input Original IMU message providing header/covariance
     *                  fields to reuse.
     * @param[in]       accelerationOdom Gravity-adjusted acceleration
     *                  expressed in the odom frame, in m/s^2.
     */
    void publishFilteredImu(const sensor_msgs::msg::Imu &input,
                            const tf2::Vector3          &accelerationOdom);

    /*!
     * @brief           Publishes the current integrated odometry estimate.
     * @param[in]       stamp Timestamp applied to the published message.
     */
    void publishOdometry(const builtin_interfaces::msg::Time &stamp);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Publishes integrated inertial odometry.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

    /*!
     * @brief       Publishes the bias-corrected, filtered IMU sample.
     */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filteredImuPublisher;

    /*!
     * @brief       Receives raw (noisy) IMU measurements.
     */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;

    /*!
     * @brief       Fixed frame in which integrated odometry is expressed.
     */
    std::string odomFrame;

    /*!
     * @brief       Rover body frame that odometry describes.
     */
    std::string baseFrame;

    /*!
     * @brief       Low-pass filter cutoff frequency in Hz.
     */
    double cutoffHz{5.0};

    /*!
     * @brief       Local gravity magnitude removed from acceleration, in m/s^2.
     */
    double gravityMps2{1.62};

    /*!
     * @brief       Maximum accepted inter-sample duration in seconds.
     */
    double maximumDtS{0.25};

    /*!
     * @brief       Odom-frame acceleration deadband in m/s^2.
     */
    double accelerationDeadbandMps2{0.02};

    /*!
     * @brief       Body-frame angular-rate deadband in rad/s.
     */
    double angularRateDeadbandRadps{0.002};

    /*!
     * @brief       Whether local gravity is subtracted from odom-frame
     *              acceleration.
     */
    bool removeGravity{true};

    /*!
     * @brief       True once one IMU sample has been observed.
     */
    bool hasPreviousStamp{false};

    /*!
     * @brief       Timestamp in seconds of the previously processed sample.
     */
    double previousStampS{0.0};

    /*!
     * @brief       Low-pass filtered, bias-corrected body-frame acceleration in
     *              m/s^2.
     */
    std::array<double, 3> filteredAcceleration{0.0, 0.0, 0.0};

    /*!
     * @brief       Low-pass filtered, bias-corrected body-frame angular rate in
     *              rad/s.
     */
    std::array<double, 3> filteredAngularRate{0.0, 0.0, 0.0};

    /*!
     * @brief       Running sum of raw acceleration during calibration, in
     *              m/s^2.
     */
    std::array<double, 3> accelerationCalibrationSum{0.0, 0.0, 0.0};

    /*!
     * @brief       Running sum of raw angular rate during calibration, in
     *              rad/s.
     */
    std::array<double, 3> angularCalibrationSum{0.0, 0.0, 0.0};

    /*!
     * @brief       Estimated stationary accelerometer bias in m/s^2.
     */
    std::array<double, 3> accelerationBias{0.0, 0.0, 0.0};

    /*!
     * @brief       Estimated stationary gyroscope bias in rad/s.
     */
    std::array<double, 3> angularRateBias{0.0, 0.0, 0.0};

    /*!
     * @brief       Constant stationary specific force in startup-fixed,
     *              metres per second squared. Its direction is measured
     *              during calibration and need not align with fixed Z.
     */
    tf2::Vector3 gravitySpecificForceFixedMps2{0.0, 0.0, 0.0};

    /*!
     * @brief       Integrated odom-frame velocity in m/s.
     */
    std::array<double, 3> velocityMps{0.0, 0.0, 0.0};

    /*!
     * @brief       Integrated odom-frame position in m.
     */
    std::array<double, 3> positionM{0.0, 0.0, 0.0};

    /*!
     * @brief       Number of samples used to estimate the stationary bias.
     */
    int calibrationSampleTarget{100};

    /*!
     * @brief       Number of calibration samples observed so far.
     */
    int calibrationSampleCount{0};

    /*!
     * @brief       Integrated body-to-odom orientation quaternion.
     */
    tf2::Quaternion orientation;
};

} /* namespace localisation::inertial_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_INERTIAL_ODOMETRY_NODE_H */
