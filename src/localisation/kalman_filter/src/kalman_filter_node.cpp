/*!
 * @File:         kalman_filter_node.cpp
 *
 * @Brief:        Connects Alpha odometry measurements to the reusable EKF.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "kalman_filter/objects/ContinuousExtendedKalmanFilter.h"

/* Data include */
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

/* Generic Libraries */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace lunar_simulator::localisation
{

class KalmanFilterNode final : public rclcpp::Node
{
  public:
    using Filter = kalman_filter::ContinuousExtendedKalmanFilter;
    using FilterStatus = kalman_filter::FilterStatus;

    KalmanFilterNode() : Node("continuous_ekf")
    {
        const std::string inertialTopic = declare_parameter<std::string>(
            "inertial_odometry_topic", "/localisation/inertial/odometry");
        const std::string visualTopic = declare_parameter<std::string>(
            "visual_odometry_topic", "/localisation/visual/odometry");
        const std::string wheelTopic = declare_parameter<std::string>(
            "wheel_odometry_topic", "/localisation/wheel/odometry");
        const std::string outputTopic = declare_parameter<std::string>(
            "output_topic", "/localisation/kalman_filter/odometry");
        odomFrame = declare_parameter<std::string>("odom_frame", "map");
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");
        const double initialPositionXM =
            declare_parameter<double>("initial_position_x_m", 0.0);
        const double initialPositionYM =
            declare_parameter<double>("initial_position_y_m", 0.0);
        const double initialPositionZM =
            declare_parameter<double>("initial_position_z_m", 0.72);
        const double initialRollRad =
            declare_parameter<double>("initial_roll_rad", 0.0);
        const double initialPitchRad =
            declare_parameter<double>("initial_pitch_rad", 0.0);
        const double initialYawRad =
            declare_parameter<double>("initial_yaw_rad", 0.0);
        const double predictionRateHz =
            declare_parameter<double>("prediction_rate_hz", 100.0);
        maximumVisualMeasurementAgeS = declare_parameter<double>(
            "maximum_visual_measurement_age_s", 0.5);
        if (maximumVisualMeasurementAgeS <= 0.0)
        {
            throw std::invalid_argument(
                "maximum_visual_measurement_age_s must be positive");
        }
        publishTf = declare_parameter<bool>("publish_tf", false);
        inertialVariance =
            declare_parameter<double>("inertial_measurement_variance", 0.20);
        visualVariance =
            declare_parameter<double>("visual_measurement_variance", 0.03);
        wheelVariance =
            declare_parameter<double>("wheel_measurement_variance", 0.08);

        Filter::StateMatrix processNoise = Filter::StateMatrix::Zero();
        setNoiseTriplet(
            processNoise, 0,
            declare_parameter<double>("process_noise_position", 0.02));
        setNoiseTriplet(
            processNoise, 3,
            declare_parameter<double>("process_noise_orientation", 0.02));
        setNoiseTriplet(
            processNoise, 6,
            declare_parameter<double>("process_noise_linear_velocity", 0.20));
        setNoiseTriplet(
            processNoise, 9,
            declare_parameter<double>("process_noise_angular_velocity", 0.10));
        const FilterStatus initStatus =
            filter.init(processNoise, Filter::StateMatrix::Identity());
        if (initStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            throw std::runtime_error("Continuous EKF initialization failed");
        }
        initialOrientation.setRPY(initialRollRad, initialPitchRad,
                                  initialYawRad);
        initialOrientation.normalize();
        initialPositionMapM.setValue(initialPositionXM, initialPositionYM,
                                     initialPositionZM);

        outputPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputTopic, rclcpp::QoS(10));
        if (publishTf)
        {
            transformBroadcaster =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        inertialSubscription = create_subscription<nav_msgs::msg::Odometry>(
            inertialTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleMeasurement(*p_message, MeasurementKind::inertial); });
        visualSubscription = create_subscription<nav_msgs::msg::Odometry>(
            visualTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleMeasurement(*p_message, MeasurementKind::visual); });
        wheelSubscription = create_subscription<nav_msgs::msg::Odometry>(
            wheelTopic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr p_message)
            { handleMeasurement(*p_message, MeasurementKind::wheel); });

        const double safeRateHz = std::max(1.0, predictionRateHz);
        predictionTimer = create_wall_timer(
            std::chrono::duration<double>(1.0 / safeRateHz),
            [this]()
            {
                if (!hasInitialState)
                {
                    return;
                }
                const rclcpp::Time stamp = now();
                const Filter::MeasurementMask emptyMask{};
                const Filter::StateVector unusedVariances =
                    Filter::StateVector::Ones();
                const FilterStatus status =
                    filter.step(stamp.seconds(), nullptr, emptyMask,
                                unusedVariances, latestState,
                                latestCovariance);
                if (status == FilterStatus::FILTER_STATUS_SUCCESS)
                {
                    hasEstimate = true;
                    publishEstimate(stamp);
                }
                else
                {
                    logStepFailure(status);
                }
            });

        RCLCPP_INFO(get_logger(),
                    "Continuous EKF fusing inertial, visual and wheel odometry "
                    "at %.1f Hz",
                    safeRateHz);
    }

    ~KalmanFilterNode() noexcept override
    {
        const FilterStatus status = filter.terminate();
        if (status != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Continuous EKF termination failed");
        }
    }

  private:
    enum class MeasurementKind
    {
        inertial,
        visual,
        wheel
    };

    static void setNoiseTriplet(Filter::StateMatrix &noise_inout,
                                Eigen::Index firstIndex_in, double variance_in)
    {
        for (Eigen::Index index = firstIndex_in; index < firstIndex_in + 3;
             ++index)
        {
            noise_inout(index, index) = std::max(1.0e-9, variance_in);
        }
    }

    Filter::StateVector
    odometryToState(const nav_msgs::msg::Odometry &message_in) const
    {
        Filter::StateVector measurement = Filter::StateVector::Zero();
        const tf2::Vector3 relativePositionM(message_in.pose.pose.position.x,
                                             message_in.pose.pose.position.y,
                                             message_in.pose.pose.position.z);
        const tf2::Vector3 positionMapM =
            initialPositionMapM +
            tf2::Matrix3x3(initialOrientation) * relativePositionM;
        measurement(0) = positionMapM.x();
        measurement(1) = positionMapM.y();
        measurement(2) = positionMapM.z();

        tf2::Quaternion relativeOrientation;
        tf2::fromMsg(message_in.pose.pose.orientation, relativeOrientation);
        relativeOrientation.normalize();
        const tf2::Quaternion orientationMap =
            initialOrientation * relativeOrientation;
        double rollRad = 0.0;
        double pitchRad = 0.0;
        double yawRad = 0.0;
        tf2::Matrix3x3(orientationMap).getRPY(rollRad, pitchRad, yawRad);
        measurement(3) = rollRad;
        measurement(4) = pitchRad;
        measurement(5) = yawRad;

        const tf2::Vector3 velocityBody(message_in.twist.twist.linear.x,
                                        message_in.twist.twist.linear.y,
                                        message_in.twist.twist.linear.z);
        const tf2::Vector3 velocityOdom =
            tf2::Matrix3x3(orientationMap) * velocityBody;
        measurement(6) = velocityOdom.x();
        measurement(7) = velocityOdom.y();
        measurement(8) = velocityOdom.z();
        measurement(9) = message_in.twist.twist.angular.x;
        measurement(10) = message_in.twist.twist.angular.y;
        measurement(11) = message_in.twist.twist.angular.z;
        return measurement;
    }

    static double wrapAngle(double angleRad_in)
    {
        return std::atan2(std::sin(angleRad_in), std::cos(angleRad_in));
    }

    static void compensateMeasurementAge(Filter::StateVector &state_inout,
                                         double ageS_in)
    {
        state_inout.segment<3>(0) += state_inout.segment<3>(6) * ageS_in;
        const double rollRad = state_inout(3);
        const double pitchRad =
            std::clamp(state_inout(4), -1.5533, 1.5533);
        const double sineRoll = std::sin(rollRad);
        const double cosineRoll = std::cos(rollRad);
        const double tangentPitch = std::tan(pitchRad);
        const double secantPitch = 1.0 / std::cos(pitchRad);
        const double rollRateRadps =
            state_inout(9) + sineRoll * tangentPitch * state_inout(10) +
            cosineRoll * tangentPitch * state_inout(11);
        const double pitchRateRadps =
            cosineRoll * state_inout(10) - sineRoll * state_inout(11);
        const double yawRateRadps =
            sineRoll * secantPitch * state_inout(10) +
            cosineRoll * secantPitch * state_inout(11);
        state_inout(3) = wrapAngle(state_inout(3) + rollRateRadps * ageS_in);
        state_inout(4) =
            wrapAngle(state_inout(4) + pitchRateRadps * ageS_in);
        state_inout(5) = wrapAngle(state_inout(5) + yawRateRadps * ageS_in);
    }

    static Filter::StateVector measurementVariances(
        const nav_msgs::msg::Odometry &message_in, double minimumVariance_in)
    {
        Filter::StateVector variances =
            Filter::StateVector::Constant(minimumVariance_in);
        for (Eigen::Index index = 0; index < 6; ++index)
        {
            const std::size_t covarianceIndex =
                static_cast<std::size_t>(index * 6 + index);
            const double poseVariance =
                message_in.pose.covariance[covarianceIndex];
            const double twistVariance =
                message_in.twist.covariance[covarianceIndex];
            if (std::isfinite(poseVariance) && poseVariance > 0.0)
            {
                variances(index) =
                    std::max(minimumVariance_in, poseVariance);
            }
            if (std::isfinite(twistVariance) && twistVariance > 0.0)
            {
                variances(index + 6) =
                    std::max(minimumVariance_in, twistVariance);
            }
        }
        return variances;
    }

    void handleMeasurement(const nav_msgs::msg::Odometry &message_in,
                           MeasurementKind kind_in)
    {
        Filter::StateVector measurement = odometryToState(message_in);
        if (!measurement.allFinite())
        {
            return;
        }

        const double filterTimestampS = now().seconds();
        if (kind_in == MeasurementKind::visual)
        {
            const rclcpp::Time measurementStamp(message_in.header.stamp,
                                                RCL_ROS_TIME);
            const double measurementAgeS =
                filterTimestampS - measurementStamp.seconds();
            if (measurementAgeS > maximumVisualMeasurementAgeS)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Dropping visual odometry %.3f s older than filter time",
                    measurementAgeS);
                return;
            }
            if (measurementAgeS > 0.0)
            {
                compensateMeasurementAge(measurement, measurementAgeS);
            }
        }
        if (!hasInitialState)
        {
            Filter::StateVector initialState = Filter::StateVector::Zero();
            initialState(0) = initialPositionMapM.x();
            initialState(1) = initialPositionMapM.y();
            initialState(2) = initialPositionMapM.z();
            double initialRollRad = 0.0;
            double initialPitchRad = 0.0;
            double initialYawRad = 0.0;
            tf2::Matrix3x3(initialOrientation)
                .getRPY(initialRollRad, initialPitchRad, initialYawRad);
            initialState(3) = initialRollRad;
            initialState(4) = initialPitchRad;
            initialState(5) = initialYawRad;
            Filter::MeasurementMask initialMask{};
            for (std::size_t index = 0U; index < 6U; ++index)
            {
                initialMask[index] = true;
            }
            const FilterStatus initialStatus =
                filter.step(filterTimestampS, &initialState, initialMask,
                            Filter::StateVector::Constant(1.0e-9), latestState,
                            latestCovariance);
            if (initialStatus != FilterStatus::FILTER_STATUS_SUCCESS)
            {
                logStepFailure(initialStatus);
                return;
            }
            hasInitialState = true;
        }

        Filter::MeasurementMask mask{};
        double variance = inertialVariance;
        if (kind_in == MeasurementKind::wheel)
        {
            /* Wheel pose is the integral of the same rates below and is
             * therefore correlated, not an independent observation. Fuse
             * encoder-derived velocity once, plus a planar Z constraint.
             */
            mask[2] = true;
            mask[5] = true;
            mask[6] = true;
            mask[7] = true;
            mask[8] = true;
            mask[11] = true;
            variance = wheelVariance;
        }
        else if (kind_in == MeasurementKind::visual)
        {
            /* Stereo VO constrains planar rover motion. Its unconstrained Z,
             * roll, and pitch drift must not pull the chassis off the terrain
             * plane or override the IMU attitude estimate.
             */
            mask[0] = true;
            mask[1] = true;
            mask[5] = true;
            mask[6] = true;
            mask[7] = true;
            mask[11] = true;
            variance = visualVariance;
        }
        else
        {
            /* Integrated IMU position and velocity are deliberately excluded:
             * their bias-driven drift is not an absolute observation. Roll,
             * pitch, and angular rates remain useful inertial measurements.
             */
            mask[3] = true;
            mask[4] = true;
            mask[9] = true;
            mask[10] = true;
            mask[11] = true;
        }

        Filter::StateVector variances =
            measurementVariances(message_in, variance);
        if (kind_in == MeasurementKind::wheel)
        {
            variances(2) = 0.01;
            variances(8) = 0.01;
        }
        const FilterStatus status = filter.step(
            filterTimestampS, &measurement, mask, variances, latestState,
            latestCovariance);
        if (status == FilterStatus::FILTER_STATUS_SUCCESS)
        {
            hasEstimate = true;
        }
        else
        {
            logStepFailure(status);
        }
    }

    void logStepFailure(FilterStatus status_in)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Continuous EKF step failed with status %u",
                             static_cast<unsigned int>(status_in));
    }

    void publishEstimate(const rclcpp::Time &stamp_in)
    {
        if (!hasEstimate)
        {
            return;
        }
        tf2::Quaternion orientation;
        orientation.setRPY(latestState(3), latestState(4), latestState(5));
        orientation.normalize();
        const tf2::Vector3 velocityOdom(latestState(6), latestState(7),
                                        latestState(8));
        const tf2::Vector3 velocityBody =
            tf2::Matrix3x3(orientation).transpose() * velocityOdom;

        nav_msgs::msg::Odometry output;
        output.header.stamp = stamp_in;
        output.header.frame_id = odomFrame;
        output.child_frame_id = baseFrame;
        output.pose.pose.position.x = latestState(0);
        output.pose.pose.position.y = latestState(1);
        output.pose.pose.position.z = latestState(2);
        output.pose.pose.orientation = tf2::toMsg(orientation);
        output.twist.twist.linear.x = velocityBody.x();
        output.twist.twist.linear.y = velocityBody.y();
        output.twist.twist.linear.z = velocityBody.z();
        output.twist.twist.angular.x = latestState(9);
        output.twist.twist.angular.y = latestState(10);
        output.twist.twist.angular.z = latestState(11);
        for (Eigen::Index index = 0; index < 6; ++index)
        {
            const std::size_t covarianceIndex =
                static_cast<std::size_t>(index * 6 + index);
            output.pose.covariance[covarianceIndex] =
                latestCovariance(index, index);
            output.twist.covariance[covarianceIndex] =
                latestCovariance(index + 6, index + 6);
        }
        outputPublisher->publish(output);

        if (transformBroadcaster)
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header = output.header;
            transform.child_frame_id = baseFrame;
            transform.transform.translation.x = latestState(0);
            transform.transform.translation.y = latestState(1);
            transform.transform.translation.z = latestState(2);
            transform.transform.rotation = output.pose.pose.orientation;
            transformBroadcaster->sendTransform(transform);
        }
    }

    Filter filter;
    Filter::StateVector latestState{Filter::StateVector::Zero()};
    Filter::StateMatrix latestCovariance{Filter::StateMatrix::Identity()};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr outputPublisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        inertialSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visualSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheelSubscription;
    rclcpp::TimerBase::SharedPtr predictionTimer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
    tf2::Quaternion initialOrientation{0.0, 0.0, 0.0, 1.0};
    tf2::Vector3 initialPositionMapM{0.0, 0.0, 0.72};
    std::string odomFrame;
    std::string baseFrame;
    double inertialVariance{0.20};
    double visualVariance{0.03};
    double wheelVariance{0.08};
    double maximumVisualMeasurementAgeS{0.5};
    bool hasEstimate{false};
    bool hasInitialState{false};
    bool publishTf{false};
};

} /* namespace lunar_simulator::localisation */

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<lunar_simulator::localisation::KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
