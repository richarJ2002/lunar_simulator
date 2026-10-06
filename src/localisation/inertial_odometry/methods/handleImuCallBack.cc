/*!
 * @File:         handleImuCallBack.cc
 *
 * @Brief:        Filters, integrates and publishes one IMU sample.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
#include "console/console.h"

/* Object Include */
#include "objects/InertialOdometryNodeClass.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>

namespace localisation::inertial_odometry
{

void InertialOdometryNode::handleImuCallBack(
    const sensor_msgs::msg::Imu &message)
{
    /* Convert this sample's ROS timestamp to seconds. */
    const double stamp_s = stampToSeconds(message.header.stamp);

    /*!
     * The very first sample only establishes a time origin: there is no
     * previous stamp yet, so no delta time (and therefore no integration)
     * can be computed.
     */
    if (!hasPreviousStamp)
    {
        /* Remember this sample's time as the integration origin. */
        previousStamp_s = stamp_s;

        /* Every later sample now has a previous stamp to diff against. */
        hasPreviousStamp = true;

        /* Nothing more can be done with the very first sample. */
        return;
    }

    /* Elapsed time since the previous processed sample. */
    const double dt_s = stamp_s - previousStamp_s;

    /* Advance the reference stamp for the next call. */
    previousStamp_s = stamp_s;

    /* Reject a non-positive or excessively large step. */
    if (!(dt_s > 0.0) || dt_s > maximumDt_s)
    {
        /*!
         * A non-positive or excessively large dt (clock jump, dropped
         * messages, simulation reset) would corrupt the integrators below,
         * so the sample is discarded rather than integrated.
         */
        SRS_LOG_WARN_THROTTLE(get_logger(),
                                *get_clock(),
                                2000,
                                "IMU dt %.3f s invalid; sample ignored",
                                dt_s);

        /* Skip integration entirely for this sample. */
        return;
    }

    /*!
     * First-order low-pass filter expressed as an exponential-smoothing
     * gain: cutoffHz <= 0 disables filtering (gain = 1, i.e. pass raw
     * samples straight through). timeConstantS is the classic RC time
     * constant 1 / (2*pi*f_c).
     */
    const double timeConstantS =
        cutoffHz > 0.0 ? 1.0 / (2.0 * M_PI * cutoffHz) : 0.0;

    /* gain = dt / (tau + dt) is the discrete-time equivalent of that
     * continuous first-order filter for this sample's dt. */
    const double gain =
        timeConstantS > 0.0 ? dt_s / (timeConstantS + dt_s) : 1.0;

    /* Copy the raw body-frame acceleration into a fixed-size array so it
     * can be indexed uniformly with the angular rate below. */
    const std::array<double, 3> rawAcceleration{message.linear_acceleration.x,
                                                message.linear_acceleration.y,
                                                message.linear_acceleration.z};

    /* Copy the raw body-frame angular rate the same way. */
    const std::array<double, 3> rawAngularRate{message.angular_velocity.x,
                                               message.angular_velocity.y,
                                               message.angular_velocity.z};

    /* Still within the startup stationary-calibration window. */
    if (calibrationSampleCount < calibrationSampleTarget)
    {
        /*!
         * Startup calibration window: the rover is assumed stationary, so
         * every raw sample here is pure bias plus noise. Accumulate sums so
         * the average can be taken once enough samples are collected,
         * instead of integrating position/velocity from noisy raw data.
         */
        for (std::size_t index = 0U; index < 3U; index++)
        {
            /* Accumulate this axis's raw acceleration sample. */
            accelerationCalibrationSum[index] += rawAcceleration[index];

            /* Accumulate this axis's raw angular-rate sample. */
            angularCalibrationSum[index] += rawAngularRate[index];
        }

        /* One more calibration sample has now been observed. */
        calibrationSampleCount++;

        /* The calibration window has just been completed. */
        if (calibrationSampleCount == calibrationSampleTarget)
        {
            std::array<double, 3> stationaryAccelerationMean{};
            for (std::size_t index = 0U; index < 3U; index++)
            {
                stationaryAccelerationMean[index] =
                    accelerationCalibrationSum[index] /
                    static_cast<double>(calibrationSampleTarget);
                angularRateBias[index] =
                    angularCalibrationSum[index] /
                    static_cast<double>(calibrationSampleTarget);
            }

            /* Static acceleration cannot separate all bias components from
             * gravity. Use measured direction plus known lunar magnitude as
             * the gravity prior; the residual is the accelerometer-bias prior. */
            gravitySpecificForceFixed_mPs2 =
                calculateGravitySpecificForceFixed(
                    tf2::Vector3(stationaryAccelerationMean[0],
                                 stationaryAccelerationMean[1],
                                 stationaryAccelerationMean[2]),
                    gravity_mPs2);
            if (gravitySpecificForceFixed_mPs2.length2() <= 1.0e-24)
            {
                SRS_LOG_ERROR(get_logger(),
                                "IMU calibration found no gravity");
                calibrationSampleCount = 0;
                accelerationCalibrationSum = {0.0, 0.0, 0.0};
                angularCalibrationSum = {0.0, 0.0, 0.0};
                return;
            }

            for (std::size_t index = 0U; index < 3U; index++)
            {
                accelerationBias[index] =
                    stationaryAccelerationMean[index] -
                    gravitySpecificForceFixed_mPs2[
                        static_cast<int>(index)];
                filteredAcceleration[index] =
                    gravitySpecificForceFixed_mPs2[static_cast<int>(index)];
                filteredAngularRate[index] = 0.0;
            }

            /* Record calibration completion once, for the operator log and
             * the recorded calibration status. */
            calibrationCompleteStamp_s = stamp_s;
            SRS_LOG_INFO(get_logger(),
                           "IMU calibration complete (%d samples)",
                           calibrationSampleTarget);
        }

        /* No integration happens until calibration has completed. */
        return;
    }

    for (std::size_t index = 0U; index < 3U; index++)
    {
        /*!
         * Exponential smoothing: filtered += gain * (bias-corrected raw -
         * filtered). Equivalent to filtered = (1-gain)*filtered +
         * gain*raw_corrected.
         */
        filteredAcceleration[index] +=
            gain * (rawAcceleration[index] - accelerationBias[index] -
                    filteredAcceleration[index]);

        /* Apply the same exponential smoothing to angular rate. */
        filteredAngularRate[index] +=
            gain * (rawAngularRate[index] - angularRateBias[index] -
                    filteredAngularRate[index]);

        /* This axis's filtered angular rate is within the deadband. */
        if (std::abs(filteredAngularRate[index]) < angularRateDeadband_radPs)
        {
            /*!
             * Suppress residual noise near zero rotation rate so a
             * perfectly still rover does not slowly drift in orientation.
             */
            filteredAngularRate[index] = 0.0;
        }
    }

    /*!
     * q(k+1) = q(k) * Exp(0.5 * omega_body * dt): integrate the body-frame
     * angular rate as a rotation-vector increment (axis = normalized
     * angular rate, angle = |omega| * dt) and compose it onto the current
     * orientation. This is the standard small-step quaternion integrator
     * for a constant angular velocity over one sample interval.
     */
    const double angularMagnitude =
        std::sqrt(filteredAngularRate[0] * filteredAngularRate[0] +
                  filteredAngularRate[1] * filteredAngularRate[1] +
                  filteredAngularRate[2] * filteredAngularRate[2]);

    /* The rotation increment to compose onto the current orientation. */
    tf2::Quaternion increment;

    /* The filtered angular rate is large enough to normalize safely. */
    if (angularMagnitude > 1.0e-12)
    {
        /* Build the increment from the normalized rotation axis and the
         * rotation angle swept over this sample's dt. */
        increment.setRotation(
            tf2::Vector3(filteredAngularRate[0] / angularMagnitude,
                         filteredAngularRate[1] / angularMagnitude,
                         filteredAngularRate[2] / angularMagnitude),
            angularMagnitude * dt_s);
    }
    else
    {
        /*!
         * Degenerate zero-rotation case: avoid dividing by a
         * near-zero magnitude and apply the identity rotation instead.
         */
        increment.setValue(0.0, 0.0, 0.0, 1.0);
    }

    /* Compose the increment onto the running orientation estimate. */
    orientation = orientation * increment;

    /* Re-normalize after composition to counter floating-point drift. */
    orientation.normalize();

    /* Express the filtered body-frame acceleration as a vector so it can
     * be rotated below. */
    const tf2::Vector3 accelerationBody(filteredAcceleration[0],
                                        filteredAcceleration[1],
                                        filteredAcceleration[2]);

    /*!
     * Rotate the filtered body-frame acceleration into startup-fixed using
     * the just-updated relative orientation, then remove the constant gravity
     * specific-force vector measured during stationary calibration. The fixed
     * frame may be tilted, so no axis is assumed vertical.
     */
    tf2::Vector3 accelerationOdom =
        tf2::Matrix3x3(orientation) * accelerationBody;

    /* Gravity removal is enabled. */
    if (removeGravity)
    {
        accelerationOdom = calculateGravityFreeAccelerationFixed(
            orientation,
            accelerationBody,
            gravitySpecificForceFixed_mPs2);
    }

    /*!
     * Snap x to exactly zero if it lies within the acceleration deadband,
     * suppressing residual noise while stationary.
     */
    if (std::abs(accelerationOdom.x()) < accelerationDeadband_mPs2)
    {
        accelerationOdom.setX(0.0);
    }

    /* Apply the same deadband to the y component. */
    if (std::abs(accelerationOdom.y()) < accelerationDeadband_mPs2)
    {
        accelerationOdom.setY(0.0);
    }

    /* Apply the same deadband to the z component. */
    if (std::abs(accelerationOdom.z()) < accelerationDeadband_mPs2)
    {
        accelerationOdom.setZ(0.0);
    }

    /*!
     * Keep the pre-update velocity so trapezoidal integration below can average
     * it with the just-updated velocity.
     */
    const std::array<double, 3> previousVelocity = velocity_mPs;

    /* Euler-integrate the x component of acceleration into velocity. */
    velocity_mPs[0] += accelerationOdom.x() * dt_s;

    /* Euler-integrate the y component of acceleration into velocity. */
    velocity_mPs[1] += accelerationOdom.y() * dt_s;

    /* Euler-integrate the z component of acceleration into velocity. */
    velocity_mPs[2] += accelerationOdom.z() * dt_s;

    for (std::size_t index = 0U; index < 3U; index++)
    {
        /*!
         * Trapezoidal integration of velocity into position, using the
         * previous and current velocity average rather than a plain Euler
         * step for slightly better accuracy given the sample rate is not
         * guaranteed uniform.
         */
        positionM[index] +=
            0.5 * (previousVelocity[index] + velocity_mPs[index]) * dt_s;
    }

    /*!
     * Publish the filtered IMU sample alongside the gravity-adjusted
     * acceleration just computed.
     */
    publishFilteredImu(message, accelerationOdom);

    /* Publish the newly integrated odometry estimate. */
    publishOdometry(message.header.stamp);
}

} /* namespace localisation::inertial_odometry */
