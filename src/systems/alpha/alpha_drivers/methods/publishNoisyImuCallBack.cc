/*!
 * @file            publishNoisyImuCallBack.cc
 *
 * @brief           Adds configured error to one IMU measurement.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNodeClass.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::publishNoisyImuCallBack(
    const sensor_msgs::msg::Imu &message_in)
{
    /* Raw IMU freshness is half of this node's readiness. */
    latestRawImuReceipt_s = now().seconds();

    /* Start from a copy of the raw, noise-free measurement. */
    sensor_msgs::msg::Imu message = message_in;

    /* Perturb every field only when noise injection is enabled. */
    if (noiseEnabled)
    {
        /*!
         * Constant per-axis bias is added first, then independent Gaussian
         * white noise per axis, matching how a real IMU's error model is
         * usually decomposed (slowly-varying bias plus fast-varying noise).
         */

        /*!
         * Add the constant bias and a fresh noise sample to angular rate
         * about the x axis.
         */
        message.angular_velocity.x +=
            imuAngularVelocityBias_radPs[X_AXIS] +
            sampleGaussian(imuAngularVelocityStddev_radPs);

        /*!
         * Add the constant bias and a fresh noise sample to angular rate
         * about the y axis.
         */
        message.angular_velocity.y +=
            imuAngularVelocityBias_radPs[Y_AXIS] +
            sampleGaussian(imuAngularVelocityStddev_radPs);

        /*!
         * Add the constant bias and a fresh noise sample to angular rate
         * about the z axis.
         */
        message.angular_velocity.z +=
            imuAngularVelocityBias_radPs[Z_AXIS] +
            sampleGaussian(imuAngularVelocityStddev_radPs);

        /*!
         * Add the constant bias and a fresh noise sample to x-axis linear
         * acceleration.
         */
        message.linear_acceleration.x +=
            imuLinearAccelerationBias_mPs2[X_AXIS] +
            sampleGaussian(imuLinearAccelerationStddev_mPs2);

        /*!
         * Add the constant bias and a fresh noise sample to y-axis linear
         * acceleration.
         */
        message.linear_acceleration.y +=
            imuLinearAccelerationBias_mPs2[Y_AXIS] +
            sampleGaussian(imuLinearAccelerationStddev_mPs2);

        /*!
         * Add the constant bias and a fresh noise sample to z-axis linear
         * acceleration.
         */
        message.linear_acceleration.z +=
            imuLinearAccelerationBias_mPs2[Z_AXIS] +
            sampleGaussian(imuLinearAccelerationStddev_mPs2);

        /* Apply a small random rotation to the reported orientation. */
        perturbOrientation(message.orientation);
    }

    /* Report the covariance actually injected only when noise is on. */
    if (noiseEnabled)
    {
        /*!
         * The published covariance must reflect the noise actually injected
         * above so that downstream consumers (e.g. the EKF) weight this
         * measurement correctly; each channel's variance is added onto
         * whatever covariance Gazebo already reported.
         */

        /* Add the orientation-noise variance onto the diagonal. */
        addVarianceToCovariance(imuOrientationStddev_rad *
                                    imuOrientationStddev_rad,
                                message.orientation_covariance);

        /* Add the angular-rate-noise variance onto the diagonal. */
        addVarianceToCovariance(imuAngularVelocityStddev_radPs *
                                    imuAngularVelocityStddev_radPs,
                                message.angular_velocity_covariance);

        /* Add the linear-acceleration-noise variance onto the diagonal. */
        addVarianceToCovariance(imuLinearAccelerationStddev_mPs2 *
                                    imuLinearAccelerationStddev_mPs2,
                                message.linear_acceleration_covariance);
    }

    /* Publish the (possibly perturbed) measurement to the public topic. */
    p_imuPublisher->publish(message);
}

} /* namespace systems::alpha::alpha_drivers */
