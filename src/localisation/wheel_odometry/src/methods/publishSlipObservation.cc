/*!
 * @File:         publishSlipObservation.cc
 *
 * @Brief:        Implements the visual-odometry-driven raw per-wheel slip
 *                observation, published for continuous_ekf to fuse.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>
#include <cmath>
#include <limits>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::publishSlipObservation(
    double jointStampS_in, const std::array<double, 6> &rawWheelSpeedMps_in,
    const std::array<double, 6> &steerAngleRad_in)
{
    /*!
     * Only attempt an observation when: slip estimation is enabled, a
     * visual reference has ever been received, that reference is still
     * within the configured timeout of the current joint-state message,
     * and it is newer than the reference used for the previous
     * observation (otherwise the same observation would be republished
     * repeatedly on stale data).
     */
    if (!shouldEstimateSlip || !hasVisualReference ||
        std::abs(jointStampS_in - latestVisualStampS) >
            visualOdometryTimeoutS ||
        latestVisualStampS <= lastSlipUpdateStampS)
    {
        /* One of the preconditions above failed; nothing to publish this
         * cycle. */
        return;
    }

    /* Clamp the configured ceiling to a valid slip-ratio range. */
    const double boundedMaximumSlip =
        std::clamp(maximumSlipRatio, 0.0, 0.99);

    /*!
     * One observation per wheel, independently: a single shared value
     * cannot represent rotational slip, where inner/outer wheels genuinely
     * slip by different amounts during a turn -- see
     * AlphaKalmanFilterNode's own class-level doc comment. A wheel that
     * fails any gate below is marked NaN rather than excluded from a
     * shared collection, so continuous_ekf can tell "not slipping" apart
     * from "not observed this cycle" (see handleSlipObservationCallBack()).
     */
    std::array<double, 6> observedSlipRatios{};
    bool anyWheelObserved = false;

    /* Evaluate every wheel independently against the same visual twist
     * reference. */
    for (std::size_t wheel = 0U; wheel < observedSlipRatios.size(); ++wheel)
    {
        observedSlipRatios[wheel] =
            std::numeric_limits<double>::quiet_NaN();

        /* Read this wheel's raw (un-slip-adjusted) circumferential
         * speed. */
        const double rawWheelSpeedMps = rawWheelSpeedMps_in[wheel];

        /*!
         * Near-zero wheel speeds make the slip ratio (which divides by
         * wheel speed) numerically unstable and physically meaningless,
         * so such wheels are excluded from this update entirely.
         */
        if (std::abs(rawWheelSpeedMps) < minimumWheelSpeedMps)
        {
            /* Leave this wheel's observation as NaN. */
            continue;
        }

        /* Precompute this wheel's steering trigonometry once. */
        const double cosine = std::cos(steerAngleRad_in[wheel]);

        /* Precompute this wheel's steering trigonometry once. */
        const double sine = std::sin(steerAngleRad_in[wheel]);

        /*!
         * Project the cached visual-odometry body twist onto this
         * wheel's rolling direction to get the rolling speed the wheel
         * *should* show if it were not slipping. This is the same
         * rolling-constraint relation used for pose integration, but
         * evaluated per-wheel here rather than solved jointly:
         *
         *   expectedRollingSpeed_i = cos(delta_i) * (vx - y_i * wz)
         *                            + sin(delta_i) * (vy + x_i * wz)
         *
         * where delta_i is wheel i's steering angle, (vx, vy, wz) is the
         * visual-odometry body twist, and (x_i, y_i) is wheel i's
         * body-frame position. All quantities are in the body frame, m/s
         * and rad/s.
         */
        const double expectedRollingSpeedMps =
            cosine * (latestVisualTwistBody.x() -
                      wheelYM[wheel] * latestVisualTwistBody.z()) +
            sine * (latestVisualTwistBody.y() +
                    wheelXM[wheel] * latestVisualTwistBody.z());

        /*!
         * A sign disagreement between the observed and expected rolling
         * speed indicates the wheel is not simply slipping forward/back
         * along its own rolling direction (e.g. transient turning
         * dynamics), so that wheel's observation is discarded this cycle
         * rather than folded into a misleading ratio.
         */
        if (rawWheelSpeedMps * expectedRollingSpeedMps < 0.0)
        {
            /* Leave this wheel's observation as NaN. */
            continue;
        }

        /* Compare expected against actual rolling speed to get this
         * wheel's raw (unbounded) slip observation. */
        const double observedSlipRatioUnbounded =
            1.0 - expectedRollingSpeedMps / rawWheelSpeedMps;

        /*!
         * Slip ratio is only physically meaningful in [0, maximumSlip);
         * a negative value would mean the wheel is rolling faster than
         * expected (traction, not slip) and is not modelled here, and an
         * overly large value usually indicates a bad visual-odometry
         * projection rather than genuine wheel slip.
         */
        if (observedSlipRatioUnbounded < 0.0 ||
            observedSlipRatioUnbounded > boundedMaximumSlip)
        {
            /* Leave this wheel's observation as NaN. */
            continue;
        }

        /* Snap a small residual observation down to exactly zero slip. */
        double observedSlipRatio = observedSlipRatioUnbounded;

        if (observedSlipRatio < slipRatioDeadband)
        {
            observedSlipRatio = 0.0;
        }

        observedSlipRatios[wheel] = observedSlipRatio;
        anyWheelObserved = true;
    }

    /* No wheel survived the gates above; there is no update to apply. */
    if (!anyWheelObserved)
    {
        return;
    }

    /*!
     * continuous_ekf owns the actual time-fusion of each wheel's own
     * observation (Kalman gain weighted by how much it currently trusts
     * that wheel's observation versus its own running estimate for that
     * wheel), replacing what used to be a fixed-gain exponential blend
     * here.
     */
    std_msgs::msg::Float64MultiArray observation;
    observation.data.assign(observedSlipRatios.begin(),
                            observedSlipRatios.end());
    slipObservationPublisher->publish(observation);

    /* Record this reference's time so a stale one is not reused. */
    lastSlipUpdateStampS = latestVisualStampS;
}

} /* namespace localisation::wheel_odometry */
