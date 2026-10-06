/*!
 * @File:         handleJointStateCallBack.cc
 *
 * @Brief:        Implements the six-wheel kinematic solve and pose
 *                integration.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
#include "console/console.h"

/* Object Include */
#include "objects/WheelOdometryNodeClass.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>
#include <cmath>

/* Module Includes */
#include "public_functions/public_functions.h"

namespace localisation::wheel_odometry
{

void WheelOdometryNode::handleJointStateCallBack(
    const sensor_msgs::msg::JointState &message_in)
{
    /* Raw (un-slip-adjusted) per-wheel speed, needed again later for
     * slip estimation. */
    std::array<double, 6> rawWheelSpeed_mPs{};

    /* Per-wheel steering angle, needed again later for slip
     * estimation. */
    std::array<double, 6> steerAngle_rad{};

    /* Read every wheel's steering angle and drive rate. */
    for (std::size_t wheel = 0U; wheel < 6U; wheel++)
    {
        /* This wheel's drive-joint position is not needed; only its
         * rate is. */
        double unusedDrivePosition = 0.0;

        /* This wheel's drive-joint rate, looked up below. */
        double driveRate_radPs = 0.0;

        /* This wheel's steering angle, looked up below. */
        double wheelSteerAngle_rad = 0.0;

        /* This wheel's steer-joint rate is not needed. */
        double unusedSteerRate = 0.0;

        /* Locate both this wheel's drive and steering joints by name. */
        if (!findJoint(message_in,
                       driveJointNames[wheel],
                       unusedDrivePosition,
                       driveRate_radPs) ||
            !findJoint(message_in,
                       steerJointNames[wheel],
                       wheelSteerAngle_rad,
                       unusedSteerRate))
        {
            /*!
             * All twelve joints (six drive, six steer) must be present
             * before any odometry can be computed; this is expected
             * transiently right after simulator startup while Gazebo is
             * still publishing the model's first joint states.
             */
            SRS_LOG_WARN_THROTTLE(get_logger(),
                                    *get_clock(),
                                    3000,
                                    "Waiting for all 12 wheel joints");
            hasAllJoints = false;

            /* Skip this callback entirely; try again next message. */
            return;
        }

        /* Retain this wheel's steering angle for the solve and the
         * slip-estimation call later in this method. */
        steerAngle_rad[wheel] = wheelSteerAngle_rad;

        /* Convert this wheel's drive rate to a raw circumferential
         * speed, applying its fixed sign convention. */
        rawWheelSpeed_mPs[wheel] =
            driveDirectionMultipliers[wheel] * wheelRadius_m * driveRate_radPs;
    }

    /* Convert this message's timestamp to seconds once, for every check
     * below that needs it. */
    const double stamp_s = stampToSeconds(message_in.header.stamp);

    /*!
     * Publish a fresh raw slip observation (when visual odometry supports
     * one) for continuous_ekf to fuse; slipRatios[] itself is only ever
     * updated by handleWheelSlipEstimateCallBack() reading back the fused
     * result, not by this call.
     */
    publishSlipObservation(stamp_s, rawWheelSpeed_mPs, steerAngle_rad);

    /* Force one initial publication so a subscriber never waits for the
     * first visual-odometry update to see a slip-ratio message. */
    if (!hasPublishedSlipRatios)
    {
        publishSlipRatios();
    }

    /* Apply the current slip ratio to every wheel's raw speed before
     * solving the kinematic system. */
    std::array<double, 6> wheelSpeed_mPs{};
    for (std::size_t wheel = 0U; wheel < slipRatios.size(); wheel++)
    {
        /* Clamp defensively in case a slip ratio ever drifted outside
         * its configured bound. */
        const double boundedSlip = shouldApplySlipFeedback
                                       ? std::clamp(slipRatios[wheel],
                                                    -maximumSlipRatio,
                                                    maximumSlipRatio)
                                       : 0.0;

        /*!
         * A positive ratio reduces rolling speed for wheel spin. A negative
         * ratio increases it for forward skid, where body travel exceeds the
         * distance implied by wheel rotation.
         */
        wheelSpeed_mPs[wheel] = rawWheelSpeed_mPs[wheel] * (1.0 - boundedSlip);
    }

    /* Twelve-row weighted least squares: six rolling rows and six
     * no-side-slip rows (see solveBodyTwist()). The no-side-slip rows make
     * lateral velocity observable in every steering layout, so no
     * observability threshold or reduced fallback solve is needed. */
    const BodyTwistSolution solution =
        solveBodyTwist(steerAngle_rad,
                       wheelSpeed_mPs,
                       wheelX_m,
                       wheelY_m,
                       wheelRadius_m * wheelAngularVelocityStddev_radPs,
                       steeringPositionStddev_rad,
                       lateralSlipStddev_mPs);
    solveCount++;

    /* A failed solve must not corrupt the integrated pose; it is counted
     * for the diagnostics report and the next message tries again. */
    if (!solution.isValid)
    {
        rejectedSolveCount++;
        return;
    }
    const Eigen::Vector3d &bodyTwist           = solution.twist_body;
    const Eigen::Matrix3d &bodyTwistCovariance = solution.covariance_body;
    latestNormalCondition                      = solution.normalCondition;
    latestLateralVelocity_mPs                   = bodyTwist.y();
    latestLateralVelocityStddev_mPs = std::sqrt(bodyTwistCovariance(1, 1));

    /* Integrate the new twist into the pose only once a previous
     * timestamp exists to measure an interval against. */
    if (hasPreviousStamp)
    {
        /* Elapsed time since the last integrated joint-state message. */
        const double dt_s = stamp_s - previousStamp_s;

        /* Integrate normally when the gap is positive and within the
         * configured bound. Pose starts at identity in startup-fixed. */
        if (dt_s > 0.0 && dt_s <= maximumIntegrationTimeGap_s)
        {
            /*!
             * Integrate only the independently observed planar wheel motion.
             * Wheel odometry does not borrow fused roll/pitch because that
             * would preprocess a measurement with the state it corrects.
             */
            const double cosineYaw = std::cos(yaw_rad);
            const double sineYaw   = std::sin(yaw_rad);
            const double fixedVelocityX_mPs =
                cosineYaw * bodyTwist.x() - sineYaw * bodyTwist.y();
            const double fixedVelocityY_mPs =
                sineYaw * bodyTwist.x() + cosineYaw * bodyTwist.y();

            /* Accumulate the rotated x velocity into position. */
            positionX_m += fixedVelocityX_mPs * dt_s;

            /* Accumulate the rotated y velocity into position. */
            positionY_m += fixedVelocityY_mPs * dt_s;

            /* Integrate yaw and keep it wrapped to [-pi, pi]. */
            yaw_rad = wrapAngle(yaw_rad + bodyTwist.z() * dt_s);
        }
        else if (dt_s > maximumIntegrationTimeGap_s)
        {
            /*!
             * A gap this large (e.g. simulator pause or dropped
             * messages) would integrate an implausibly large
             * displacement from a single instantaneous twist sample, so
             * the gap is skipped entirely rather than integrated.
             */
            SRS_LOG_WARN_THROTTLE(get_logger(),
                                    *get_clock(),
                                    2000,
                                    "Wheel gap %.2f s > %.2f s; skipped",
                                    dt_s,
                                    maximumIntegrationTimeGap_s);
        }
    }

    /* Record this message's time as the reference for the next
     * integration interval. */
    previousStamp_s = stamp_s;

    /* Mark that an interval reference now exists for future calls. */
    hasPreviousStamp = true;

    /* Publish the pose and body twist relative to startup-fixed. */
    publishOdometry(message_in.header.stamp, bodyTwist, bodyTwistCovariance);

    /* Track uninterrupted publishing for the readiness report: a gap
     * longer than readinessMaximumGap_s starts a new run. */
    hasAllJoints       = true;
    const double now_s = now().seconds();
    if (latestPublishTime_s < 0.0 ||
        now_s - latestPublishTime_s > readinessMaximumGap_s)
    {
        continuousPublishStart_s = now_s;
    }
    latestPublishTime_s = now_s;
}

} /* namespace localisation::wheel_odometry */
