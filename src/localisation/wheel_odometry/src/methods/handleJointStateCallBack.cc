/*!
 * @File:         handleJointStateCallBack.cc
 *
 * @Brief:        Implements the six-wheel rolling-constraint solve and pose
 *                integration.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace localisation::wheel_odometry
{

void WheelOdometryNode::handleJointStateCallBack(
    const sensor_msgs::msg::JointState &message_in)
{
    /*!
     * Build the 6x3 rolling-constraint matrix and the 6x1 slip-adjusted
     * wheel-speed vector so that solving `rollingMatrix * bodyTwist =
     * wheelSpeed` in least squares yields the (vx, vy, wz) body twist
     * that best explains all six observed wheel speeds simultaneously.
     */
    Eigen::Matrix<double, 6, 3> rollingMatrix;

    /* Slip-adjusted per-wheel speed vector, filled in below. */
    Eigen::Matrix<double, 6, 1> wheelSpeedMps;

    /* Raw (un-slip-adjusted) per-wheel speed, needed again later for
     * slip estimation. */
    std::array<double, 6> rawWheelSpeedMps{};

    /* Per-wheel steering angle, needed again later for slip
     * estimation. */
    std::array<double, 6> steerAngleRad{};

    /* Populate one row of the rolling-constraint system per wheel. */
    for (std::size_t wheel = 0U; wheel < 6U; ++wheel)
    {
        /* This wheel's drive-joint position is not needed; only its
         * rate is. */
        double unusedDrivePosition = 0.0;

        /* This wheel's drive-joint rate, looked up below. */
        double driveRateRadps = 0.0;

        /* This wheel's steering angle, looked up below. */
        double wheelSteerAngleRad = 0.0;

        /* This wheel's steer-joint rate is not needed. */
        double unusedSteerRate = 0.0;

        /* Locate both this wheel's drive and steering joints by name. */
        if (!findJoint(message_in, driveJointNames[wheel],
                       unusedDrivePosition, driveRateRadps) ||
            !findJoint(message_in, steerJointNames[wheel], wheelSteerAngleRad,
                       unusedSteerRate))
        {
            /*!
             * All twelve joints (six drive, six steer) must be present
             * before any odometry can be computed; this is expected
             * transiently right after simulator startup while Gazebo is
             * still publishing the model's first joint states.
             */
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Waiting for all six drive and steering joints");

            /* Skip this callback entirely; try again next message. */
            return;
        }

        /* Retain this wheel's steering angle for the slip-estimation
         * call later in this method. */
        steerAngleRad[wheel] = wheelSteerAngleRad;

        /* Precompute this wheel's steering trigonometry once. */
        const double cosine = std::cos(wheelSteerAngleRad);

        /* Precompute this wheel's steering trigonometry once. */
        const double sine = std::sin(wheelSteerAngleRad);

        /*!
         * Row i of the rolling matrix encodes wheel i's
         * rolling-constraint coefficients on (vx, vy, wz):
         *   cos(delta_i), sin(delta_i),
         *   -y_i*cos(delta_i) + x_i*sin(delta_i)
         * as derived in the class-level documentation.
         */
        rollingMatrix(static_cast<Eigen::Index>(wheel), 0) = cosine;

        /* Fill in this row's vy coefficient. */
        rollingMatrix(static_cast<Eigen::Index>(wheel), 1) = sine;

        /* Fill in this row's wz coefficient. */
        rollingMatrix(static_cast<Eigen::Index>(wheel), 2) =
            -wheelYM[wheel] * cosine + wheelXM[wheel] * sine;

        /* Convert this wheel's drive rate to a raw circumferential
         * speed, applying its fixed sign convention. */
        rawWheelSpeedMps[wheel] = driveDirectionMultipliers[wheel] *
                                  wheelRadiusM * driveRateRadps;
    }

    /* Convert this message's timestamp to seconds once, for every check
     * below that needs it. */
    const double stampS = stampToSeconds(message_in.header.stamp);

    /*!
     * Publish a fresh raw slip observation (when visual odometry supports
     * one) for continuous_ekf to fuse; slipRatios[] itself is only ever
     * updated by handleWheelSlipEstimateCallBack() reading back the fused
     * result, not by this call.
     */
    publishSlipObservation(stampS, rawWheelSpeedMps, steerAngleRad);

    /* Force one initial publication so a subscriber never waits for the
     * first visual-odometry update to see a slip-ratio message. */
    if (!hasPublishedSlipRatios)
    {
        publishSlipRatios();
    }

    /* Apply the current slip ratio to every wheel's raw speed before
     * solving the rolling-constraint system. */
    for (std::size_t wheel = 0U; wheel < slipRatios.size(); ++wheel)
    {
        /* Clamp defensively in case a slip ratio ever drifted outside
         * its configured bound. */
        const double boundedSlip =
            std::clamp(slipRatios[wheel], 0.0, maximumSlipRatio);

        /*!
         * A wheel spinning with slip ratio s only advances the rover by
         * a fraction (1 - s) of its raw circumferential speed; the
         * remainder is lost to the wheel spinning in place relative to
         * the ground.
         */
        wheelSpeedMps(static_cast<Eigen::Index>(wheel)) =
            rawWheelSpeedMps[wheel] * (1.0 - boundedSlip);
    }

    /*!
     * For each wheel i:
     * cos(delta_i) vx + sin(delta_i) vy
     * + (-y_i cos(delta_i) + x_i sin(delta_i)) wz = r omega_i.
     * The overdetermined six-wheel system is solved in minimum-norm
     * least squares. This sets an unobservable lateral component to
     * zero when all wheels are parallel instead of allowing arbitrary
     * sideways drift.
     */
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 3>>
        rollingDecomposition(rollingMatrix);

    /*!
     * All six steering joints report position with independent Gaussian
     * encoder noise (wheel_position_stddev_rad in alpha_drivers, typically
     * a few milliradians), so "all wheels parallel" almost never presents
     * as an exactly-zero singular value in practice -- every steering
     * angle carries a small nonzero reading even when the true angle is
     * exactly 0. A numerical-zero threshold (e.g. 1e-6) is far below that
     * noise floor, so it fails to catch this near-singular case: the
     * decomposition then treats the barely-nonzero vy direction as
     * "observed" and inverts a tiny singular value, amplifying ordinary
     * encoder noise into large spurious lateral-velocity solutions (a few
     * milliradians of steering noise measured a lateral velocity of over
     * 0.1 m/s on a stationary rover during diagnosis). Flooring the
     * threshold at lateralObservabilityThreshold (default 0.02, an order
     * of magnitude above that noise floor and well below any deliberately
     * commanded crab/turn angle) restores the class-level documentation's
     * intent: treat a near-parallel wheel configuration as genuinely
     * unobservable in vy, not merely exactly-parallel.
     */
    rollingDecomposition.setThreshold(lateralObservabilityThreshold);

    /* Solve for the body twist that best explains all six wheels. */
    const Eigen::Vector3d bodyTwist =
        rollingDecomposition.solve(wheelSpeedMps);

    /* Guard against a degenerate solve before it can corrupt the
     * integrated pose. */
    if (!bodyTwist.allFinite())
    {
        /*!
         * A degenerate or numerically unstable solve (e.g. all wheels
         * momentarily reporting identical, ill-conditioned angles) must
         * not corrupt the integrated pose; skip this cycle and try
         * again on the next joint-state message.
         */
        return;
    }

    /* Integrate the new twist into the pose only once a previous
     * timestamp exists to measure an interval against. */
    if (hasPreviousStamp)
    {
        /* Elapsed time since the last integrated joint-state message. */
        const double dtS = stampS - previousStampS;

        /* Integrate normally when the gap is positive and within the
         * configured bound, and once ground_truth's one-time initial pose
         * has seeded this node's own integrated pose and live roll/pitch
         * cache (see handleInitialPoseCallBack()) -- integrating from the
         * not-yet-seeded (zero) defaults would corrupt the pose this node
         * publishes below. */
        if (dtS > 0.0 && dtS <= maximumIntegrationDtS && hasReceivedInitialPose)
        {
            /*!
             * Euler-integrate the body twist into the map-aligned pose
             * using the full current attitude (roll/pitch borrowed from
             * continuous_ekf's latest fused estimate, see
             * handleKalmanFilterCallBack(); yaw this node's own, at the
             * *start* of this interval) rather than yaw alone, so a body-
             * frame (vx, vy) increment on a tilted rover is correctly
             * projected onto all three map axes instead of assuming a
             * level body frame -- this is what lets z become a genuine
             * estimate below instead of staying fixed at zero.
             */
            tf2::Matrix3x3 rotation;
            rotation.setRPY(latestRollRad, latestPitchRad, yawRad);

            /* Body-frame velocity has no z component: the rolling-
             * constraint solve above has no vertical degree of freedom. */
            const tf2::Vector3 bodyVelocityMps(bodyTwist.x(), bodyTwist.y(),
                                               0.0);

            /* Rotate the body-frame velocity into the map-aligned frame. */
            const tf2::Vector3 worldVelocityMps = rotation * bodyVelocityMps;

            /* Accumulate the rotated x velocity into position. */
            positionXM += worldVelocityMps.x() * dtS;

            /* Accumulate the rotated y velocity into position. */
            positionYM += worldVelocityMps.y() * dtS;

            /* Accumulate the rotated z velocity into position. */
            positionZM += worldVelocityMps.z() * dtS;

            /* Integrate yaw and keep it wrapped to [-pi, pi]. */
            yawRad = wrapAngle(yawRad + bodyTwist.z() * dtS);
        }
        else if (dtS > maximumIntegrationDtS)
        {
            /*!
             * A gap this large (e.g. simulator pause or dropped
             * messages) would integrate an implausibly large
             * displacement from a single instantaneous twist sample, so
             * the gap is skipped entirely rather than integrated.
             */
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping %.3f s wheel integration gap (limit %.3f s)",
                dtS, maximumIntegrationDtS);
        }
    }

    /* Record this message's time as the reference for the next
     * integration interval. */
    previousStampS = stampS;

    /* Mark that an interval reference now exists for future calls. */
    hasPreviousStamp = true;

    if (hasReceivedInitialPose)
    {
        /* Publish the newly integrated pose and current twist -- withheld
         * entirely until ground_truth's one-time initial pose has seeded
         * this node (see handleInitialPoseCallBack()), so continuous_ekf
         * can never fuse a not-yet-seeded (zero) wheel pose as though it
         * were real. */
        publishOdometry(message_in.header.stamp, bodyTwist);
    }
}

} /* namespace localisation::wheel_odometry */
