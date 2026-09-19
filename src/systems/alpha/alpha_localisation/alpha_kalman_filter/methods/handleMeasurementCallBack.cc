/*!
 * @File:         handleMeasurementCallBack.cc
 *
 * @Brief:        Implements EKF measurement fusion for one odometry source.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <algorithm>
#include <cstddef>

#include <tf2/LinearMath/Matrix3x3.h>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::handleMeasurementCallBack(
    const nav_msgs::msg::Odometry &message_in, MeasurementKind kind_in)
{
    if (!hasReceivedInitialPose)
    {
        /*!
         * initialPositionMapM/initialOrientation have not been seeded from
         * ground_truth's one-time initial pose yet (see
         * handleInitialPoseCallBack()), so there is nothing sensible to
         * rebase this measurement against or seed the filter's own initial
         * state from; drop it and retry on the next measurement.
         */
        return;
    }

    /* Rebase this message's relative pose/twist into the shared map frame
     * and the EKF's eighteen-state representation (verbatim, not rebased,
     * for wheel -- see odometryToState()'s own doc comment). */
    StateVector measurement = odometryToState(message_in, kind_in);

    if (!measurement.allFinite())
    {
        /*!
         * A non-finite conversion (e.g. from a malformed upstream message)
         * must never reach the EKF; silently drop it rather than corrupt
         * the filter state.
         */
        return;
    }

    /* Read this node's own clock once so every check below compares
     * against the same instant. */
    const double filterTimestampSNow = now().seconds();

    if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        /*!
         * Stereo visual odometry is the only source expensive enough to
         * lag this node's current time by a non-negligible amount, so
         * only it is checked for staleness and extrapolated forward.
         */
        const rclcpp::Time measurementStamp(message_in.header.stamp,
                                            RCL_ROS_TIME);

        /* How far behind this node's current time this measurement
         * actually is. */
        const double measurementAgeS =
            filterTimestampSNow - measurementStamp.seconds();

        if (measurementAgeS > maximumVisualMeasurementAgeS)
        {
            /* Too stale to trust even after extrapolation; warn (rate
             * limited) and drop the measurement entirely. */
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping visual odometry %.3f s older than filter time",
                measurementAgeS);

            return;
        }

        if (measurementAgeS > 0.0)
        {
            /* Extrapolate the measurement forward to this node's current
             * time before it is fused below. */
            compensateMeasurementAge(measurement, measurementAgeS);
        }
    }

    /*!
     * Each odometry source only observes a subset of the eighteen-state
     * vector; the indices below document exactly which states this
     * measurement kind is trusted to correct, and why the remaining
     * states are deliberately left to the other two sources or to pure
     * prediction.
     */
    std::vector<Eigen::Index> observedIndices;

    /* Default to the inertial variance; overridden below for the other
     * two measurement kinds. */
    double variance = inertialVariance;

    if (kind_in == MeasurementKind::MEASUREMENT_KIND_WHEEL)
    {
        /*!
         * Wheel pose is the integral of the same rates below and is
         * therefore correlated, not an independent observation. Fuse
         * encoder-derived velocity once, plus a planar Z constraint.
         */

        /* Observe z position (2), yaw (5), x/y/z linear velocity (6,7,8)
         * and yaw rate (11). */
        observedIndices = {2, 5, 6, 7, 8, 11};

        /* Wheel odometry uses its own configured variance floor. */
        variance = wheelVariance;
    }
    else if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        /*!
         * Stereo VO constrains planar rover motion. Its unconstrained Z
         * drift must not pull the chassis off the terrain plane, so Z stays
         * excluded. Roll and pitch ARE included, deliberately loosely
         * weighted (visualAttitudeVariance, well above the tight variance
         * used for position/yaw/velocity below): the IMU dominates
         * roll/pitch moment-to-moment, but the IMU's own attitude is an
         * open-loop integration that drifts slowly and indefinitely absent
         * external correction, while stereo VO's independent, per-frame PnP
         * attitude estimate does not share that bias -- so even a loose
         * per-update visual correction, applied at every stereo frame,
         * anchors long-run roll/pitch drift without overriding the IMU's
         * better short-term estimate.
         */

        /* Observe x position (0), y position (1), roll (3), pitch (4), yaw
         * (5), x/y linear velocity (6,7) and yaw rate (11). */
        observedIndices = {0, 1, 3, 4, 5, 6, 7, 11};

        /* Visual odometry uses its own configured variance floor for
         * position/yaw/velocity; roll/pitch use visualAttitudeVariance
         * instead (see the per-axis override below). */
        variance = visualVariance;
    }
    else
    {
        /*!
         * Integrated IMU position and velocity are deliberately excluded:
         * their bias-driven drift is not an absolute observation. Roll,
         * pitch, and angular rates remain useful inertial measurements.
         */

        /* Observe roll (3), pitch (4) and all three body angular-rate
         * states (9,10,11). */
        observedIndices = {3, 4, 9, 10, 11};

        /*!
         * Cache the IMU's own gyro yaw rate for gyroOnlyYawRad (see its
         * doc comment) -- this is the raw inertial reading, never wheel's,
         * so it stays a slip-immune cross-check on wheel's own yaw.
         */
        latestGyroYawRateRadps = measurement(11);
    }

    if (!hasInitialState)
    {
        /*!
         * Seed the filter's initial position and attitude from the
         * configured initial pose (not from this measurement's own
         * relative pose, which is exactly zero at this point) so the very
         * first EKF step establishes a sensible time origin and a state
         * consistent with the configured initial_position_*_m /
         * initial_*_rad parameters.
         */
        StateVector initialState = StateVector::Zero();

        /* Seed the x/y/z position from the configured initial pose. */
        initialState(0) = initialPositionMapM.x();
        initialState(1) = initialPositionMapM.y();
        initialState(2) = initialPositionMapM.z();

        /* Destination for the roll/pitch/yaw decomposition below. */
        double initialRollRad = 0.0;
        double initialPitchRad = 0.0;
        double initialYawRad = 0.0;

        /* Decompose the configured initial orientation into roll, pitch
         * and yaw, the EKF's own attitude representation. */
        tf2::Matrix3x3(initialOrientation)
            .getRPY(initialRollRad, initialPitchRad, initialYawRad);

        /* Seed roll, pitch and yaw from the decomposition above. */
        initialState(3) = initialRollRad;
        initialState(4) = initialPitchRad;
        initialState(5) = initialYawRad;

        /* Seed the gyro-only yaw cross-check from the same configured
         * initial yaw, so it starts in agreement with the fused state. */
        gyroOnlyYawRad = initialYawRad;

        /* Seed the engine with this state and an Identity starting
         * covariance. */
        const FilterStatus initStatus =
            filter.initialize(STATE_SIZE, initialState,
                              StateMatrix::Identity());

        if (initStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            /* Report the failure and abandon this callback; the next
             * measurement will retry seeding. */
            logStepFailure(initStatus);

            return;
        }

        /* Establish the time origin used by predictTo(). */
        filterTimestampS = filterTimestampSNow;

        /*!
         * Apply a near-exact (tiny-variance) direct correction of the
         * just-seeded position/attitude states. The innovation is exactly
         * zero (state already equals the seed), so this only sharply
         * shrinks covariance for those six states -- establishing high
         * initial confidence in the configured starting pose, matching
         * how this seeding behaved before the engine stopped doing it
         * implicitly.
         */
        const std::vector<Eigen::Index> seedIndices{0, 1, 2, 3, 4, 5};
        const Eigen::MatrixXd seedObservation =
            buildObservationMatrix(seedIndices, STATE_SIZE);
        const Eigen::VectorXd seedInnovation = Eigen::VectorXd::Zero(
            static_cast<Eigen::Index>(seedIndices.size()));
        const Eigen::MatrixXd seedNoise =
            Eigen::MatrixXd::Identity(
                static_cast<Eigen::Index>(seedIndices.size()),
                static_cast<Eigen::Index>(seedIndices.size())) *
            1.0e-9;

        const FilterStatus seedStatus =
            filter.update(seedInnovation, seedObservation, seedNoise);

        if (seedStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            /* The seed correction failing does not invalidate the state
             * just seeded above; report it and continue to this
             * measurement's own correction below. */
            logStepFailure(seedStatus);
        }

        /* Record that the time origin now exists. */
        hasInitialState = true;
    }

    /* Propagate the filter to this measurement's arrival time before
     * correcting with it. */
    const FilterStatus predictStatus = predictTo(filterTimestampSNow);

    if (predictStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        /* Report the rejected step without crashing the callback. */
        logStepFailure(predictStatus);

        return;
    }

    /* Read the post-prediction state once; every observed axis's
     * innovation below is computed against this same snapshot. */
    const StateVector currentState = filter.getState();

    /* Derive per-state variances from the message's own reported
     * covariance, falling back to this measurement kind's floor. */
    const StateVector fullVariances =
        measurementVariances(message_in, variance);

    /* Innovation (measurement minus predicted state) and measurement
     * noise for only the observed axes; built one entry at a time
     * below. */
    Eigen::VectorXd innovation(
        static_cast<Eigen::Index>(observedIndices.size()));
    Eigen::MatrixXd measurementNoise = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(observedIndices.size()),
        static_cast<Eigen::Index>(observedIndices.size()));

    for (std::size_t row = 0; row < observedIndices.size(); ++row)
    {
        const Eigen::Index index = observedIndices[row];

        /* Innovation is simply measured minus predicted for a directly
         * observed state. */
        double delta = measurement(index) - currentState(index);

        /* Roll, pitch and yaw (state indices 3-5) are angles and must be
         * wrapped so a measurement near +/-pi does not produce a
         * spuriously large innovation across the wrap boundary. */
        if (index >= 3 && index <= 5)
        {
            delta = wrapAngle(delta);
        }

        innovation(static_cast<Eigen::Index>(row)) = delta;

        double axisVariance = fullVariances(index);

        if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL &&
                 (index == 3 || index == 4))
        {
            /*!
             * Deliberately loose relative to visualVariance (used for this
             * same message's position/yaw/velocity axes) and to inertial's
             * own roll/pitch variance: this correction exists to anchor
             * long-run IMU drift, not to compete with the IMU moment-to-
             * moment. Ignores this message's own reported covariance
             * (fullVariances above), matching the wheel Z override's
             * pattern, so a low-inlier-count frame's tighter self-reported
             * covariance can never accidentally let visual dominate
             * attitude.
             */
            axisVariance = visualAttitudeVariance;
        }
        else if (kind_in == MeasurementKind::MEASUREMENT_KIND_WHEEL &&
                 index == 5)
        {
            /*!
             * Wheel's own reported yaw variance (fullVariances above)
             * assumes it can observe yaw as reliably as position/velocity,
             * but a single shared linear slip ratio cannot represent
             * rotational slip (see WheelOdometryNode.h's class doc
             * comment): under sustained turning, wheel's own yaw can run
             * well ahead of the truth with nothing internal to flag it
             * (confirmed live: ~1.68x too fast at 0.15 rad/s commanded).
             * Inflate on top of the base variance with two independent
             * signals that this failure mode is active: how fast wheel
             * itself reports turning (faster turning risks more slip),
             * and how much wheel's yaw disagrees with gyroOnlyYawRad, an
             * independent, slip-immune cross-check integrated purely from
             * the IMU's own gyro reading. Both terms are ~0 at rest,
             * matching every stationary-divergence fix verified earlier.
             */
            const double wheelYawRateRadps = measurement(11);
            const double gyroDisagreementRad =
                wrapAngle(measurement(index) - gyroOnlyYawRad);

            const double yawVarianceInflation =
                wheelYawRateVarianceGain * wheelYawRateRadps *
                    wheelYawRateRadps +
                wheelYawGyroDisagreementGain * gyroDisagreementRad *
                    gyroDisagreementRad;

            /*!
             * Capped: under this exact failure mode, wheel's own yaw is
             * often close to uncorrelated noise (not just "off by a
             * factor"), so gyroDisagreementRad routinely approaches its
             * maximum (pi) -- confirmed live to reach the point of
             * catastrophic, unbounded position runaway (a resting-yaw
             * disagreement inflating this one axis by 100x+ relative to
             * this same update's other five observed wheel axes,
             * O(0.01-0.08), made the 6x6 measurement-noise matrix
             * ill-conditioned enough that Eigen::LDLT still reported
             * success while producing an inaccurate Kalman gain that
             * corrupted the cross-covariant, tightly-coupled velocity
             * states instead of just de-weighting yaw). The cap keeps
             * wheel's yaw de-weighted far below any other observed axis
             * without letting this one entry's scale run away within the
             * same small system.
             */
            axisVariance += std::min(yawVarianceInflation,
                                     maximumWheelYawVarianceInflation);
        }

        measurementNoise(static_cast<Eigen::Index>(row),
                         static_cast<Eigen::Index>(row)) = axisVariance;
    }

    const Eigen::MatrixXd observation =
        buildObservationMatrix(observedIndices, STATE_SIZE);

    /* Apply the observation-matrix-weighted, variance-weighted
     * measurement to the filter. */
    const FilterStatus updateStatus =
        filter.update(innovation, observation, measurementNoise);

    if (updateStatus == FilterStatus::FILTER_STATUS_SUCCESS)
    {
        /* Wrap roll, pitch and yaw back into their canonical range after
         * the additive correction, then write the wrapped state back
         * into the engine, which has no knowledge of which states are
         * angles. */
        StateVector wrapped = filter.getState();
        wrapped(3) = wrapAngle(wrapped(3));
        wrapped(4) = wrapAngle(wrapped(4));
        wrapped(5) = wrapAngle(wrapped(5));

        const FilterStatus setStatus = filter.setState(wrapped);

        if (setStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            logStepFailure(setStatus);

            return;
        }

        if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
        {
            /*!
             * Visual is the trustworthy absolute yaw reference when
             * available, so re-sync the gyro-only cross-check to it here
             * -- otherwise pure gyro integration would slowly drift over
             * long or visual-rich periods, defeating its purpose as a
             * cross-check for the specific periods visual is stale (see
             * gyroOnlyYawRad's doc comment).
             */
            gyroOnlyYawRad = wrapped(5);
        }

        /* Refresh this node's cached copy of the engine's state and
         * covariance for publishEstimate() to read. */
        latestState = filter.getState();
        latestCovariance = filter.getCovariance();

        /* Record that at least one valid estimate now exists. */
        hasEstimate = true;
    }
    else
    {
        /* Report the rejected step without crashing the callback. */
        logStepFailure(updateStatus);
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
