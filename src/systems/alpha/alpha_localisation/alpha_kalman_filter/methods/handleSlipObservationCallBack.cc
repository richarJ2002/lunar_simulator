/*!
 * @File:         handleSlipObservationCallBack.cc
 *
 * @Brief:        Implements fusion of whichever wheels' raw slip
 *                observations arrived this cycle into their own states.
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
#include <cmath>
#include <vector>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::handleSlipObservationCallBack(
    const std_msgs::msg::Float64MultiArray &message_in)
{
    if (!hasInitialState)
    {
        /*!
         * A slip observation carries no pose/attitude information, so
         * unlike handleMeasurementCallBack() it cannot seed the filter's
         * time origin; silently drop it until an odometry measurement has
         * already done so.
         */
        return;
    }

    if (message_in.data.size() != static_cast<std::size_t>(WHEEL_COUNT))
    {
        /*!
         * A malformed message (wrong element count) must never reach the
         * EKF; wheel_odometry always publishes exactly WHEEL_COUNT
         * elements (see WheelOdometryNode::publishSlipObservation()).
         */
        return;
    }

    /*!
     * Build the list of wheels actually observed this cycle: a NaN entry
     * means that wheel failed one of wheel_odometry's own gates (near-zero
     * speed, sign disagreement, out-of-range ratio, or no visual reference
     * at all) and carries no information this cycle, so it is excluded
     * from the observed indices below rather than fused as though it were
     * a genuine zero-slip reading.
     */
    std::vector<Eigen::Index> observedIndices;
    std::vector<double> observedSlipRatios;

    observedIndices.reserve(static_cast<std::size_t>(WHEEL_COUNT));
    observedSlipRatios.reserve(static_cast<std::size_t>(WHEEL_COUNT));

    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        const double observedSlipRatio =
            message_in.data[static_cast<std::size_t>(wheel)];

        if (!std::isfinite(observedSlipRatio))
        {
            /* This wheel was not observable this cycle; skip it. */
            continue;
        }

        observedIndices.push_back(SLIP_STATE_START_INDEX + wheel);
        observedSlipRatios.push_back(observedSlipRatio);
    }

    if (observedIndices.empty())
    {
        /* No wheel was observable this cycle; nothing to fuse. */
        return;
    }

    /* Propagate the filter to this observation's arrival time before
     * correcting with it, exactly as handleMeasurementCallBack() does. */
    const FilterStatus predictStatus = predictTo(now().seconds());

    if (predictStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        logStepFailure(predictStatus);

        return;
    }

    /* Read the post-prediction state once so every observed wheel's
     * innovation below is computed against the same snapshot. */
    const StateVector currentState = filter.getState();

    /* This is a direct, one-hot observation of exactly the observed
     * wheels' own slip states -- an unobserved wheel is simply absent from
     * this update, not corrected toward zero. */
    Eigen::VectorXd innovation(
        static_cast<Eigen::Index>(observedIndices.size()));

    for (std::size_t row = 0; row < observedIndices.size(); ++row)
    {
        innovation(static_cast<Eigen::Index>(row)) =
            observedSlipRatios[row] - currentState(observedIndices[row]);
    }

    const Eigen::MatrixXd observation =
        buildObservationMatrix(observedIndices, STATE_SIZE);

    /* Every observed wheel shares the same configured measurement
     * variance; there is no reason to expect one wheel's slip observation
     * to be inherently noisier than another's. */
    const Eigen::MatrixXd measurementNoise =
        Eigen::MatrixXd::Identity(
            static_cast<Eigen::Index>(observedIndices.size()),
            static_cast<Eigen::Index>(observedIndices.size())) *
        slipMeasurementVariance;

    const FilterStatus updateStatus =
        filter.update(innovation, observation, measurementNoise);

    if (updateStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        logStepFailure(updateStatus);

        return;
    }

    /*!
     * Clamp every wheel's slip state back into the physically meaningful
     * range after the additive correction (not just the wheels observed
     * this cycle, since an earlier update could have left another wheel
     * outside range), then write the clamped state back into the engine,
     * which has no knowledge of this state's valid bounds -- the same
     * pattern handleMeasurementCallBack() and predictTo() use for wrapping
     * angles.
     */
    StateVector clamped = filter.getState();

    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        const Eigen::Index index = SLIP_STATE_START_INDEX + wheel;

        clamped(index) = std::clamp(clamped(index), 0.0, maximumSlipRatio);
    }

    const FilterStatus setStatus = filter.setState(clamped);

    if (setStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        logStepFailure(setStatus);

        return;
    }

    /* Refresh this node's cached copy of the engine's state and covariance
     * for publishEstimate() and publishWheelSlip() to read. */
    latestState = filter.getState();
    latestCovariance = filter.getCovariance();

    /* Record that at least one valid estimate now exists. */
    hasEstimate = true;

    /* Republish the freshly fused six-wheel slip estimate for
     * wheel_odometry. */
    publishWheelSlip();
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
