/*!
 * @File:         predictTo.cc
 *
 * @Brief:        Implements substep-bounded propagation of the owned filter
 *                to a requested monotonic timestamp.
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

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus
AlphaKalmanFilterNode::predictTo(double targetTimestampS_in)
{
    /* How far forward the state needs to be propagated. */
    double remainingTimeS = targetTimestampS_in - filterTimestampS;

    /* Small negative gaps are floating-point noise around zero, not a
     * genuine request to predict backwards. */
    constexpr double timestampToleranceS = 1.0e-9;

    if (remainingTimeS < -timestampToleranceS)
    {
        /* A genuinely earlier target time is a caller error. */
        return FilterStatus::FILTER_STATUS_INVALID_INPUT;
    }

    if (remainingTimeS <= 0.0)
    {
        /* Already at (or within tolerance of) the target time; nothing
         * to propagate. */
        return FilterStatus::FILTER_STATUS_SUCCESS;
    }

    while (remainingTimeS > 0.0)
    {
        /* Bound each Euler step to 20 ms so the linearized process model
         * stays a good approximation even over a long prediction gap. */
        const double timeStepS = std::min(remainingTimeS, 0.02);

        /* Re-evaluate Alpha's process model at the engine's current
         * state before every substep, since it is nonlinear. */
        const StateVector currentState = filter.getState();
        StateVector stateDerivative;
        StateMatrix processJacobian;
        computeProcessModel(currentState, stateDerivative, processJacobian);

        /* Advance state and covariance by exactly one bounded step. */
        const FilterStatus predictStatus =
            filter.predict(timeStepS, stateDerivative, processJacobian,
                           processNoise);

        if (predictStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            /* Stop propagating as soon as a step fails; filterTimestampS
             * intentionally stays at the last successfully reached time. */
            return predictStatus;
        }

        /* Wrap roll, pitch and yaw back into the canonical [-pi, pi]
         * range after integration, then write the wrapped state back into
         * the engine, which has no knowledge of which states are angles. */
        StateVector wrapped = filter.getState();
        wrapped(3) = wrapAngle(wrapped(3));
        wrapped(4) = wrapAngle(wrapped(4));
        wrapped(5) = wrapAngle(wrapped(5));

        const FilterStatus setStatus = filter.setState(wrapped);

        if (setStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return setStatus;
        }

        /*!
         * Advance the gyro-only yaw cross-check by the same bounded step,
         * using only the IMU's own gyro reading -- never wheel's, which is
         * exactly what this exists to check against. See
         * gyroOnlyYawRad's doc comment.
         */
        gyroOnlyYawRad =
            wrapAngle(gyroOnlyYawRad + latestGyroYawRateRadps * timeStepS);

        /* Record how much time this step actually covered. */
        filterTimestampS += timeStepS;
        remainingTimeS -= timeStepS;
    }

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
