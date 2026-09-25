/*!
 * @file            update.cc
 *
 * @brief           Implements one estimate low-pass filter update.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/objects/EstimateLowPassFilter.h"

/* C++ Standard Library Includes */
#include <cmath>

namespace control::estimate_filter
{

namespace
{

/*!
 * Exact discrete gain of a first-order low-pass with corner frequency
 * cutoffHz_in over a step of dt_s_in: alpha = 1 - exp(-dt / tau) with
 * tau = 1 / (2 pi f_c).
 */
double calculateFirstOrderGain(double cutoffHz_in, double dt_s_in)
{
    constexpr double TWO_PI = 6.283185307179586;
    return 1.0 - std::exp(-dt_s_in * TWO_PI * cutoffHz_in);
}

} /* anonymous namespace */

FilterUpdateResult
    EstimateLowPassFilter::update(const EstimateSample &sample_in,
                                  bool                  shouldReset_in)
{
    /* A non-finite sample would poison every later output. */
    if (!std::isfinite(sample_in.stamp_s) ||
        !sample_in.position_fixed_m.allFinite() ||
        !sample_in.orientation_bodyToFixed.coeffs().allFinite() ||
        !(sample_in.orientation_bodyToFixed.norm() > 0.0) ||
        !sample_in.linearVelocity_body_mPerS.allFinite() ||
        !sample_in.angularVelocity_body_radPerS.allFinite())
    {
        return FilterUpdateResult::FILTER_UPDATE_RESULT_SKIPPED;
    }

    const double dt_s = sample_in.stamp_s - output.stamp_s;
    if (!hasState || shouldReset_in || dt_s > configuration.maximumGapS)
    {
        reset(sample_in);
        return FilterUpdateResult::FILTER_UPDATE_RESULT_RESET;
    }

    /* A repeated or backwards stamp carries no new time to filter over. */
    if (!(dt_s > 0.0))
    {
        return FilterUpdateResult::FILTER_UPDATE_RESULT_SKIPPED;
    }

    /* Body twist: two identical first-order stages (critically damped). */
    const double velocityGain =
        calculateFirstOrderGain(configuration.velocityCutoffHz, dt_s);
    firstStageLinearVelocity_body_mPerS +=
        velocityGain * (sample_in.linearVelocity_body_mPerS -
                        firstStageLinearVelocity_body_mPerS);
    firstStageAngularVelocity_body_radPerS +=
        velocityGain * (sample_in.angularVelocity_body_radPerS -
                        firstStageAngularVelocity_body_radPerS);
    output.linearVelocity_body_mPerS +=
        velocityGain * (firstStageLinearVelocity_body_mPerS -
                        output.linearVelocity_body_mPerS);
    output.angularVelocity_body_radPerS +=
        velocityGain * (firstStageAngularVelocity_body_radPerS -
                        output.angularVelocity_body_radPerS);

    /* Position: predict with the filtered velocity expressed in the fixed
     * frame, then pull toward the input position. */
    const Eigen::Vector3d predictedPosition_fixed_m =
        output.position_fixed_m + output.orientation_bodyToFixed *
                                      output.linearVelocity_body_mPerS * dt_s;
    const double positionGain =
        calculateFirstOrderGain(configuration.positionCutoffHz, dt_s);
    output.position_fixed_m =
        predictedPosition_fixed_m +
        positionGain * (sample_in.position_fixed_m - predictedPosition_fixed_m);

    /* Attitude: shortest-path spherical interpolation toward the input. */
    const double attitudeGain =
        calculateFirstOrderGain(configuration.attitudeCutoffHz, dt_s);
    output.orientation_bodyToFixed =
        output.orientation_bodyToFixed
            .slerp(attitudeGain, sample_in.orientation_bodyToFixed.normalized())
            .normalized();

    output.stamp_s = sample_in.stamp_s;
    return FilterUpdateResult::FILTER_UPDATE_RESULT_FILTERED;
}

} /* namespace control::estimate_filter */
