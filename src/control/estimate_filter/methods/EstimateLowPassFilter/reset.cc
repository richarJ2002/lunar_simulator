/*!
 * @file            reset.cc
 *
 * @brief           Implements restarting the estimate filter from a sample.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/objects/EstimateLowPassFilterClass.h"

namespace control::estimate_filter
{

void EstimateLowPassFilter::reset(const EstimateSample &sample_in)
{
    output = sample_in;
    output.orientation_bodyToFixed.normalize();
    firstStageLinearVelocity_body_mPerS = sample_in.linearVelocity_body_mPerS;
    firstStageAngularVelocity_body_radPs =
        sample_in.angularVelocity_body_radPs;
    hasState = true;
}

} /* namespace control::estimate_filter */
