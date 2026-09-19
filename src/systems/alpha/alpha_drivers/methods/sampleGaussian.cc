/*!
 * @file            sampleGaussian.cc
 *
 * @brief           Samples zero-mean Gaussian noise.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

double AlphaDriverNode::sampleGaussian(double standardDeviation_in)
{
    /*!
     * Skip constructing a distribution (and consuming engine state) when
     * noise is disabled or this channel's configured stddev is exactly
     * zero, so disabling noise does not perturb the deterministic sequence
     * consumed by other channels.
     */
    if (!noiseEnabled || standardDeviation_in == 0.0)
    {
        /* No noise for this channel; report exactly zero. */
        return 0.0;
    }

    /* Build a zero-mean distribution at this channel's configured
     * standard deviation. */
    std::normal_distribution<double> distribution(0.0, standardDeviation_in);

    /* Draw and return one sample, advancing the shared engine's state. */
    return distribution(randomEngine);
}

} /* namespace systems::alpha::alpha_drivers */
