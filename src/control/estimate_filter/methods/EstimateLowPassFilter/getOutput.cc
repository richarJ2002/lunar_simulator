/*!
 * @file            getOutput.cc
 *
 * @brief           Implements access to the filtered estimate.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/objects/EstimateLowPassFilterClass.h"

namespace control::estimate_filter
{

const EstimateSample &EstimateLowPassFilter::getOutput() const
{
    return output;
}

} /* namespace control::estimate_filter */
