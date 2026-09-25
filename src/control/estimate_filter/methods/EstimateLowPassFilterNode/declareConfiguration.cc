/*!
 * @file            declareConfiguration.cc
 *
 * @brief           Implements the estimate filter's parameter declaration.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "estimate_filter/objects/EstimateLowPassFilterNode.h"

namespace control::estimate_filter
{

LowPassConfiguration EstimateLowPassFilterNode::declareConfiguration()
{
    LowPassConfiguration configuration;
    configuration.velocityCutoffHz =
        declare_parameter<double>("velocity_cutoff_hz", 1.0);
    configuration.positionCutoffHz =
        declare_parameter<double>("position_cutoff_hz", 0.5);
    configuration.attitudeCutoffHz =
        declare_parameter<double>("attitude_cutoff_hz", 1.0);
    configuration.maximumGapS = declare_parameter<double>("maximum_gap_s", 0.5);
    if (!isConfigurationValid(configuration))
    {
        throw std::invalid_argument(
            "estimate filter cutoffs and gap must be finite and positive");
    }
    return configuration;
}

} /* namespace control::estimate_filter */
