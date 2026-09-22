/*!
 * @file            calculateFixedFromBody.cc
 *
 * @brief           Implements startup-fixed transform rebasing.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/GroundTruthNode.h"

namespace localisation::ground_truth
{

tf2::Transform GroundTruthNode::calculateFixedFromBody(
    const tf2::Transform &mapFromFixed_in,
    const tf2::Transform &mapFromBody_in)
{
    return mapFromFixed_in.inverse() * mapFromBody_in;
}

} /* namespace localisation::ground_truth */
