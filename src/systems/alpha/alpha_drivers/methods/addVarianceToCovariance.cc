/*!
 * @file            addVarianceToCovariance.cc
 *
 * @brief           Adds a variance to each covariance diagonal.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::addVarianceToCovariance(
    double variance_in, std::array<double, 9U> &covariance_inout)
{
    /*!
     * ROS represents "unknown covariance" with a negative first diagonal
     * entry (REP 103); this node knows the true injected variance, so an
     * unknown covariance is replaced with an all-zero matrix before the
     * known variance is added onto the diagonal.
     */
    if (covariance_inout[0U] < 0.0)
    {
        /* Replace the "unknown" sentinel with a known all-zero matrix. */
        covariance_inout.fill(0.0);
    }

    /* Add the variance onto the x-axis diagonal entry. */
    covariance_inout[0U] += variance_in;

    /* Add the variance onto the y-axis diagonal entry. */
    covariance_inout[4U] += variance_in;

    /* Add the variance onto the z-axis diagonal entry. */
    covariance_inout[8U] += variance_in;
}

} /* namespace systems::alpha::alpha_drivers */
