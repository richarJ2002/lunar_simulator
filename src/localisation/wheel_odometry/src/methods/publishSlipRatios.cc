/*!
 * @File:         publishSlipRatios.cc
 *
 * @Brief:        Implements publication of the current six-wheel slip
 *                estimate.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::wheel_odometry
{

void WheelOdometryNode::publishSlipRatios()
{
    /* Build the flat six-element message from the current slip ratios. */
    std_msgs::msg::Float64MultiArray output;

    /* Copy all six wheel-order slip ratios into the message payload. */
    output.data.assign(slipRatios.begin(), slipRatios.end());

    /*!
     * The slip-ratio topic is latched (transient-local) at construction
     * so that a late-joining subscriber still sees the most recent
     * estimate.
     */
    slipRatioPublisher->publish(output);

    /* Record that at least one publication has happened, so a caller
     * can force one before any visual-odometry update has occurred. */
    hasPublishedSlipRatios = true;
}

} /* namespace localisation::wheel_odometry */
