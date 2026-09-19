/*!
 * @File:         publishEstimatedPath.cc
 *
 * @Brief:        Validates one fused estimate and, if valid, broadcasts its
 *                TF transform and appends it to the retained path.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::publishEstimatedPath(
    const nav_msgs::msg::Odometry &odometry_in)
{
    /* Filled in below by prepareOdometry() if the estimate is valid. */
    nav_msgs::msg::Odometry preparedOdometry;

    /*!
     * Validation/reframing happens once, then both downstream consumers
     * (the TF broadcast and the retained path) reuse the same prepared
     * message.
     */
    if (!prepareOdometry(odometry_in, mapFrame, estimatedBaseFrame,
                         preparedOdometry))
    {
        /* The estimate was invalid; nothing more to do this call. */
        return;
    }

    /* Broadcast the estimated map-to-body transform for this sample. */
    publishTransform(preparedOdometry);

    /* Add this sample to the retained, rate-limited estimated path. */
    appendPathPose(preparedOdometry, estimatedPath, lastEstimatedPathStampNs,
                   *p_estimatedPathPublisher);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
