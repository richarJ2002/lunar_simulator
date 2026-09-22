/*!
 * @File:         measurementVariances.cc
 *
 * @Brief:        Implements derivation of per-state measurement variances
 *                from a message's reported covariance.
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
#include <cmath>
#include <cstddef>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::MeasurementVarianceVector
    AlphaKalmanFilterNode::measurementVariances(
        const nav_msgs::msg::Odometry &message_in,
        double                         minimumVariance_in)
{
    /*!
     * Start from the configured floor for every state; only the six pose
     * and six twist diagonal covariance entries below can raise a specific
     * state's variance above that floor.
     */
    MeasurementVarianceVector variances =
        MeasurementVarianceVector::Constant(minimumVariance_in);

    for (Eigen::Index index = 0; index < 6; ++index)
    {
        /*!
         * nav_msgs::msg::Odometry stores each 6x6 covariance as a
         * row-major 36-element array; the diagonal entry for axis `index`
         * sits at index * 6 + index.
         */
        const std::size_t covarianceIndex =
            static_cast<std::size_t>(index * 6 + index);

        /* Read this axis's reported pose variance. */
        const double poseVariance = message_in.pose.covariance[covarianceIndex];

        /* Read this axis's reported twist variance. */
        const double twistVariance =
            message_in.twist.covariance[covarianceIndex];

        /*!
         * An unreported or invalid covariance entry (commonly left at 0 or
         * a negative sentinel by an upstream publisher) must not be trusted
         * as an authoritative near-zero variance, so only a finite positive
         * value is allowed to override the floor.
         */
        if (std::isfinite(poseVariance) && poseVariance > 0.0)
        {
            /* Raise this pose state's variance above the floor. */
            variances(index) = std::max(minimumVariance_in, poseVariance);
        }

        if (std::isfinite(twistVariance) && twistVariance > 0.0)
        {
            /* Raise this twist state's variance above the floor. */
            variances(index + 6) = std::max(minimumVariance_in, twistVariance);
        }
    }

    /* Hand the fully populated variance vector back to the caller. */
    return variances;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
