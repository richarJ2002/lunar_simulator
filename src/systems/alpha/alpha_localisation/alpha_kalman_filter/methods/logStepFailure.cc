/*!
 * @File:         logStepFailure.cc
 *
 * @Brief:        Implements throttled logging of a rejected EKF step.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::logStepFailure(FilterStatus status_in)
{
    /*!
     * Throttled to avoid flooding the log if the filter rejects steps
     * repeatedly (e.g. during a sustained numerical failure).
     */
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Continuous EKF step failed with status %u",
                         static_cast<unsigned int>(status_in));
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
