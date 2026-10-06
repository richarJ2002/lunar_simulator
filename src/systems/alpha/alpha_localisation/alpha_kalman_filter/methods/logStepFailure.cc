/*!
 * @File:         logStepFailure.cc
 *
 * @Brief:        Implements throttled logging of a rejected EKF step.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
#include "console/console.h"

/* Object Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

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
    SRS_LOG_WARN_THROTTLE(get_logger(),
                            *get_clock(),
                            2000,
                            "EKF step failed (status %u)",
                            static_cast<unsigned int>(status_in));
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
