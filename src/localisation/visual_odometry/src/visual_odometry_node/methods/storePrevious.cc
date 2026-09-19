/*!
 * @File:         storePrevious.cc
 *
 * @Brief:        Implements retention of the current frame as "previous".
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::visual_odometry
{

void VisualOdometryNode::storePrevious(
    const cv::Mat &left_in, const cv::Mat &right_in,
    const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * cv::Mat uses shared, reference-counted storage; clone() forces a deep
     * copy so a retained "previous" frame cannot be silently mutated later
     * through the caller's now-stale reference to the same underlying
     * buffer.
     */
    previousLeft = left_in.clone();

    /* Deep-copy the right frame for the same reason as the left frame. */
    previousRight = right_in.clone();

    /* Retain this frame's timestamp, in seconds, for the next callback's
     * elapsed-time calculation. */
    previousStampS = stampToSeconds(stamp_in);
}

} /* namespace localisation::visual_odometry */
