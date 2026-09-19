/*!
 * @File:         stampToSeconds.cc
 *
 * @Brief:        Implements ROS timestamp to seconds conversion.
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

double
VisualOdometryNode::stampToSeconds(const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * ROS timestamps split whole seconds and nanoseconds into two integer
     * fields specifically to avoid double-precision rounding for large
     * epoch values; combine them into one double only here, at the point of
     * use, where sub-frame timing precision is all that matters.
     */
    return static_cast<double>(stamp_in.sec) +
           1.0e-9 * static_cast<double>(stamp_in.nanosec);
}

} /* namespace localisation::visual_odometry */
