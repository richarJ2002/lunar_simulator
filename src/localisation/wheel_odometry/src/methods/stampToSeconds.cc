/*!
 * @File:         stampToSeconds.cc
 *
 * @Brief:        Implements ROS timestamp to floating-point seconds
 *                conversion.
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

double
WheelOdometryNode::stampToSeconds(const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * ROS timestamps are split into whole seconds and nanoseconds so that
     * time arithmetic never loses precision the way a single double would
     * for large wall-clock values. This node only needs relative deltas at
     * millisecond precision, so recombining into one double here is safe.
     */
    return static_cast<double>(stamp_in.sec) +
           1.0e-9 * static_cast<double>(stamp_in.nanosec);
}

} /* namespace localisation::wheel_odometry */
