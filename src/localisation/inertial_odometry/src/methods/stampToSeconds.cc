/*!
 * @File:         stampToSeconds.cc
 *
 * @Brief:        Converts a ROS timestamp to seconds.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/InertialOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::inertial_odometry
{

double InertialOdometryNode::stampToSeconds(
    const builtin_interfaces::msg::Time &stamp)
{
    /*!
     * ROS stamps split time into whole seconds and a nanosecond remainder;
     * recombine them into one floating-point second count for arithmetic.
     */
    return static_cast<double>(stamp.sec) +
           1.0e-9 * static_cast<double>(stamp.nanosec);
}

} /* namespace localisation::inertial_odometry */
