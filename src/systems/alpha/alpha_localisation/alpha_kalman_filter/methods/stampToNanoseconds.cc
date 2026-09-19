/*!
 * @File:         stampToNanoseconds.cc
 *
 * @Brief:        Converts a ROS timestamp to a single integer nanosecond
 *                count.
 *
 * @Date:         15/09/2026
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

std::int64_t AlphaKalmanFilterNode::stampToNanoseconds(
    const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * ROS splits a timestamp into whole seconds and a nanosecond remainder;
     * combine both fields into one linear count so samples can be compared
     * and subtracted directly.
     */
    return static_cast<std::int64_t>(stamp_in.sec) *
               static_cast<std::int64_t>(NANOSECONDS_PER_SECOND) +
           static_cast<std::int64_t>(stamp_in.nanosec);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
