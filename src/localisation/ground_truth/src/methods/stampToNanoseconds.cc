/*!
 * @File:         stampToNanoseconds.cc
 *
 * @Brief:        Converts a ROS timestamp to nanoseconds.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/GroundTruthNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::ground_truth
{

std::int64_t GroundTruthNode::stampToNanoseconds(
    const builtin_interfaces::msg::Time &stamp_in)
{
    /*!
     * ROS message timestamps split whole seconds from the sub-second
     * nanosecond remainder; combine both fields into one linear count so
     * samples can be compared and subtracted directly.
     */
    return static_cast<std::int64_t>(stamp_in.sec) *
               static_cast<std::int64_t>(NANOSECONDS_PER_SECOND) +
           static_cast<std::int64_t>(stamp_in.nanosec);
}

} /* namespace localisation::ground_truth */
