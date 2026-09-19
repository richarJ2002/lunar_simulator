/*!
 * @File:         prepareOdometry.cc
 *
 * @Brief:        Validates and reframes one estimate odometry message.
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

bool AlphaKalmanFilterNode::prepareOdometry(const nav_msgs::msg::Odometry &odometry_in,
                                      const std::string &parentFrame_in,
                                      const std::string &childFrame_in,
                                      nav_msgs::msg::Odometry &odometry_out)
{
    /* Reject the message outright rather than build a transform or path
     * point from a NaN/degenerate pose. */
    if (!isPoseValid(odometry_in.pose.pose))
    {
        /* Bound log spam if the estimator keeps producing bad poses. */
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Ignoring an invalid Alpha pose");

        /* Nothing further can be done with this sample; tell the caller
         * so it also stops. */
        return false;
    }

    /* Start from a copy of the validated odometry message. */
    odometry_out = odometry_in;

    /*!
     * The Kalman filter node does not know the TF frame ids this node
     * publishes under, so the parent frame id is overwritten with this
     * node's configured value.
     */
    odometry_out.header.frame_id = parentFrame_in;

    /* Overwrite the child frame id the same way. */
    odometry_out.child_frame_id = childFrame_in;

    /* Tell the caller the reframed message is valid to use. */
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
