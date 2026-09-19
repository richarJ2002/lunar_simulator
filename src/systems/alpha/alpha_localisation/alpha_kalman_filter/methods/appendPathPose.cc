/*!
 * @File:         appendPathPose.cc
 *
 * @Brief:        Appends a time-sampled pose to a bounded path.
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

void AlphaKalmanFilterNode::appendPathPose(
    const nav_msgs::msg::Odometry &odometry_in,
    nav_msgs::msg::Path &path_inout, std::int64_t &lastStampNs_inout,
    rclcpp::Publisher<nav_msgs::msg::Path> &publisher_inout)
{
    /*!
     * Convert the incoming timestamp into one linear nanosecond count so it
     * can be compared against the last retained sample below.
     */
    const std::int64_t currentStampNs =
        stampToNanoseconds(odometry_in.header.stamp);

    /*!
     * A timestamp going backwards means the simulation clock jumped or
     * restarted (e.g. use_sim_time reset); the stale path would otherwise
     * show a discontinuous jump, so it is cleared and resampling restarts
     * from this message.
     */
    if (lastStampNs_inout >= 0 && currentStampNs < lastStampNs_inout)
    {
        /* The retained path is no longer time-consistent; clear it. */
        path_inout.poses.clear();

        /* Forget the last sample so the next one is always accepted. */
        lastStampNs_inout = -1;
    }

    /*!
     * Throttle path growth to the configured sample period regardless of how
     * fast the estimate topic actually publishes.
     */
    if (lastStampNs_inout >= 0 &&
        currentStampNs - lastStampNs_inout < pathSamplePeriodNs)
    {
        /* Too soon since the last retained sample; publish nothing. */
        return;
    }

    /* Build one stamped pose from the current odometry message. */
    geometry_msgs::msg::PoseStamped pose;

    /* Copy the timestamp and frame id onto the new pose. */
    pose.header = odometry_in.header;

    /* Copy the rover's position and orientation onto the new pose. */
    pose.pose = odometry_in.pose.pose;

    /* Keep the path's own header aligned with its newest sample. */
    path_inout.header = odometry_in.header;

    /* Append the new sample to the end of the retained path. */
    path_inout.poses.push_back(pose);

    /*!
     * Drop the oldest sample once the retained history exceeds the
     * configured bound, keeping memory and RViz redraw cost bounded for
     * long-running simulations.
     */
    if (path_inout.poses.size() > pathMaximumPoses)
    {
        /* Erase the oldest (first) element of the path. */
        path_inout.poses.erase(path_inout.poses.begin());
    }

    /* Record this sample's time for the next sample-period check. */
    lastStampNs_inout = currentStampNs;

    /* Republish the whole path so subscribers see the new sample. */
    publisher_inout.publish(path_inout);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
