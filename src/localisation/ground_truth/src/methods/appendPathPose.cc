/*!
 * @File:         appendPathPose.cc
 *
 * @Brief:        Adds a time-sampled truth pose to the bounded path.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/GroundTruthNode.h"

/* Data include */
#include <geometry_msgs/msg/pose_stamped.hpp>

/* Generic Libraries */
/* None */

namespace localisation::ground_truth
{

void GroundTruthNode::appendPathPose(
    const nav_msgs::msg::Odometry &odometry_in)
{
    /*!
     * Convert the incoming timestamp into one linear nanosecond count so it
     * can be compared against the last retained sample below.
     */
    const std::int64_t currentStampNs =
        stampToNanoseconds(odometry_in.header.stamp);

    /* Detect a simulation reset or clock jump back in time. */
    if (lastPathStampNs >= 0 && currentStampNs < lastPathStampNs)
    {
        /* The retained path is no longer time-consistent; clear it. */
        groundTruthPath.poses.clear();

        /* Forget the last sample so the next one is always accepted. */
        lastPathStampNs = -1;
    }

    /*!
     * Ground truth arrives far denser than a comparison path needs; skip
     * this sample unless the configured sample period has elapsed.
     */
    if (lastPathStampNs >= 0 &&
        currentStampNs - lastPathStampNs < pathSamplePeriodNs)
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
    groundTruthPath.header = odometry_in.header;

    /* Append the new sample to the end of the retained path. */
    groundTruthPath.poses.push_back(pose);

    /*!
     * Once the path exceeds its configured maximum length, drop the oldest
     * retained pose so it stays bounded over a long run.
     */
    if (groundTruthPath.poses.size() > pathMaximumPoses)
    {
        /* Erase the oldest (first) element of the path. */
        groundTruthPath.poses.erase(groundTruthPath.poses.begin());
    }

    /* Record this sample's time for the next sample-period check. */
    lastPathStampNs = currentStampNs;

    /* Republish the whole path so subscribers see the new sample. */
    p_pathPublisher->publish(groundTruthPath);
}

} /* namespace localisation::ground_truth */
