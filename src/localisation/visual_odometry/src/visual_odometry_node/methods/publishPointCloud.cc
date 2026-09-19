/*!
 * @File:         publishPointCloud.cc
 *
 * @Brief:        Implements sparse inlier point-cloud publication.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* Data include */
#include <sensor_msgs/point_cloud2_iterator.hpp>

/* Generic Libraries */
#include <cstddef>
#include <vector>

namespace localisation::visual_odometry
{

void
VisualOdometryNode::publishPointCloud(
    const builtin_interfaces::msg::Time &stamp_in,
    const std::vector<cv::Point3f> &correlatedPoints_in,
    const std::vector<int> &inlierIndices_in,
    const cv::Matx44d &mapFromOptical_in)
{
    /* Inlier points, transformed into the map frame, to publish below. */
    std::vector<cv::Point3f> mapPoints;

    /* Reserve up front since the final count is already known. */
    mapPoints.reserve(inlierIndices_in.size());

    /*!
     * inlierIndices_in indexes into correlatedPoints_in; bounds-check each
     * index defensively even though callers currently only ever pass
     * consistent pairs, since this is the boundary where a future caller
     * mistake would otherwise corrupt memory.
     */
    for (const int inlierIndex : inlierIndices_in)
    {
        /* Reject an index that would read outside correlatedPoints_in. */
        if (inlierIndex < 0 || static_cast<std::size_t>(inlierIndex) >=
                                   correlatedPoints_in.size())
        {
            continue;
        }

        /* The reconstructed point this inlier index refers to. */
        const cv::Point3f &point =
            correlatedPoints_in[static_cast<std::size_t>(inlierIndex)];

        /*!
         * Homogeneous transform: append a 1 so translation is applied, then
         * drop back to 3-D.
         */
        const cv::Vec4d pointOptical(point.x, point.y, point.z, 1.0);

        /* Apply the supplied optical-to-map transform. */
        const cv::Vec4d pointMap = mapFromOptical_in * pointOptical;

        /* Record the transformed point for publication below. */
        mapPoints.emplace_back(static_cast<float>(pointMap[0]),
                               static_cast<float>(pointMap[1]),
                               static_cast<float>(pointMap[2]));
    }

    /* Message to populate and publish below. */
    sensor_msgs::msg::PointCloud2 cloud;

    /* Timestamp the cloud with the frame it was reconstructed from. */
    cloud.header.stamp = stamp_in;

    /* Every point below is already expressed in this frame. */
    cloud.header.frame_id = odomFrame;

    /* An unorganized point cloud is always exactly one row tall. */
    cloud.height = 1U;

    /* No point in this cloud is ever NaN/Inf by construction. */
    cloud.is_dense = true;

    /* Helper that lays out the requested fields in the message. */
    sensor_msgs::PointCloud2Modifier modifier(cloud);

    /* Declare a plain float32 x/y/z point cloud. This ROS API is itself a
     * C-style variadic function; there is no non-vararg alternative. */
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    modifier.setPointCloud2FieldsByString(1, "xyz");

    /* Size the underlying buffer for exactly this many points. */
    modifier.resize(mapPoints.size());

    /* Writable view over the message's x field. */
    sensor_msgs::PointCloud2Iterator<float> xIterator(cloud, "x");

    /* Writable view over the message's y field. */
    sensor_msgs::PointCloud2Iterator<float> yIterator(cloud, "y");

    /* Writable view over the message's z field. */
    sensor_msgs::PointCloud2Iterator<float> zIterator(cloud, "z");

    /* Copy every transformed point into the message's packed buffer. */
    for (const cv::Point3f &point : mapPoints)
    {
        /* Write this point's x coordinate. */
        *xIterator = point.x;

        /* Write this point's y coordinate. */
        *yIterator = point.y;

        /* Write this point's z coordinate. */
        *zIterator = point.z;

        /* Advance to the next point's x slot. */
        ++xIterator;

        /* Advance to the next point's y slot. */
        ++yIterator;

        /* Advance to the next point's z slot. */
        ++zIterator;
    }

    /*!
     * Publishing unconditionally, even with zero points, lets subscribers
     * tell "no inliers this frame" apart from "node not running".
     */
    p_pointCloudPublisher->publish(cloud);
}

} /* namespace localisation::visual_odometry */
