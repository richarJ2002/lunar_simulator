/*!
 * @File:         multiply.cc
 *
 * @Brief:        Implements homogeneous transform composition.
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

cv::Matx44d VisualOdometryNode::multiply(const cv::Matx44d &left_in,
                                         const cv::Matx44d &right_in)
{
    /*!
     * A thin named wrapper around OpenCV's operator* keeps every transform
     * composition in this file self-documenting about multiplication order
     * (left_in is applied after right_in, i.e. left_in * right_in maps a
     * point expressed in right_in's source frame all the way to left_in's
     * target frame) instead of relying on a bare operator call at each site.
     */
    return left_in * right_in;
}

} /* namespace localisation::visual_odometry */
