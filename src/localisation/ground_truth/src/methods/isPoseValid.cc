/*!
 * @File:         isPoseValid.cc
 *
 * @Brief:        Tests whether a ground-truth pose can be published safely.
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
#include <cmath>

namespace localisation::ground_truth
{

bool GroundTruthNode::isPoseValid(const geometry_msgs::msg::Pose &pose_in)
{
    /*!
     * A zero-norm quaternion cannot represent a rotation; computing its
     * squared norm lets both that case and non-finite components be
     * checked together below.
     */
    const double orientationNormSquared =
        pose_in.orientation.x * pose_in.orientation.x +
        pose_in.orientation.y * pose_in.orientation.y +
        pose_in.orientation.z * pose_in.orientation.z +
        pose_in.orientation.w * pose_in.orientation.w;

    /*!
     * Gazebo can momentarily report a not-yet-settled or degenerate pose
     * (for example immediately after model spawn); reject it unless every
     * position component is finite and the orientation has a genuine,
     * finite, nonzero norm.
     */
    return std::isfinite(pose_in.position.x) &&
           std::isfinite(pose_in.position.y) &&
           std::isfinite(pose_in.position.z) &&
           std::isfinite(orientationNormSquared) &&
           orientationNormSquared > 1.0e-12;
}

} /* namespace localisation::ground_truth */
