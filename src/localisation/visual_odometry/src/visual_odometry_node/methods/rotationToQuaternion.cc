/*!
 * @File:         rotationToQuaternion.cc
 *
 * @Brief:        Implements rotation-matrix-to-quaternion conversion.
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
#include <cmath>

namespace localisation::visual_odometry
{

geometry_msgs::msg::Quaternion
VisualOdometryNode::rotationToQuaternion(const cv::Matx44d &transform_in)
{
    /* Sum of the rotation's diagonal, used to pick the numerically
     * stable branch below. */
    const double trace =
        transform_in(0, 0) + transform_in(1, 1) + transform_in(2, 2);

    /* Quaternion this function assembles and returns. */
    geometry_msgs::msg::Quaternion result;

    /*!
     * Standard "largest diagonal term" quaternion extraction (Shepperd's
     * method). Using the trace when it is positive, or otherwise the
     * largest of the three rotation-matrix diagonal entries, as the pivot
     * for the square root keeps the computation numerically stable for
     * every rotation angle -- a naive single-branch formula loses precision
     * or divides by a near-zero term close to certain rotations. Each
     * branch below applies the same closed-form formula pivoted on a
     * different diagonal term, so its four component assignments are one
     * inseparable step rather than four independent ones.
     */
    if (trace > 0.0)
    {
        /* Pivot on the trace; this branch is stable whenever it is
         * positive. */
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        result.w = 0.25 * scale;
        result.x = (transform_in(2, 1) - transform_in(1, 2)) / scale;
        result.y = (transform_in(0, 2) - transform_in(2, 0)) / scale;
        result.z = (transform_in(1, 0) - transform_in(0, 1)) / scale;
    }
    else if (transform_in(0, 0) > transform_in(1, 1) &&
             transform_in(0, 0) > transform_in(2, 2))
    {
        /* Pivot on the x-axis diagonal term, the largest of the three. */
        const double scale =
            2.0 * std::sqrt(1.0 + transform_in(0, 0) - transform_in(1, 1) -
                            transform_in(2, 2));
        result.w = (transform_in(2, 1) - transform_in(1, 2)) / scale;
        result.x = 0.25 * scale;
        result.y = (transform_in(0, 1) + transform_in(1, 0)) / scale;
        result.z = (transform_in(0, 2) + transform_in(2, 0)) / scale;
    }
    else if (transform_in(1, 1) > transform_in(2, 2))
    {
        /* Pivot on the y-axis diagonal term, the largest of the three. */
        const double scale =
            2.0 * std::sqrt(1.0 + transform_in(1, 1) - transform_in(0, 0) -
                            transform_in(2, 2));
        result.w = (transform_in(0, 2) - transform_in(2, 0)) / scale;
        result.x = (transform_in(0, 1) + transform_in(1, 0)) / scale;
        result.y = 0.25 * scale;
        result.z = (transform_in(1, 2) + transform_in(2, 1)) / scale;
    }
    else
    {
        /* Pivot on the z-axis diagonal term, the largest of the three. */
        const double scale =
            2.0 * std::sqrt(1.0 + transform_in(2, 2) - transform_in(0, 0) -
                            transform_in(1, 1));
        result.w = (transform_in(1, 0) - transform_in(0, 1)) / scale;
        result.x = (transform_in(0, 2) + transform_in(2, 0)) / scale;
        result.y = (transform_in(1, 2) + transform_in(2, 1)) / scale;
        result.z = 0.25 * scale;
    }

    /* Return the assembled unit quaternion. */
    return result;
}

} /* namespace localisation::visual_odometry */
