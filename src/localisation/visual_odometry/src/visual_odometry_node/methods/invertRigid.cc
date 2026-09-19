/*!
 * @File:         invertRigid.cc
 *
 * @Brief:        Implements closed-form rigid-transform inversion.
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

cv::Matx44d VisualOdometryNode::invertRigid(const cv::Matx44d &transform_in)
{
    /*!
     * Every transform this node builds is rigid: its upper-left 3x3 block is
     * an orthonormal rotation R and its last column is a translation t, i.e.
     *   transform_in = [ R  t ]
     *                  [ 0  1 ]
     * For an orthonormal R, R^-1 == R^T, so the inverse rigid transform is
     *   inverse = [ R^T  -R^T * t ]
     *             [ 0     1       ]
     * This is exact and far cheaper than a general 4x4 matrix inverse, and
     * it avoids amplifying floating-point error the way a generic inverse
     * could on a near-singular matrix.
     */

    /* Start the result at identity; only the rotation/translation blocks
     * below are overwritten. */
    cv::Matx44d inverse = cv::Matx44d::eye();

    /* Extracted 3x3 rotation block of transform_in. */
    cv::Matx33d rotation;

    /* Extracted translation column of transform_in. */
    cv::Vec3d translation;

    /* Copy the rotation block and translation column out of the packed
     * 4x4 input; one loop performs both extractions together. */
    for (int row = 0; row < 3; ++row)
    {
        translation[row] = transform_in(row, 3);
        for (int column = 0; column < 3; ++column)
        {
            rotation(row, column) = transform_in(row, column);
        }
    }

    /* R^-1 == R^T for an orthonormal rotation. */
    const cv::Matx33d rotationTranspose = rotation.t();

    /* -R^T * t, the inverse translation from the closed-form above. */
    const cv::Vec3d inverseTranslation = -(rotationTranspose * translation);

    /* Write the inverted rotation and translation back into the packed
     * 4x4 result; one loop performs both writes together. */
    for (int row = 0; row < 3; ++row)
    {
        inverse(row, 3) = inverseTranslation[row];
        for (int column = 0; column < 3; ++column)
        {
            inverse(row, column) = rotationTranspose(row, column);
        }
    }

    /* Return the fully assembled inverse transform. */
    return inverse;
}

} /* namespace localisation::visual_odometry */
