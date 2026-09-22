/*!
 * @File:         configureCameraTransform.cc
 *
 * @Brief:        Implements derivation of the fixed camera-to-body transform.
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

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
cv::Matx44d
    VisualOdometryNode::calculateBodyFromOptical(double cameraXM_in,
                                                 double cameraZM_in,
                                                 double cameraPitchRad_in)
{
    const double cosine = std::cos(cameraPitchRad_in);
    const double sine   = std::sin(cameraPitchRad_in);
    const cv::Matx33d
        baseFromMount(cosine, 0.0, sine, 0.0, 1.0, 0.0, -sine, 0.0, cosine);
    const cv::Matx33d
        mountFromOptical(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0);
    const cv::Matx33d baseFromOptical = baseFromMount * mountFromOptical;

    cv::Matx44d transform = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            transform(row, column) = baseFromOptical(row, column);
        }
    }
    transform(0, 3) = cameraXM_in;
    transform(2, 3) = cameraZM_in;
    return transform;
}

/* This is a private method with a single, already-reviewed call site
 * (the constructor); each argument is passed by name from a
 * distinctly-named local variable, not positionally from ambiguous data. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void VisualOdometryNode::configureCameraTransform(double cameraXM_in,
                                                  double cameraZM_in,
                                                  double cameraPitchRad_in)
{
    bodyFromOptical =
        calculateBodyFromOptical(cameraXM_in, cameraZM_in, cameraPitchRad_in);

    /*!
     * Seed the accumulated pose so that, before any motion is estimated, the
     * rover's world-frame pose (worldFromOptical composed with the inverse
     * of bodyFromOptical) equals identity at the body origin.
     */
    worldFromOptical = bodyFromOptical;
}

} /* namespace localisation::visual_odometry */
