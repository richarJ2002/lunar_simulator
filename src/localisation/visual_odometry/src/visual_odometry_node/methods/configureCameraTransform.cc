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

/* This is a private method with a single, already-reviewed call site
 * (the constructor); each argument is passed by name from a
 * distinctly-named local variable, not positionally from ambiguous
 * data. */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void VisualOdometryNode::configureCameraTransform(double cameraXM_in,
                                                  double cameraZM_in,
                                                  double cameraPitchRad_in)
{
    /* Cosine of the configured mount pitch, used to build the rotation
     * matrix below. */
    const double cosine = std::cos(cameraPitchRad_in);

    /* Sine of the configured mount pitch, used to build the rotation
     * matrix below. */
    const double sine = std::sin(cameraPitchRad_in);

    /*!
     * baseFromMount rotates the camera MOUNT frame (aligned with the body
     * axes before pitch is applied) into the body frame, by the configured
     * downward pitch about the body y axis:
     *   baseFromMount = [ cos(pitch)   0   sin(pitch) ]
     *                   [ 0            1   0          ]
     *                   [ -sin(pitch)  0   cos(pitch) ]
     */
    const cv::Matx33d
        baseFromMount(cosine, 0.0, sine, 0.0, 1.0, 0.0, -sine, 0.0, cosine);

    /*!
     * mountFromOptical rotates the OpenCV optical frame (x right, y down,
     * z forward, the convention solvePnPRansac and the pinhole camera model
     * both assume) into the mount frame (x forward, y left, z up, matching
     * the body-axis convention before pitch). This fixed 90-degree axis
     * remap is required once per camera model, independent of mount pose.
     */
    const cv::Matx33d
        mountFromOptical(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0);

    /* Compose mount-to-body and optical-to-mount to get optical-to-body
     * directly; multiplication order matters because rotations do not
     * commute. */
    const cv::Matx33d baseFromOptical = baseFromMount * mountFromOptical;

    /* Start the fixed transform at identity; only the rotation block and
     * translation column below are overwritten. */
    bodyFromOptical = cv::Matx44d::eye();

    /* Copy the derived rotation into the upper-left 3x3 block. */
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            bodyFromOptical(row, column) = baseFromOptical(row, column);
        }
    }

    /* Only x and z offsets are configurable; the camera is assumed centred
     * on the body y axis (no lateral offset). */
    bodyFromOptical(0, 3) = cameraXM_in;

    /* Set the z (vertical) mount offset. */
    bodyFromOptical(2, 3) = cameraZM_in;

    /*!
     * Seed the accumulated pose so that, before any motion is estimated, the
     * rover's world-frame pose (worldFromOptical composed with the inverse
     * of bodyFromOptical) equals identity at the body origin.
     */
    worldFromOptical = bodyFromOptical;
}

} /* namespace localisation::visual_odometry */
