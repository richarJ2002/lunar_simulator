/*!
 * @File:         updatePose.cc
 *
 * @Brief:        Implements accepted PnP RANSAC pose-estimate application.
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
#include <vector>

#include <opencv2/calib3d.hpp>

namespace localisation::visual_odometry
{

/* This is a private method with a single, already-reviewed call site
 * (handleStereoCallBack.cc), which passes both cv::Mat arguments directly from
 * solvePnPRansac's own out-parameters in matching order. */
void VisualOdometryNode::updatePose(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    const cv::Mat &rotationVector_in, const cv::Mat &translationVector_in,
    const builtin_interfaces::msg::Time &stamp_in, double dtS_in,
    const std::vector<cv::Point3f> &correlatedPoints_in,
    const std::vector<int> &inlierIndices_in)
{
    /* Expanded 3x3 rotation matrix from the compact Rodrigues vector,
     * filled in by cv::Rodrigues below. */
    cv::Mat rotationMatrix;

    /* solvePnPRansac returns a compact Rodrigues rotation vector; expand
     * it to a full 3x3 rotation matrix so it can be assembled into the
     * 4x4 homogeneous transform below. */
    cv::Rodrigues(rotationVector_in, rotationMatrix);

    /* Start the previous-to-current transform at identity; only the
     * rotation block and translation column below are overwritten. */
    cv::Matx44d currentFromPrevious = cv::Matx44d::eye();

    /*!
     * Assemble the 4x4 homogeneous currentFromPrevious transform (previous
     * optical frame -> current optical frame) from the translation vector
     * and expanded rotation matrix above.
     */
    for (int row = 0; row < 3; ++row)
    {
        /* Copy this row's translation component from PnP's output. */
        currentFromPrevious(row, 3) = translationVector_in.at<double>(row);

        /* Copy this row's rotation components from the expanded matrix. */
        for (int column = 0; column < 3; ++column)
        {
            currentFromPrevious(row, column) =
                rotationMatrix.at<double>(row, column);
        }
    }

    /* The pose before this update, needed unmodified below to express the
     * point cloud in the frame it was actually observed in. */
    const cv::Matx44d previousWorldFromOptical = worldFromOptical;

    /* The pre-update body pose, published as the odometry twist's "before"
     * state. */
    const cv::Matx44d previousWorldFromBody =
        multiply(previousWorldFromOptical, invertRigid(bodyFromOptical));

    /*!
     * Accumulate the estimated motion into the running world-frame pose.
     * worldFromOptical currently maps the PREVIOUS optical frame into world;
     * right-multiplying by the inverse of currentFromPrevious (i.e. by
     * previousFromCurrent) advances it to map the CURRENT optical frame into
     * world instead. previousWorldFromOptical is saved first because it is
     * needed, still un-advanced, when publishing the point cloud below.
     */
    worldFromOptical =
        multiply(worldFromOptical, invertRigid(currentFromPrevious));

    /* The post-update body pose, published as the odometry twist's
     * "after" state. */
    const cv::Matx44d worldFromBody =
        multiply(worldFromOptical, invertRigid(bodyFromOptical));

    /* Publish the finite-differenced pose/twist for this step. */
    publishOdometry(stamp_in, dtS_in, inlierIndices_in.size(),
                    previousWorldFromBody, worldFromBody);

    /*!
     * Publish the inlier points in the frame they were reconstructed in
     * (the PREVIOUS optical frame), transformed by the pre-update
     * previousWorldFromOptical so the cloud lines up with the pose that was
     * true when those points were observed.
     */
    publishPointCloud(stamp_in, correlatedPoints_in, inlierIndices_in,
                      previousWorldFromOptical);
}

} /* namespace localisation::visual_odometry */
