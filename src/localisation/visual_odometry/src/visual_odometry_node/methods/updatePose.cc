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

namespace
{

cv::Matx33d rotationBlock(const cv::Matx44d &transform_in)
{
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            rotation(row, column) = transform_in(row, column);
        }
    }
    return rotation;
}

cv::Matx33d skew(const cv::Vec3d &vector_in)
{
    return cv::Matx33d(0.0,
                       -vector_in[2],
                       vector_in[1],
                       vector_in[2],
                       0.0,
                       -vector_in[0],
                       -vector_in[1],
                       vector_in[0],
                       0.0);
}

VisualOdometryNode::PoseCovariance adjoint(const cv::Matx44d &transform_in)
{
    VisualOdometryNode::PoseCovariance result =
        VisualOdometryNode::PoseCovariance::zeros();
    const cv::Matx33d rotation = rotationBlock(transform_in);
    const cv::Vec3d   translation(transform_in(0, 3),
                                transform_in(1, 3),
                                transform_in(2, 3));
    const cv::Matx33d translationRotation = skew(translation) * rotation;
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            result(row, column)         = rotation(row, column);
            result(row, column + 3)     = translationRotation(row, column);
            result(row + 3, column + 3) = rotation(row, column);
        }
    }
    return result;
}

} /* anonymous namespace */

/* This is a private method with a single, already-reviewed call site
 * (handleStereoCallBack.cc), which passes both cv::Mat arguments directly from
 * solvePnPRansac's own out-parameters in matching order. */
void VisualOdometryNode::updatePose(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    const cv::Mat                       &rotationVector_in,
    const cv::Mat                       &translationVector_in,
    const builtin_interfaces::msg::Time &stamp_in,
    double                               dtS_in,
    const std::vector<cv::Point3f>      &correlatedPoints_in,
    const std::vector<int>              &inlierIndices_in,
    const VisualPoseQuality             &quality_in)
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

    const cv::Matx44d previousFromCurrent = invertRigid(currentFromPrevious);
    const cv::Matx44d opticalFromBody     = invertRigid(bodyFromOptical);
    const cv::Matx44d previousBodyFromCurrentBody =
        multiply(multiply(bodyFromOptical, previousFromCurrent),
                 opticalFromBody);

    const PoseCovariance inverseMotionAdjoint   = adjoint(previousFromCurrent);
    const PoseCovariance bodyFromOpticalAdjoint = adjoint(bodyFromOptical);
    const PoseCovariance relativeBodyCovariance =
        bodyFromOpticalAdjoint * inverseMotionAdjoint *
        quality_in.relativeCovariance * inverseMotionAdjoint.t() *
        bodyFromOpticalAdjoint.t();

    const cv::Matx33d previousWorldRotation =
        rotationBlock(previousWorldFromBody);
    const cv::Vec3d   relativeTranslationBody(previousBodyFromCurrentBody(0, 3),
                                            previousBodyFromCurrentBody(1, 3),
                                            previousBodyFromCurrentBody(2, 3));
    PoseCovariance    priorJacobian = PoseCovariance::eye();
    const cv::Matx33d attitudeToPosition =
        -previousWorldRotation * skew(relativeTranslationBody);
    PoseCovariance relativeToFixed = PoseCovariance::zeros();
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            priorJacobian(row, column + 3) = attitudeToPosition(row, column);
            relativeToFixed(row, column)   = previousWorldRotation(row, column);
            relativeToFixed(row + 3, column + 3) =
                previousWorldRotation(row, column);
        }
    }
    accumulatedPoseCovariance =
        priorJacobian * accumulatedPoseCovariance * priorJacobian.t() +
        relativeToFixed * relativeBodyCovariance * relativeToFixed.t();
    accumulatedPoseCovariance =
        0.5 * (accumulatedPoseCovariance + accumulatedPoseCovariance.t());

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
    publishOdometry(stamp_in,
                    dtS_in,
                    previousWorldFromBody,
                    worldFromBody,
                    accumulatedPoseCovariance,
                    relativeBodyCovariance);

    /*!
     * Publish the inlier points in the frame they were reconstructed in
     * (the PREVIOUS optical frame), transformed by the pre-update
     * previousWorldFromOptical so the cloud lines up with the pose that was
     * true when those points were observed.
     */
    publishPointCloud(stamp_in,
                      correlatedPoints_in,
                      inlierIndices_in,
                      previousWorldFromOptical);
}

} /* namespace localisation::visual_odometry */
