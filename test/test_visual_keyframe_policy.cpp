/**
 * @file            test_visual_keyframe_policy.cpp
 *
 * @brief           Tests visual dropout and keyframe-retention policy.
 */

#include <gtest/gtest.h>

#include "visual_odometry_node/objects/VisualOdometryNode.h"

#include <cmath>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace
{

using Node = localisation::visual_odometry::VisualOdometryNode;

TEST(VisualKeyframePolicy, RetainsAcceptedKeyframeAfterRecoverableFailure)
{
    EXPECT_EQ(Node::selectKeyframeAction(false, true, 0.2, 0.75),
              Node::KeyframeAction::RETAIN_KEYFRAME);
}

TEST(VisualKeyframePolicy, AdvancesKeyframeAfterAcceptedEstimate)
{
    EXPECT_EQ(Node::selectKeyframeAction(true, true, 0.2, 0.75),
              Node::KeyframeAction::ADVANCE_KEYFRAME);
}

TEST(VisualKeyframePolicy, MarksPoseUnavailableAfterUnrecoverableGap)
{
    EXPECT_EQ(Node::selectKeyframeAction(false, true, 0.8, 0.75),
              Node::KeyframeAction::MARK_POSE_UNAVAILABLE);
}

TEST(VisualKeyframePolicy, AdvancesAfterSuccessfulLongBaselineEstimate)
{
    EXPECT_EQ(Node::selectKeyframeAction(true, true, 0.8, 0.75),
              Node::KeyframeAction::ADVANCE_KEYFRAME);
}

TEST(VisualKeyframePolicy, AdvancesDiagnosticsWhilePoseUnavailable)
{
    EXPECT_EQ(Node::selectKeyframeAction(false, false, 0.2, 0.75),
              Node::KeyframeAction::ADVANCE_KEYFRAME);
}

TEST(VisualGeometry, CameraOpticalAxesMapIntoBodyConvention)
{
    const cv::Matx44d bodyFromOptical =
        Node::calculateBodyFromOptical(0.932, 0.60, 0.0);

    EXPECT_DOUBLE_EQ(bodyFromOptical(0, 2), 1.0);
    EXPECT_DOUBLE_EQ(bodyFromOptical(1, 0), -1.0);
    EXPECT_DOUBLE_EQ(bodyFromOptical(2, 1), -1.0);
    EXPECT_DOUBLE_EQ(bodyFromOptical(0, 3), 0.932);
    EXPECT_DOUBLE_EQ(bodyFromOptical(2, 3), 0.60);

    const double      pitchRad = 10.0 * std::acos(-1.0) / 180.0;
    const cv::Matx44d pitched =
        Node::calculateBodyFromOptical(0.0, 0.0, pitchRad);
    EXPECT_NEAR(pitched(0, 2), std::cos(pitchRad), 1.0e-12);
    EXPECT_NEAR(pitched(2, 2), -std::sin(pitchRad), 1.0e-12);
}

TEST(VisualGeometry, LowParallaxClusterCannotReportPreciseTranslation)
{
    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 800.0,
                                  0.0,
                                  512.0,
                                  0.0,
                                  800.0,
                                  512.0,
                                  0.0,
                                  0.0,
                                  1.0);
    const cv::Mat rotationVector =
        (cv::Mat_<double>(3, 1) << 0.01, -0.02, 0.005);
    const cv::Mat translationVector =
        (cv::Mat_<double>(3, 1) << 0.05, -0.01, 0.10);
    std::vector<cv::Point3f> wellDistributedPoints;
    std::vector<cv::Point3f> distantClusteredPoints;
    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            wellDistributedPoints.emplace_back(
                static_cast<float>((column - 2.5) * 0.65),
                static_cast<float>((row - 2.5) * 0.45),
                static_cast<float>(4.0 + 0.4 * row + 0.2 * column));
            distantClusteredPoints.emplace_back(
                static_cast<float>((column - 2.5) * 0.05),
                static_cast<float>((row - 2.5) * 0.04),
                static_cast<float>(22.0 + 0.1 * row + 0.05 * column));
        }
    }

    std::vector<cv::Point2f> wellDistributedPixels;
    std::vector<cv::Point2f> distantClusteredPixels;
    cv::projectPoints(wellDistributedPoints,
                      rotationVector,
                      translationVector,
                      cameraMatrix,
                      cv::noArray(),
                      wellDistributedPixels);
    cv::projectPoints(distantClusteredPoints,
                      rotationVector,
                      translationVector,
                      cameraMatrix,
                      cv::noArray(),
                      distantClusteredPixels);
    for (std::size_t index = 0U; index < wellDistributedPixels.size(); ++index)
    {
        const float deterministicResidual = index % 2U == 0U ? 0.15F : -0.15F;
        wellDistributedPixels[index].x += deterministicResidual;
        distantClusteredPixels[index].x += deterministicResidual;
    }
    std::vector<int> inliers(wellDistributedPoints.size());
    std::iota(inliers.begin(), inliers.end(), 0);

    Node::VisualQualityConfiguration configuration;
    configuration.maximumNormalCondition = 1.0e16;
    const Node::VisualPoseQuality wellDistributed =
        Node::calculateVisualPoseQuality(wellDistributedPoints,
                                         wellDistributedPixels,
                                         inliers,
                                         rotationVector,
                                         translationVector,
                                         cameraMatrix,
                                         configuration);
    const Node::VisualPoseQuality distantClustered =
        Node::calculateVisualPoseQuality(distantClusteredPoints,
                                         distantClusteredPixels,
                                         inliers,
                                         rotationVector,
                                         translationVector,
                                         cameraMatrix,
                                         configuration);
    std::vector<cv::Point2f> highResidualPixels = wellDistributedPixels;
    for (std::size_t index = 0U; index < highResidualPixels.size(); ++index)
    {
        highResidualPixels[index].y += index % 2U == 0U ? 1.0F : -1.0F;
    }
    const Node::VisualPoseQuality highResidual =
        Node::calculateVisualPoseQuality(wellDistributedPoints,
                                         highResidualPixels,
                                         inliers,
                                         rotationVector,
                                         translationVector,
                                         cameraMatrix,
                                         configuration);

    ASSERT_TRUE(wellDistributed.isValid);
    EXPECT_LT(distantClustered.occupancyRatio, wellDistributed.occupancyRatio);
    EXPECT_LT(distantClustered.disparityPercentilesPx[1],
              wellDistributed.disparityPercentilesPx[1]);
    ASSERT_TRUE(highResidual.isValid);
    EXPECT_GT(highResidual.reprojectionRmsPx,
              wellDistributed.reprojectionRmsPx);
    EXPECT_GT(highResidual.relativeCovariance(0, 0),
              wellDistributed.relativeCovariance(0, 0));
    if (distantClustered.isValid)
    {
        const double wellDistributedTranslationVariance =
            wellDistributed.relativeCovariance(0, 0) +
            wellDistributed.relativeCovariance(1, 1) +
            wellDistributed.relativeCovariance(2, 2);
        const double distantTranslationVariance =
            distantClustered.relativeCovariance(0, 0) +
            distantClustered.relativeCovariance(1, 1) +
            distantClustered.relativeCovariance(2, 2);
        EXPECT_GT(distantTranslationVariance,
                  wellDistributedTranslationVariance);
    }
}

TEST(VisualGeometry, StereoDepthUncertaintyFollowsOpticalDepthAxis)
{
    const cv::Mat cameraMatrix   = (cv::Mat_<double>(3, 3) << 800.0,
                                  0.0,
                                  512.0,
                                  0.0,
                                  800.0,
                                  512.0,
                                  0.0,
                                  0.0,
                                  1.0);
    const cv::Mat rotationVector = (cv::Mat_<double>(3, 1) << 0.0, 0.25, 0.0);
    const cv::Mat translationVector =
        (cv::Mat_<double>(3, 1) << 0.02, -0.01, 0.08);
    std::vector<cv::Point3f> objectPoints;
    for (int row = 0; row < 6; ++row)
    {
        for (int column = 0; column < 6; ++column)
        {
            objectPoints.emplace_back(
                static_cast<float>((column - 2.5) * 0.45),
                static_cast<float>((row - 2.5) * 0.35),
                static_cast<float>(5.0 + 0.2 * row + 0.1 * column));
        }
    }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints,
                      rotationVector,
                      translationVector,
                      cameraMatrix,
                      cv::noArray(),
                      imagePoints);
    for (std::size_t index = 0U; index < imagePoints.size(); ++index)
    {
        imagePoints[index].x += index % 2U == 0U ? 0.1F : -0.1F;
    }
    std::vector<int> inliers(objectPoints.size());
    std::iota(inliers.begin(), inliers.end(), 0);

    Node::VisualQualityConfiguration lowDisparityNoiseConfiguration;
    lowDisparityNoiseConfiguration.maximumNormalCondition       = 1.0e16;
    lowDisparityNoiseConfiguration.minimumTranslationVarianceM2 = 1.0e-12;
    lowDisparityNoiseConfiguration.disparityStddevPx            = 0.25;
    Node::VisualQualityConfiguration highDisparityNoiseConfiguration =
        lowDisparityNoiseConfiguration;
    highDisparityNoiseConfiguration.disparityStddevPx = 2.0;

    const Node::VisualPoseQuality lowDisparityNoise =
        Node::calculateVisualPoseQuality(objectPoints,
                                         imagePoints,
                                         inliers,
                                         rotationVector,
                                         translationVector,
                                         cameraMatrix,
                                         lowDisparityNoiseConfiguration);
    const Node::VisualPoseQuality highDisparityNoise =
        Node::calculateVisualPoseQuality(objectPoints,
                                         imagePoints,
                                         inliers,
                                         rotationVector,
                                         translationVector,
                                         cameraMatrix,
                                         highDisparityNoiseConfiguration);

    ASSERT_TRUE(lowDisparityNoise.isValid);
    ASSERT_TRUE(highDisparityNoise.isValid);
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);
    const cv::Vec3d      depthAxisCurrent(rotationMatrix.at<double>(0, 2),
                                     rotationMatrix.at<double>(1, 2),
                                     rotationMatrix.at<double>(2, 2));
    Node::PoseCovariance disparityCovarianceDifference =
        highDisparityNoise.relativeCovariance -
        lowDisparityNoise.relativeCovariance;
    const double disparityVarianceDifference =
        disparityCovarianceDifference(0, 0) +
        disparityCovarianceDifference(1, 1) +
        disparityCovarianceDifference(2, 2);
    ASSERT_GT(disparityVarianceDifference, 0.0);
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            EXPECT_NEAR(disparityCovarianceDifference(row, column),
                        disparityVarianceDifference * depthAxisCurrent[row] *
                            depthAxisCurrent[column],
                        1.0e-10);
        }
    }
}

} /* namespace */
