/**
 * @file            calculateVisualPoseQuality.cc
 *
 * @brief           Implements geometry-aware PnP quality and covariance.
 */

#include "visual_odometry_node/objects/VisualOdometryNode.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace localisation::visual_odometry
{

namespace
{

double percentile(std::vector<double> values_in, double quantile_in)
{
    if (values_in.empty())
    {
        return 0.0;
    }
    std::sort(values_in.begin(), values_in.end());
    const double position =
        quantile_in * static_cast<double>(values_in.size() - 1U);
    const std::size_t lower    = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper    = static_cast<std::size_t>(std::ceil(position));
    const double      fraction = position - static_cast<double>(lower);
    return values_in[lower] * (1.0 - fraction) + values_in[upper] * fraction;
}

} /* anonymous namespace */

VisualOdometryNode::VisualPoseQuality
    VisualOdometryNode::calculateVisualPoseQuality(
        const std::vector<cv::Point3f>   &objectPoints_in,
        const std::vector<cv::Point2f>   &imagePoints_in,
        const std::vector<int>           &inlierIndices_in,
        const cv::Mat                    &rotationVector_in,
        const cv::Mat                    &translationVector_in,
        const cv::Mat                    &cameraMatrix_in,
        const VisualQualityConfiguration &configuration_in)
{
    VisualPoseQuality quality;
    if (objectPoints_in.size() != imagePoints_in.size() ||
        inlierIndices_in.size() < 4U || cameraMatrix_in.rows != 3 ||
        cameraMatrix_in.cols != 3 || configuration_in.imageWidthPx <= 0 ||
        configuration_in.imageHeightPx <= 0 ||
        configuration_in.occupancyGridRows <= 0 ||
        configuration_in.occupancyGridColumns <= 0 ||
        !(configuration_in.fxPx > 0.0) || !(configuration_in.baselineM > 0.0))
    {
        return quality;
    }

    std::vector<cv::Point3f> inlierObjectPoints;
    std::vector<cv::Point2f> inlierImagePoints;
    std::vector<double>      depthsM;
    std::vector<double>      disparitiesPx;
    inlierObjectPoints.reserve(inlierIndices_in.size());
    inlierImagePoints.reserve(inlierIndices_in.size());
    depthsM.reserve(inlierIndices_in.size());
    disparitiesPx.reserve(inlierIndices_in.size());

    const int cellCount = configuration_in.occupancyGridRows *
                          configuration_in.occupancyGridColumns;
    std::vector<bool> occupiedCells(static_cast<std::size_t>(cellCount), false);
    std::size_t       nearMidCount = 0U;
    for (const int index : inlierIndices_in)
    {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= objectPoints_in.size())
        {
            return quality;
        }
        const cv::Point3f &point3d =
            objectPoints_in[static_cast<std::size_t>(index)];
        const cv::Point2f &point2d =
            imagePoints_in[static_cast<std::size_t>(index)];
        if (!std::isfinite(point3d.x) || !std::isfinite(point3d.y) ||
            !std::isfinite(point3d.z) || !(point3d.z > 0.0F) ||
            !std::isfinite(point2d.x) || !std::isfinite(point2d.y))
        {
            return quality;
        }

        inlierObjectPoints.push_back(point3d);
        inlierImagePoints.push_back(point2d);
        const double depthM = static_cast<double>(point3d.z);
        depthsM.push_back(depthM);
        disparitiesPx.push_back(configuration_in.fxPx *
                                configuration_in.baselineM / depthM);
        if (depthM <= configuration_in.nearMidDepthM)
        {
            ++nearMidCount;
        }

        const int column = std::clamp(
            static_cast<int>(
                point2d.x *
                static_cast<float>(configuration_in.occupancyGridColumns) /
                static_cast<float>(configuration_in.imageWidthPx)),
            0,
            configuration_in.occupancyGridColumns - 1);
        const int row = std::clamp(
            static_cast<int>(
                point2d.y *
                static_cast<float>(configuration_in.occupancyGridRows) /
                static_cast<float>(configuration_in.imageHeightPx)),
            0,
            configuration_in.occupancyGridRows - 1);
        occupiedCells[static_cast<std::size_t>(
            row * configuration_in.occupancyGridColumns + column)] = true;
    }

    quality.occupancyRatio =
        static_cast<double>(
            std::count(occupiedCells.begin(), occupiedCells.end(), true)) /
        static_cast<double>(cellCount);
    quality.nearMidRatio = static_cast<double>(nearMidCount) /
                           static_cast<double>(inlierIndices_in.size());
    quality.depthPercentilesM      = {percentile(depthsM, 0.10),
                                      percentile(depthsM, 0.50),
                                      percentile(depthsM, 0.90)};
    quality.disparityPercentilesPx = {percentile(disparitiesPx, 0.10),
                                      percentile(disparitiesPx, 0.50),
                                      percentile(disparitiesPx, 0.90)};

    cv::Mat                  jacobian;
    cv::Mat                  rotationMatrix;
    std::vector<cv::Point2f> projectedPoints;
    try
    {
        cv::Mat rotationMatrixNative;
        cv::Rodrigues(rotationVector_in, rotationMatrixNative);
        rotationMatrixNative.convertTo(rotationMatrix, CV_64F);
        cv::projectPoints(inlierObjectPoints,
                          rotationVector_in,
                          translationVector_in,
                          cameraMatrix_in,
                          cv::noArray(),
                          projectedPoints,
                          jacobian);
    }
    catch (const cv::Exception &)
    {
        return quality;
    }
    if (jacobian.rows != static_cast<int>(2U * inlierObjectPoints.size()) ||
        jacobian.cols < 6)
    {
        return quality;
    }

    double squaredErrorPx2 = 0.0;
    for (std::size_t index = 0U; index < projectedPoints.size(); ++index)
    {
        const cv::Point2f residual =
            inlierImagePoints[index] - projectedPoints[index];
        squaredErrorPx2 += static_cast<double>(residual.dot(residual));
    }
    quality.reprojectionRmsPx = std::sqrt(
        squaredErrorPx2 / static_cast<double>(projectedPoints.size()));

    const cv::Mat poseJacobian = jacobian.colRange(0, 6);
    const cv::Mat normalMatrix = poseJacobian.t() * poseJacobian;
    cv::Mat       singularValues;
    cv::SVD::compute(normalMatrix, singularValues);
    if (singularValues.rows < 6)
    {
        return quality;
    }
    const double maximumSingularValue = singularValues.at<double>(0);
    const double minimumSingularValue = singularValues.at<double>(5);
    if (!(maximumSingularValue > 0.0) ||
        !(minimumSingularValue >
          maximumSingularValue * std::numeric_limits<double>::epsilon()))
    {
        return quality;
    }
    quality.normalCondition = maximumSingularValue / minimumSingularValue;
    if (!std::isfinite(quality.normalCondition) ||
        quality.normalCondition > configuration_in.maximumNormalCondition)
    {
        return quality;
    }

    cv::Mat inverseNormal;
    if (cv::invert(normalMatrix, inverseNormal, cv::DECOMP_SVD) == 0.0)
    {
        return quality;
    }
    const double degreesOfFreedom =
        std::max(1.0,
                 static_cast<double>(2U * inlierObjectPoints.size()) - 6.0);
    const double pixelVariance =
        std::max(configuration_in.reprojectionNoiseFloorPx *
                     configuration_in.reprojectionNoiseFloorPx,
                 squaredErrorPx2 / degreesOfFreedom);
    const double coverageScale =
        std::max(1.0,
                 configuration_in.minimumCoverageRatio /
                     std::max(quality.occupancyRatio,
                              1.0 / static_cast<double>(cellCount)));
    const double nearMidScale = std::max(
        1.0,
        configuration_in.minimumNearMidRatio /
            std::max(quality.nearMidRatio,
                     1.0 / static_cast<double>(inlierIndices_in.size())));
    const double disparityScale =
        std::max(1.0,
                 configuration_in.preferredDisparityPx /
                     std::max(quality.disparityPercentilesPx[1], 1.0e-6));
    const double translationInflation = coverageScale * coverageScale *
                                        nearMidScale * nearMidScale *
                                        disparityScale * disparityScale;

    constexpr std::array<int, 6> PARAMETER_ORDER{3, 4, 5, 0, 1, 2};
    for (int row = 0; row < 6; ++row)
    {
        const double rowScale = row < 3 ? std::sqrt(translationInflation) : 1.0;
        for (int column = 0; column < 6; ++column)
        {
            const double columnScale =
                column < 3 ? std::sqrt(translationInflation) : 1.0;
            quality.relativeCovariance(row, column) =
                pixelVariance * rowScale * columnScale *
                inverseNormal.at<double>(
                    PARAMETER_ORDER[static_cast<std::size_t>(row)],
                    PARAMETER_ORDER[static_cast<std::size_t>(column)]);
        }
    }

    /* Stereo depth z = f*b/d has first-order uncertainty
     * sigma_z = f*b*sigma_d/d^2. PnP's image Jacobian treats reconstructed
     * object points as exact, so add the standard error of the median-depth
     * support explicitly rather than allowing distant points to report
     * unrealistically precise translation. The points were reconstructed in
     * the previous optical frame while PnP translation is expressed in the
     * current optical frame, so R_previous_to_current maps that depth axis. */
    const double medianDisparityPx  = quality.disparityPercentilesPx[1];
    const double medianDepthStddevM = configuration_in.fxPx *
                                      configuration_in.baselineM *
                                      configuration_in.disparityStddevPx /
                                      (medianDisparityPx * medianDisparityPx);
    const double stereoTranslationVariance =
        medianDepthStddevM * medianDepthStddevM * coverageScale *
        coverageScale * nearMidScale * nearMidScale /
        static_cast<double>(inlierIndices_in.size());
    const cv::Vec3d depthAxisCurrent(rotationMatrix.at<double>(0, 2),
                                     rotationMatrix.at<double>(1, 2),
                                     rotationMatrix.at<double>(2, 2));
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            quality.relativeCovariance(row, column) +=
                stereoTranslationVariance * depthAxisCurrent[row] *
                depthAxisCurrent[column];
        }
    }

    for (int index = 0; index < 6; ++index)
    {
        const double minimumVariance =
            index < 3 ? configuration_in.minimumTranslationVarianceM2
                      : configuration_in.minimumRotationVarianceRad2;
        const double maximumVariance =
            index < 3 ? configuration_in.maximumTranslationVarianceM2
                      : configuration_in.maximumRotationVarianceRad2;
        quality.relativeCovariance(index, index) =
            std::max(quality.relativeCovariance(index, index), minimumVariance);
        if (!std::isfinite(quality.relativeCovariance(index, index)) ||
            quality.relativeCovariance(index, index) > maximumVariance)
        {
            return quality;
        }
    }

    quality.relativeCovariance =
        0.5 * (quality.relativeCovariance + quality.relativeCovariance.t());
    quality.isValid = true;
    return quality;
}

} /* namespace localisation::visual_odometry */
