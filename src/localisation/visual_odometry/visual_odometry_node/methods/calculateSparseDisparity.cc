/*!
 * @file            calculateSparseDisparity.cc
 *
 * @brief           Implements sub-pixel stereo matching at keyframe corners.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "visual_odometry_node/objects/VisualOdometryNodeClass.h"

/* C++ Standard Library Includes */
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

/* External Library Includes */
#include <opencv2/imgproc.hpp>

namespace localisation::visual_odometry
{

namespace
{

/*!
 * @brief           A match must beat the best non-adjacent candidate's
 *                  squared-difference cost by at least this factor.
 */
constexpr double UNIQUENESS_RATIO = 0.8;

/*!
 * @brief           Result of searching one patch along one row.
 */
struct RowMatch
{
    /*!
     * @brief           True when an unambiguous, interior minimum exists.
     */
    bool isValid{false};

    /*!
     * @brief           Sub-pixel position of the minimum in cost columns.
     */
    double column{0.0};
};

/*!
 * @brief           Finds the unambiguous minimum of a one-row cost vector
 *                  and refines it with a parabola through its neighbours.
 *
 * @param[in]       cost_in
 *                  1-by-N CV_32F squared-difference costs.
 *
 * @return          The refined minimum, invalid at the range edges or when
 *                  a non-adjacent candidate is nearly as good.
 */
RowMatch findRowMinimum(const cv::Mat &cost_in)
{
    RowMatch  match;
    const int columnCount = cost_in.cols;
    if (columnCount < 3)
    {
        return match;
    }
    const float *p_cost     = cost_in.ptr<float>(0);
    int          bestColumn = 0;
    for (int column = 1; column < columnCount; column++)
    {
        if (p_cost[column] < p_cost[bestColumn])
        {
            bestColumn = column;
        }
    }

    /* A minimum on the edge may lie outside the searched range. */
    if (bestColumn == 0 || bestColumn == columnCount - 1)
    {
        return match;
    }

    /* Repetitive or flat texture matches almost equally well elsewhere. */
    float secondBest = std::numeric_limits<float>::infinity();
    for (int column = 0; column < columnCount; column++)
    {
        if (std::abs(column - bestColumn) > 1)
        {
            secondBest = std::min(secondBest, p_cost[column]);
        }
    }
    if (!(static_cast<double>(p_cost[bestColumn]) <
          UNIQUENESS_RATIO * static_cast<double>(secondBest)))
    {
        return match;
    }

    const double previous  = p_cost[bestColumn - 1];
    const double best      = p_cost[bestColumn];
    const double next      = p_cost[bestColumn + 1];
    const double curvature = previous - 2.0 * best + next;
    double       offset    = 0.0;
    if (curvature > 0.0)
    {
        offset = std::clamp(0.5 * (previous - next) / curvature, -0.5, 0.5);
    }
    match.isValid = true;
    match.column  = static_cast<double>(bestColumn) + offset;
    return match;
}

} /* anonymous namespace */

void VisualOdometryNode::calculateSparseDisparity(
    const cv::Mat                                                  &left_in,
    const cv::Mat                                                  &right_in,
    const std::array<feature_tracking::Point2D,
                     feature_tracking::MAXIMUM_SUPPORTED_FEATURES> &corners_in,
    std::size_t cornerCount_in,
    int         maximumDisparityPx_in,
    int         halfWindowPx_in,
    double      maximumLeftRightDifferencePx_in,
    std::array<float, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        &disparityPx_out)
{
    disparityPx_out.fill(std::numeric_limits<float>::quiet_NaN());
    const int         halfWindow = halfWindowPx_in;
    const cv::Size    patchSize(2 * halfWindow + 1, 2 * halfWindow + 1);
    const std::size_t cornerCount =
        std::min(cornerCount_in, feature_tracking::MAXIMUM_SUPPORTED_FEATURES);

    cv::Mat patch;
    cv::Mat strip;
    cv::Mat cost;
    for (std::size_t index = 0U; index < cornerCount; index++)
    {
        const float column = corners_in[index].x;
        const float row    = corners_in[index].y;

        /* Keep both patches and the whole search strip inside the images;
         * getRectSubPix would otherwise replicate border pixels into a
         * spurious match. */
        const int searchRange =
            std::min(maximumDisparityPx_in,
                     static_cast<int>(std::floor(column)) - halfWindow - 1);
        if (searchRange < 3 || row < static_cast<float>(halfWindow + 1) ||
            row > static_cast<float>(left_in.rows - halfWindow - 2) ||
            column > static_cast<float>(left_in.cols - halfWindow - 2))
        {
            continue;
        }

        /* Left to right: cost column i places the patch at right column
         * column - searchRange + i, i.e. disparity searchRange - i. */
        cv::getRectSubPix(left_in,
                          patchSize,
                          cv::Point2f(column, row),
                          patch,
                          CV_32F);
        cv::getRectSubPix(
            right_in,
            cv::Size(searchRange + 2 * halfWindow + 1, patchSize.height),
            cv::Point2f(column - 0.5F * static_cast<float>(searchRange), row),
            strip,
            CV_32F);
        cv::matchTemplate(strip, patch, cost, cv::TM_SQDIFF);
        const RowMatch forward = findRowMinimum(cost);
        if (!forward.isValid)
        {
            continue;
        }
        const double disparityPx =
            static_cast<double>(searchRange) - forward.column;

        /* Right to left: the matched right patch must find the corner
         * again; cost column j places it at left column
         * matchedColumn + j, i.e. disparity j. */
        const float matchedColumn = column - static_cast<float>(disparityPx);
        const int   backRange =
            std::min(maximumDisparityPx_in,
                     left_in.cols - halfWindow - 2 -
                         static_cast<int>(std::ceil(matchedColumn)));
        if (backRange < 3)
        {
            continue;
        }
        cv::getRectSubPix(right_in,
                          patchSize,
                          cv::Point2f(matchedColumn, row),
                          patch,
                          CV_32F);
        cv::getRectSubPix(
            left_in,
            cv::Size(backRange + 2 * halfWindow + 1, patchSize.height),
            cv::Point2f(matchedColumn + 0.5F * static_cast<float>(backRange),
                        row),
            strip,
            CV_32F);
        cv::matchTemplate(strip, patch, cost, cv::TM_SQDIFF);
        const RowMatch backward = findRowMinimum(cost);
        if (!backward.isValid || std::abs(backward.column - disparityPx) >
                                     maximumLeftRightDifferencePx_in)
        {
            continue;
        }
        disparityPx_out[index] = static_cast<float>(disparityPx);
    }
}

} /* namespace localisation::visual_odometry */
