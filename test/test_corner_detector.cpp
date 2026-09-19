/*!
 * @File:         test_corner_detector.cpp
 *
 * @Brief:        Unit tests for ShiTomasiCornerDetector.
 *
 * @Date:         16/09/2026
 *
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include "feature_tracking/objects/FeatureTrackingStatus.h"
#include "feature_tracking/objects/ImageView.h"
#include "feature_tracking/objects/Point2D.h"
#include "feature_tracking/objects/ShiTomasiCornerDetector.h"

namespace
{

using localisation::visual_odometry::feature_tracking::FeatureTrackingStatus;
using localisation::visual_odometry::feature_tracking::ImageView;
using localisation::visual_odometry::feature_tracking::Point2D;
using localisation::visual_odometry::feature_tracking::
    ShiTomasiCornerDetector;

/*!
 * @brief           Wraps a tightly-packed pixel buffer in an `ImageView`.
 */
ImageView makeView(const std::vector<std::uint8_t> &pixels_in, int width_in,
                   int height_in)
{
    ImageView view;
    view.p_pixels = pixels_in.data();
    view.width = width_in;
    view.height = height_in;
    view.strideBytes = static_cast<std::size_t>(width_in);
    return view;
}

/*!
 * @brief           Builds a uniform-intensity image.
 */
std::vector<std::uint8_t> makeUniformImage(int width_in, int height_in,
                                           std::uint8_t value_in)
{
    return std::vector<std::uint8_t>(
        static_cast<std::size_t>(width_in) *
            static_cast<std::size_t>(height_in),
        value_in);
}

/*!
 * @brief           Builds an image with a single unambiguous L-shaped
 *                  corner: a bright rectangle occupying every pixel at or
 *                  below/right of `(cornerRow_in, cornerColumn_in)` on a
 *                  dark background.
 */
std::vector<std::uint8_t> makeSingleCornerImage(int width_in, int height_in,
                                                int cornerRow_in,
                                                int cornerColumn_in)
{
    std::vector<std::uint8_t> pixels =
        makeUniformImage(width_in, height_in, 20U);

    for (int row = cornerRow_in; row < height_in; ++row)
    {
        for (int column = cornerColumn_in; column < width_in; ++column)
        {
            pixels[(static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(width_in)) +
                   static_cast<std::size_t>(column)] = 220U;
        }
    }

    return pixels;
}

/*!
 * @brief           Builds a checkerboard pattern with `squarePx_in`-sized
 *                  squares.
 */
std::vector<std::uint8_t> makeCheckerboardImage(int width_in, int height_in,
                                                int squarePx_in)
{
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>(width_in) *
        static_cast<std::size_t>(height_in));

    for (int row = 0; row < height_in; ++row)
    {
        for (int column = 0; column < width_in; ++column)
        {
            const bool isLightSquare =
                ((row / squarePx_in) + (column / squarePx_in)) % 2 == 0;
            pixels[(static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(width_in)) +
                   static_cast<std::size_t>(column)] =
                isLightSquare ? 220U : 20U;
        }
    }

    return pixels;
}

/*!
 * @brief           Squared Euclidean distance between two points.
 */
float distanceSquared(const Point2D &first_in, const Point2D &second_in)
{
    const float deltaX = first_in.x - second_in.x;
    const float deltaY = first_in.y - second_in.y;
    return (deltaX * deltaX) + (deltaY * deltaY);
}

} /* anonymous namespace */

TEST(ShiTomasiCornerDetectorTest, RequiresInitializeBeforeDetect)
{
    ShiTomasiCornerDetector detector;
    const std::vector<std::uint8_t> pixels = makeUniformImage(16, 16, 0U);
    const ImageView image = makeView(pixels, 16, 16);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 999U;

    EXPECT_EQ(
        detector.detect(image, features, count),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED);
    EXPECT_EQ(count, 0U);
}

TEST(ShiTomasiCornerDetectorTest, RejectsFeatureCountExceedingCompileTimeCap)
{
    ShiTomasiCornerDetector detector;

    EXPECT_EQ(
        detector.initialize(
            64, 64, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES + 1U,
            0.01F, 8.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
}

TEST(ShiTomasiCornerDetectorTest, RejectsNonPositiveQualityLevel)
{
    ShiTomasiCornerDetector detector;

    EXPECT_EQ(
        detector.initialize(64, 64, 100U, 0.0F, 8.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(
        detector.initialize(64, 64, 100U, -0.1F, 8.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(
        detector.initialize(64, 64, 100U, 1.5F, 8.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
}

TEST(ShiTomasiCornerDetectorTest, RejectsNonPositiveMinimumDistance)
{
    ShiTomasiCornerDetector detector;

    EXPECT_EQ(
        detector.initialize(64, 64, 100U, 0.01F, 0.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
    EXPECT_EQ(
        detector.initialize(64, 64, 100U, 0.01F, -1.0F),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION);
}

TEST(ShiTomasiCornerDetectorTest, RejectsImageDimensionMismatch)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(64, 64, 100U, 0.01F, 8.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels = makeUniformImage(32, 32, 0U);
    const ImageView image = makeView(pixels, 32, 32);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    EXPECT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT);
}

TEST(ShiTomasiCornerDetectorTest, DetectsSingleSyntheticCornerAtKnownPixel)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(64, 64, 10U, 0.01F, 4.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels =
        makeSingleCornerImage(64, 64, 32, 32);
    const ImageView image = makeView(pixels, 64, 64);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    ASSERT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    ASSERT_GT(count, 0U);

    const Point2D expectedCorner{32.0F, 32.0F};
    bool foundNearExpectedCorner = false;
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (distanceSquared(features[index], expectedCorner) <= 4.0F)
        {
            foundNearExpectedCorner = true;
            break;
        }
    }
    EXPECT_TRUE(foundNearExpectedCorner);
}

TEST(ShiTomasiCornerDetectorTest, CountsAllCornersOnAKnownCheckerboardPattern)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(64, 64, 100U, 0.01F, 3.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    constexpr int squarePx = 8;
    const std::vector<std::uint8_t> pixels =
        makeCheckerboardImage(64, 64, squarePx);
    const ImageView image = makeView(pixels, 64, 64);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    ASSERT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    /* Every interior grid intersection should have a detected feature
     * nearby; boundary intersections are excluded, matching the
     * detector's own documented border-exclusion policy. */
    for (int gridRow = 1; gridRow < 64 / squarePx; ++gridRow)
    {
        for (int gridColumn = 1; gridColumn < 64 / squarePx; ++gridColumn)
        {
            const Point2D expectedIntersection{
                static_cast<float>(gridColumn * squarePx),
                static_cast<float>(gridRow * squarePx)};

            bool foundNearIntersection = false;
            for (std::size_t index = 0U; index < count; ++index)
            {
                if (distanceSquared(features[index], expectedIntersection) <=
                    4.0F)
                {
                    foundNearIntersection = true;
                    break;
                }
            }
            EXPECT_TRUE(foundNearIntersection)
                << "no detected feature near grid intersection ("
                << expectedIntersection.x << ", " << expectedIntersection.y
                << ")";
        }
    }
}

TEST(ShiTomasiCornerDetectorTest, ReturnsZeroFeaturesOnAUniformImage)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(32, 32, 50U, 0.01F, 4.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels = makeUniformImage(32, 32, 128U);
    const ImageView image = makeView(pixels, 32, 32);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 999U;

    EXPECT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_EQ(count, 0U);
}

TEST(ShiTomasiCornerDetectorTest, CapsAcceptedFeaturesAtConfiguredMaximum)
{
    ShiTomasiCornerDetector detector;
    constexpr std::size_t maximumFeatures = 10U;
    ASSERT_EQ(detector.initialize(64, 64, maximumFeatures, 0.01F, 1.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    /* A fine checkerboard has far more than 10 strong interior corners,
     * with a very permissive minimum distance so the configured maximum
     * is the only thing limiting the count. */
    const std::vector<std::uint8_t> pixels = makeCheckerboardImage(64, 64, 4);
    const ImageView image = makeView(pixels, 64, 64);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    ASSERT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_EQ(count, maximumFeatures);
}

TEST(ShiTomasiCornerDetectorTest,
    EnforcesConfiguredMinimumDistanceBetweenAcceptedFeatures)
{
    ShiTomasiCornerDetector detector;
    constexpr float minimumDistancePx = 6.0F;
    ASSERT_EQ(detector.initialize(64, 64, 100U, 0.01F, minimumDistancePx),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    /* Adjacent grid intersections are only 3px apart, well inside the
     * configured 6px minimum, so the selection step must actively
     * suppress most of them. */
    const std::vector<std::uint8_t> pixels = makeCheckerboardImage(64, 64, 3);
    const ImageView image = makeView(pixels, 64, 64);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    ASSERT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    ASSERT_GT(count, 1U);

    const float minimumDistanceSquared = minimumDistancePx * minimumDistancePx;
    for (std::size_t first = 0U; first < count; ++first)
    {
        for (std::size_t second = first + 1U; second < count; ++second)
        {
            EXPECT_GE(distanceSquared(features[first], features[second]),
                     minimumDistanceSquared);
        }
    }
}

TEST(ShiTomasiCornerDetectorTest, IsDeterministicAcrossRepeatedCalls)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(64, 64, 50U, 0.01F, 4.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels = makeCheckerboardImage(64, 64, 8);
    const ImageView image = makeView(pixels, 64, 64);

    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        firstFeatures{};
    std::size_t firstCount = 0U;
    ASSERT_EQ(detector.detect(image, firstFeatures, firstCount),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    for (int repeat = 0; repeat < 5; ++repeat)
    {
        std::array<Point2D,
                   ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
            repeatFeatures{};
        std::size_t repeatCount = 0U;
        ASSERT_EQ(detector.detect(image, repeatFeatures, repeatCount),
                 FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

        ASSERT_EQ(repeatCount, firstCount);
        for (std::size_t index = 0U; index < firstCount; ++index)
        {
            EXPECT_FLOAT_EQ(repeatFeatures[index].x, firstFeatures[index].x);
            EXPECT_FLOAT_EQ(repeatFeatures[index].y, firstFeatures[index].y);
        }
    }
}

TEST(ShiTomasiCornerDetectorTest, ExcludesCornerAtTheExtremeImageBorder)
{
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(32, 32, 50U, 0.01F, 2.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    /* A checkerboard starting exactly at pixel (0, 0) has corner-like
     * structure sitting right at the border on every side. */
    const std::vector<std::uint8_t> pixels = makeCheckerboardImage(32, 32, 4);
    const ImageView image = makeView(pixels, 32, 32);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    ASSERT_EQ(detector.detect(image, features, count),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    for (std::size_t index = 0U; index < count; ++index)
    {
        EXPECT_GT(features[index].x, 0.0F);
        EXPECT_LT(features[index].x, 31.0F);
        EXPECT_GT(features[index].y, 0.0F);
        EXPECT_LT(features[index].y, 31.0F);
    }
}

TEST(ShiTomasiCornerDetectorTest,
    CompletesDetectionWithinIndicativeWallClockBudget)
{
    /* This bounds wall-clock time on whatever machine runs the test; it
     * is indicative only and is NOT a certified worst-case-execution-time
     * analysis (see
     * docs/compliance/feature_tracking/VERIFICATION_REPORT.md). */
    ShiTomasiCornerDetector detector;
    ASSERT_EQ(detector.initialize(1024, 1024, 500U, 0.01F, 8.0F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels =
        makeCheckerboardImage(1024, 1024, 16);
    const ImageView image = makeView(pixels, 1024, 1024);
    std::array<Point2D, ShiTomasiCornerDetector::MAXIMUM_SUPPORTED_FEATURES>
        features{};
    std::size_t count = 0U;

    const auto start = std::chrono::steady_clock::now();
    const auto status = detector.detect(image, features, count);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(status, FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                 .count(),
             2000);
}

TEST(ShiTomasiCornerDetectorTest,
    ClosedFormMinEigenvalueMatchesDirectComputation)
{
    /* Pure-math check, independent of the class: for
     * M = [[4,1],[1,2]], the characteristic polynomial
     * (4-l)(2-l)-1=0 gives l^2-6l+7=0, i.e. l = 3 +/- sqrt(2), so
     * lambda_min = 3 - sqrt(2). */
    const double sxx = 4.0;
    const double syy = 2.0;
    const double sxy = 1.0;

    const double trace = sxx + syy;
    const double difference = sxx - syy;
    const double discriminant =
        std::sqrt((difference * difference) + (4.0 * sxy * sxy));
    const double minimumEigenvalue = 0.5 * (trace - discriminant);

    const double expected = 3.0 - std::sqrt(2.0);
    EXPECT_NEAR(minimumEigenvalue, expected, 1.0e-9);
}
