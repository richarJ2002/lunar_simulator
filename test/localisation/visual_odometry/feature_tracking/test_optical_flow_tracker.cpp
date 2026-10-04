/*!
 * @File:         test_optical_flow_tracker.cpp
 *
 * @Brief:        Unit tests for PyramidalLucasKanadeTracker.
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
#include "feature_tracking/objects/PyramidalLucasKanadeTracker.h"

namespace
{

using localisation::visual_odometry::feature_tracking::FeatureTrackingStatus;
using localisation::visual_odometry::feature_tracking::ImageView;
using localisation::visual_odometry::feature_tracking::Point2D;
using localisation::visual_odometry::feature_tracking::
    PyramidalLucasKanadeTracker;

constexpr double PI = 3.14159265358979323846;

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
 * @brief           An analytic, richly-textured intensity field, sampled
 *                  by `makeTexturedImage()` to build ground-truth
 *                  translated image pairs without any discretization bias
 *                  between them.
 */
float analyticPattern(float x_in, float y_in)
{
    return 128.0F +
          (60.0F * std::sin(2.0F * static_cast<float>(PI) * x_in / 17.0F)) +
          (60.0F * std::sin(2.0F * static_cast<float>(PI) * y_in / 23.0F));
}

/*!
 * @brief           Rasterizes `analyticPattern()`, offset by
 *                  `(offsetX_in, offsetY_in)`, into an image.
 *
 * Two images built from this same analytic field at different offsets
 * form an exact ground-truth translated pair: content at
 * `(x, y)` in the offset-`(0,0)` image appears at
 * `(x + offsetX_in, y + offsetY_in)` in the offset image.
 */
std::vector<std::uint8_t> makeTexturedImage(int width_in, int height_in,
                                            float offsetX_in,
                                            float offsetY_in)
{
    std::vector<std::uint8_t> pixels(
        static_cast<std::size_t>(width_in) *
        static_cast<std::size_t>(height_in));

    for (int row = 0; row < height_in; ++row)
    {
        for (int column = 0; column < width_in; ++column)
        {
            const float value = analyticPattern(
                static_cast<float>(column) - offsetX_in,
                static_cast<float>(row) - offsetY_in);
            const float clamped =
                std::max(0.0F, std::min(255.0F, std::round(value)));
            pixels[(static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(width_in)) +
                   static_cast<std::size_t>(column)] =
                static_cast<std::uint8_t>(clamped);
        }
    }

    return pixels;
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

} /* anonymous namespace */

TEST(PyramidalLucasKanadeTrackerTest, RequiresInitializeBeforeTrack)
{
    PyramidalLucasKanadeTracker tracker;
    const std::vector<std::uint8_t> pixels = makeUniformImage(64, 64, 0U);
    const ImageView image = makeView(pixels, 64, 64);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{32.0F, 32.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    EXPECT_EQ(
        tracker.track(image, image, previousFeatures, 1U, currentFeatures,
                      status, error),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED);
    EXPECT_EQ(status[0], 0U);
}

TEST(PyramidalLucasKanadeTrackerTest, RejectsImageDimensionMismatch)
{
    PyramidalLucasKanadeTracker tracker;
    ASSERT_EQ(tracker.initialize(64, 64, 1, 9, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> previousPixels =
        makeUniformImage(64, 64, 0U);
    const std::vector<std::uint8_t> currentPixels =
        makeUniformImage(32, 32, 0U);
    const ImageView previousImage = makeView(previousPixels, 64, 64);
    const ImageView currentImage = makeView(currentPixels, 32, 32);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{32.0F, 32.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    EXPECT_EQ(
        tracker.track(previousImage, currentImage, previousFeatures, 1U,
                      currentFeatures, status, error),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT);
}

TEST(PyramidalLucasKanadeTrackerTest,
    TracksPureTranslationWithKnownGroundTruthDisplacement)
{
    PyramidalLucasKanadeTracker tracker;
    /* Coarsest level (256 -> 128 -> 64 -> 32) must be >= windowSizePx
     * (21); a smaller image at this pyramid depth would leave no
     * in-bounds window anywhere, even at the image center. */
    ASSERT_EQ(tracker.initialize(256, 256, 3, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    constexpr float groundTruthDx = 3.0F;
    constexpr float groundTruthDy = -2.0F;
    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(256, 256, 0.0F, 0.0F);
    const std::vector<std::uint8_t> currentPixels =
        makeTexturedImage(256, 256, groundTruthDx, groundTruthDy);
    const ImageView previousImage = makeView(previousPixels, 256, 256);
    const ImageView currentImage = makeView(currentPixels, 256, 256);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{128.0F, 128.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    ASSERT_EQ(tracker.track(previousImage, currentImage, previousFeatures,
                            1U, currentFeatures, status, error),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    ASSERT_EQ(status[0], 1U);

    EXPECT_NEAR(currentFeatures[0].x, 128.0F + groundTruthDx, 0.1F);
    EXPECT_NEAR(currentFeatures[0].y, 128.0F + groundTruthDy, 0.1F);
}

TEST(PyramidalLucasKanadeTrackerTest, MarksLowTextureWindowAsLost)
{
    PyramidalLucasKanadeTracker tracker;
    /* Coarsest level (64 -> 32) must be >= windowSizePx (21). */
    ASSERT_EQ(tracker.initialize(64, 64, 1, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels = makeUniformImage(64, 64, 128U);
    const ImageView image = makeView(pixels, 64, 64);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{32.0F, 32.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    ASSERT_EQ(
        tracker.track(image, image, previousFeatures, 1U, currentFeatures,
                      status, error),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_EQ(status[0], 0U);
}

TEST(PyramidalLucasKanadeTrackerTest, MarksOutOfBoundsFeatureAsLost)
{
    PyramidalLucasKanadeTracker tracker;
    /* Coarsest level (64 -> 32) must be >= windowSizePx (21). */
    ASSERT_EQ(tracker.initialize(64, 64, 1, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> pixels =
        makeTexturedImage(64, 64, 0.0F, 0.0F);
    const ImageView image = makeView(pixels, 64, 64);

    /* A window half-width of 10 (windowSizePx=21) cannot possibly fit
     * around a feature at pixel (1, 1) without leaving the image. */
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{1.0F, 1.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    ASSERT_EQ(
        tracker.track(image, image, previousFeatures, 1U, currentFeatures,
                      status, error),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_EQ(status[0], 0U);
}

TEST(PyramidalLucasKanadeTrackerTest,
    ProducesLargerErrorForAMismatchedThanAnAccurateTrack)
{
    PyramidalLucasKanadeTracker tracker;
    /* Coarsest level (256 -> 128 -> 64 -> 32) must be >= windowSizePx
     * (21). */
    ASSERT_EQ(tracker.initialize(256, 256, 3, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    /* Accurate case: a genuine small translation of the same pattern. */
    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(256, 256, 0.0F, 0.0F);
    const std::vector<std::uint8_t> accurateCurrentPixels =
        makeTexturedImage(256, 256, 2.0F, 1.0F);
    const ImageView previousImage = makeView(previousPixels, 256, 256);
    const ImageView accurateCurrentImage =
        makeView(accurateCurrentPixels, 256, 256);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{128.0F, 128.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        accurateError{};

    ASSERT_EQ(tracker.track(previousImage, accurateCurrentImage,
                            previousFeatures, 1U, currentFeatures, status,
                            accurateError),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    ASSERT_EQ(status[0], 1U);

    /* Mismatched case: a "current" image built from a differently-phased
     * pattern, so no genuine match exists anywhere nearby -- gradient
     * descent still converges to SOME position, but the residual there
     * should be substantially larger than the accurate case's. */
    std::vector<std::uint8_t> mismatchedCurrentPixels(
        previousPixels.size());
    for (std::size_t index = 0U; index < mismatchedCurrentPixels.size();
        ++index)
    {
        mismatchedCurrentPixels[index] =
            static_cast<std::uint8_t>(255U - previousPixels[index]);
    }
    const ImageView mismatchedCurrentImage =
        makeView(mismatchedCurrentPixels, 256, 256);

    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        mismatchedError{};
    const auto mismatchedStatus =
        tracker.track(previousImage, mismatchedCurrentImage,
                      previousFeatures, 1U, currentFeatures, status,
                      mismatchedError);
    ASSERT_EQ(mismatchedStatus,
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    if (status[0] == 1U)
    {
        EXPECT_GT(mismatchedError[0], accurateError[0]);
    }
}

TEST(PyramidalLucasKanadeTrackerTest,
    HandlesTheFullConfiguredMaximumFeatureBatchWithoutOverrun)
{
    PyramidalLucasKanadeTracker tracker;
    ASSERT_EQ(tracker.initialize(256, 256, 2, 9, 10, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(256, 256, 0.0F, 0.0F);
    const std::vector<std::uint8_t> currentPixels =
        makeTexturedImage(256, 256, 1.0F, 1.0F);
    const ImageView previousImage = makeView(previousPixels, 256, 256);
    const ImageView currentImage = makeView(currentPixels, 256, 256);

    constexpr std::size_t featureCount =
        PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES;
    std::array<Point2D, featureCount> previousFeatures{};
    for (std::size_t index = 0U; index < featureCount; ++index)
    {
        previousFeatures[index] = Point2D{128.0F, 128.0F};
    }
    std::array<Point2D, featureCount> currentFeatures{};
    std::array<std::uint8_t, featureCount> status{};
    std::array<float, featureCount> error{};

    ASSERT_EQ(tracker.track(previousImage, currentImage, previousFeatures,
                            featureCount, currentFeatures, status, error),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    for (std::size_t index = 0U; index < featureCount; ++index)
    {
        EXPECT_EQ(status[index], 1U);
    }
}

TEST(PyramidalLucasKanadeTrackerTest, IsDeterministicAcrossRepeatedCalls)
{
    PyramidalLucasKanadeTracker tracker;
    /* Coarsest level (256 -> 128 -> 64 -> 32) must be >= windowSizePx
     * (21). */
    ASSERT_EQ(tracker.initialize(256, 256, 3, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(256, 256, 0.0F, 0.0F);
    const std::vector<std::uint8_t> currentPixels =
        makeTexturedImage(256, 256, 3.0F, -2.0F);
    const ImageView previousImage = makeView(previousPixels, 256, 256);
    const ImageView currentImage = makeView(currentPixels, 256, 256);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{128.0F, 128.0F};

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        firstFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        firstStatus{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        firstError{};
    ASSERT_EQ(tracker.track(previousImage, currentImage, previousFeatures,
                            1U, firstFeatures, firstStatus, firstError),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    /* A systematic bug that always loses every feature would otherwise
     * make this test pass vacuously (both calls losing the feature
     * "consistently"); require an actual, successful track. */
    ASSERT_EQ(firstStatus[0], 1U);

    for (int repeat = 0; repeat < 5; ++repeat)
    {
        std::array<Point2D,
                   PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
            repeatFeatures{};
        std::array<std::uint8_t,
                  PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
            repeatStatus{};
        std::array<float,
                   PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
            repeatError{};
        ASSERT_EQ(tracker.track(previousImage, currentImage,
                                previousFeatures, 1U, repeatFeatures,
                                repeatStatus, repeatError),
                 FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

        EXPECT_EQ(repeatStatus[0], firstStatus[0]);
        EXPECT_FLOAT_EQ(repeatFeatures[0].x, firstFeatures[0].x);
        EXPECT_FLOAT_EQ(repeatFeatures[0].y, firstFeatures[0].y);
        EXPECT_FLOAT_EQ(repeatError[0], firstError[0]);
    }
}

TEST(PyramidalLucasKanadeTrackerTest,
    ConvergesWithinAConstrainedIterationBudget)
{
    PyramidalLucasKanadeTracker tracker;
    /* A tight iteration budget must still terminate promptly (never
     * loop indefinitely) and still return a well-defined result for a
     * modest, well-textured displacement. Coarsest level
     * (256 -> 128 -> 64 -> 32) must be >= windowSizePx (21). */
    ASSERT_EQ(tracker.initialize(256, 256, 3, 21, 3, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(256, 256, 0.0F, 0.0F);
    const std::vector<std::uint8_t> currentPixels =
        makeTexturedImage(256, 256, 1.0F, 1.0F);
    const ImageView previousImage = makeView(previousPixels, 256, 256);
    const ImageView currentImage = makeView(currentPixels, 256, 256);

    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    previousFeatures[0] = Point2D{128.0F, 128.0F};
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    ASSERT_EQ(
        tracker.track(previousImage, currentImage, previousFeatures, 1U,
                      currentFeatures, status, error),
        FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    /* A systematic bug that always loses the feature would otherwise
     * make this test pass vacuously; require an actual, successful
     * track within the constrained iteration budget. */
    EXPECT_EQ(status[0], 1U);
}

TEST(PyramidalLucasKanadeTrackerTest,
    CompletesTrackingWithinIndicativeWallClockBudget)
{
    /* This bounds wall-clock time on whatever machine runs the test; it
     * is indicative only and is NOT a certified worst-case-execution-time
     * analysis (see
     * docs/compliance/feature_tracking/VERIFICATION_REPORT.md). */
    PyramidalLucasKanadeTracker tracker;
    ASSERT_EQ(tracker.initialize(1024, 1024, 3, 21, 30, 0.01F, 1.0e-4F),
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);

    const std::vector<std::uint8_t> previousPixels =
        makeTexturedImage(1024, 1024, 0.0F, 0.0F);
    const std::vector<std::uint8_t> currentPixels =
        makeTexturedImage(1024, 1024, 2.0F, 1.0F);
    const ImageView previousImage = makeView(previousPixels, 1024, 1024);
    const ImageView currentImage = makeView(currentPixels, 1024, 1024);

    constexpr std::size_t featureCount = 500U;
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        previousFeatures{};
    for (std::size_t index = 0U; index < featureCount; ++index)
    {
        previousFeatures[index] =
            Point2D{100.0F + static_cast<float>(index % 20) * 40.0F,
                    100.0F +
                        static_cast<float>(index / 20) * 40.0F};
    }
    std::array<Point2D, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        currentFeatures{};
    std::array<std::uint8_t,
              PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        status{};
    std::array<float, PyramidalLucasKanadeTracker::MAXIMUM_SUPPORTED_FEATURES>
        error{};

    const auto start = std::chrono::steady_clock::now();
    const auto trackStatus =
        tracker.track(previousImage, currentImage, previousFeatures,
                      featureCount, currentFeatures, status, error);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(trackStatus,
             FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS);
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed)
                 .count(),
             2000);
}
