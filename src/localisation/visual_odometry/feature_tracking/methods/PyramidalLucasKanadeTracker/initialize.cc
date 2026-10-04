/*!
 * @File:         initialize.cc
 *
 * @Brief:        Implements pyramidal Lucas-Kanade tracker initialization.
 *
 * @Date:         16/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "feature_tracking/objects/PyramidalLucasKanadeTracker.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

/* This is an established, tested public API (see
 * test_optical_flow_tracker.cpp): every parameter is independently
 * range-validated below, so a swapped call is rejected by that
 * validation, not merely by hoping the caller orders arguments
 * correctly. */
FeatureTrackingStatus PyramidalLucasKanadeTracker::initialize(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    int   width_in,
    int   height_in,
    int   maximumPyramidLevel_in,
    int   windowSizePx_in,
    int   maximumIterations_in,
    float epsilonPx_in,
    float minimumEigenvalueThreshold_in) noexcept
{
    /* Reject a resolution outside the configured sanity ceiling. */
    if (width_in <= 0 ||
        width_in > feature_tracking::MAXIMUM_SUPPORTED_WIDTH_PX ||
        height_in <= 0 ||
        height_in > feature_tracking::MAXIMUM_SUPPORTED_HEIGHT_PX)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Reject a pyramid depth outside the fixed-capacity level arrays. */
    if (maximumPyramidLevel_in < 0 ||
        maximumPyramidLevel_in >= MAXIMUM_SUPPORTED_PYRAMID_LEVELS)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* The tracking window must be odd (it has a well-defined center
     * pixel), wide enough to carry meaningful gradient information, and
     * no wider than the fixed-size stack buffers refineFeatureAtLevel()
     * uses to hold one feature's template window. */
    if (windowSizePx_in < 3 || (windowSizePx_in % 2) == 0 ||
        windowSizePx_in > MAXIMUM_WINDOW_SIZE_PX)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* An iteration budget of zero would never refine anything. */
    if (maximumIterations_in <= 0)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /*!
     * The coarsest pyramid level must be at least as large as the
     * tracking window in both dimensions, matching buildPyramid()'s
     * `(w+1)/2` halving-per-level convention. Otherwise no feature --
     * not even one at the coarsest level's own center -- could ever have
     * a fully in-bounds window there, and every feature would be
     * silently lost at the coarsest level regardless of image content.
     */
    int coarsestLevelWidth  = width_in;
    int coarsestLevelHeight = height_in;
    for (int level = 0; level < maximumPyramidLevel_in; ++level)
    {
        coarsestLevelWidth  = (coarsestLevelWidth + 1) / 2;
        coarsestLevelHeight = (coarsestLevelHeight + 1) / 2;
    }
    if (coarsestLevelWidth < windowSizePx_in ||
        coarsestLevelHeight < windowSizePx_in)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* A non-positive convergence threshold could never be satisfied. */
    if (!std::isfinite(epsilonPx_in) || epsilonPx_in <= 0.0F)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* A negative eigenvalue threshold is not physically meaningful (the
     * structure tensor's eigenvalues are never negative); zero is
     * permitted (disables the low-texture gate entirely). */
    if (!std::isfinite(minimumEigenvalueThreshold_in) ||
        minimumEigenvalueThreshold_in < 0.0F)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Every parameter validated; commit the configuration. */
    width_                      = width_in;
    height_                     = height_in;
    maximumPyramidLevel_        = maximumPyramidLevel_in;
    windowSizePx_               = windowSizePx_in;
    maximumIterations_          = maximumIterations_in;
    epsilonPx_                  = epsilonPx_in;
    minimumEigenvalueThreshold_ = minimumEigenvalueThreshold_in;

    /*!
     * This is the one heap allocation this object ever performs: every
     * pyramid level's fixed-size storage, sized exactly once here and
     * never resized by track(). Level 0 is the full resolution; each
     * coarser level is `(w+1)/2 x (h+1)/2` of the one below it, matching
     * `cv::pyrDown`'s own size convention.
     */
    int levelWidth  = width_;
    int levelHeight = height_;
    for (int level = 0; level <= maximumPyramidLevel_; ++level)
    {
        const std::size_t pixelCount = static_cast<std::size_t>(levelWidth) *
                                       static_cast<std::size_t>(levelHeight);

        /* Template/previous pyramid: record this level's dimensions. */
        previousPyramid_[static_cast<std::size_t>(level)].width  = levelWidth;
        previousPyramid_[static_cast<std::size_t>(level)].height = levelHeight;

        /* Allocate this level's intensity buffer. */
        previousPyramid_[static_cast<std::size_t>(level)].intensity.assign(
            pixelCount,
            0.0F);

        /* Allocate this level's gradient buffers -- only the template/
         * previous pyramid needs gradients; the target/current pyramid
         * only needs bilinear-sampled intensity. */
        previousPyramid_[static_cast<std::size_t>(level)].gradientX.assign(
            pixelCount,
            0.0F);
        previousPyramid_[static_cast<std::size_t>(level)].gradientY.assign(
            pixelCount,
            0.0F);

        /* Target/current pyramid: record this level's dimensions. */
        currentPyramid_[static_cast<std::size_t>(level)].width  = levelWidth;
        currentPyramid_[static_cast<std::size_t>(level)].height = levelHeight;

        /* Allocate this level's intensity buffer (gradientX/gradientY
         * stay empty for this pyramid). */
        currentPyramid_[static_cast<std::size_t>(level)].intensity.assign(
            pixelCount,
            0.0F);

        /* Derive the next (coarser) level's dimensions. */
        levelWidth  = (levelWidth + 1) / 2;
        levelHeight = (levelHeight + 1) / 2;
    }

    /* The defined initialization phase is now complete. */
    isInitialized_ = true;

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
