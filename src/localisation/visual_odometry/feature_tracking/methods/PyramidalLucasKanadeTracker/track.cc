/*!
 * @File:         track.cc
 *
 * @Brief:        Implements pyramidal Lucas-Kanade tracking of a feature
 *                set between two images.
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
#include <array>
#include <cstddef>
#include <cstdint>

namespace localisation::visual_odometry::feature_tracking
{

FeatureTrackingStatus PyramidalLucasKanadeTracker::track(
    const ImageView                                       &previousImage_in,
    const ImageView                                       &currentImage_in,
    const std::array<Point2D, MAXIMUM_SUPPORTED_FEATURES> &previousFeatures_in,
    std::size_t                                      previousFeatureCount_in,
    std::array<Point2D, MAXIMUM_SUPPORTED_FEATURES> &currentFeatures_out,
    std::array<std::uint8_t, MAXIMUM_SUPPORTED_FEATURES> &trackingStatus_out,
    std::array<float, MAXIMUM_SUPPORTED_FEATURES> &trackingError_out) noexcept
{
    /* Mark every feature lost up front, so every early-return path below
     * still leaves well-defined output for the caller. */
    for (std::size_t index = 0U; index < previousFeatureCount_in; ++index)
    {
        trackingStatus_out[index]  = 0U;
        trackingError_out[index]   = 0.0F;
        currentFeatures_out[index] = Point2D{};
    }

    /* Reject a call before a successful initialize(). */
    if (!isInitialized_)
    {
        return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED;
    }

    /* Reject a null buffer or a resolution mismatch on either image
     * against what initialize() allocated pyramid levels for. */
    if (previousImage_in.p_pixels == nullptr ||
        previousImage_in.width != width_ ||
        previousImage_in.height != height_ ||
        currentImage_in.p_pixels == nullptr ||
        currentImage_in.width != width_ || currentImage_in.height != height_)
    {
        return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT;
    }

    /* Reject a feature count exceeding the fixed-size input/output
     * buffers' shared capacity. */
    if (previousFeatureCount_in > MAXIMUM_SUPPORTED_FEATURES)
    {
        return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT;
    }

    /* Build both pyramids once for this call; the template/previous
     * pyramid additionally gets per-level gradients, the target/current
     * pyramid does not (see buildPyramid()'s own documentation). */
    buildPyramid(previousImage_in, true, previousPyramid_);
    buildPyramid(currentImage_in, false, currentPyramid_);

    /* Track each requested feature independently, coarse-to-fine. */
    for (std::size_t featureIndex = 0U; featureIndex < previousFeatureCount_in;
         ++featureIndex)
    {
        /* This feature's fixed template position at the current
         * pyramid level, recomputed exactly (not propagated) from the
         * original full-resolution position each level. */
        Point2D templatePositionAtLevel{};

        /* This feature's refined search position at the current pyramid
         * level; seeded with zero initial flow at the coarsest level,
         * then carried level-to-level below. */
        Point2D searchPosition{};

        /* This level's residual error, valid only once a level succeeds;
         * the finest (last) level's value is what gets reported to the
         * caller for a feature that survives every level. */
        float levelError = 0.0F;

        bool isLost = false;

        for (int level = maximumPyramidLevel_; level >= 0; --level)
        {
            /* Scale factor from full resolution down to this level:
             * 2^level, matching buildPyramid()'s halving-per-level
             * convention. */
            const auto scale = static_cast<float>(1 << level);

            /* The template position is always derived directly from the
             * original, known previous-frame position -- never
             * propagated -- since it is exact at every level. */
            templatePositionAtLevel =
                Point2D{previousFeatures_in[featureIndex].x / scale,
                        previousFeatures_in[featureIndex].y / scale};

            if (level == maximumPyramidLevel_)
            {
                /* Coarsest level: zero initial flow, matching this
                 * tracker not supporting an externally supplied initial
                 * guess (mirroring the current call site's use of
                 * cv::calcOpticalFlowPyrLK without
                 * OPTFLOW_USE_INITIAL_FLOW). */
                searchPosition = templatePositionAtLevel;
            }
            else
            {
                /* Propagate the coarser level's converged estimate down
                 * to this (finer) level's scale. */
                searchPosition.x *= 2.0F;
                searchPosition.y *= 2.0F;
            }

            const bool refinedSuccessfully = refineFeatureAtLevel(
                currentPyramid_[static_cast<std::size_t>(level)],
                previousPyramid_[static_cast<std::size_t>(level)],
                templatePositionAtLevel,
                searchPosition,
                levelError);

            if (!refinedSuccessfully)
            {
                isLost = true;
                break;
            }
        }

        if (isLost)
        {
            /* Already defaulted to lost/zeroed above; nothing further
             * to record for this feature. */
            continue;
        }

        /* This feature survived every pyramid level; report its final
         * (finest-level, full-resolution) position and residual. */
        trackingStatus_out[featureIndex]  = 1U;
        trackingError_out[featureIndex]   = levelError;
        currentFeatures_out[featureIndex] = searchPosition;
    }

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
