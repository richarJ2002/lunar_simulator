/*!
 * @File:         initialize.cc
 *
 * @Brief:        Implements Shi-Tomasi corner detector initialization.
 *
 * @Date:         16/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "feature_tracking/objects/ShiTomasiCornerDetector.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <cmath>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

/* This is an established, tested public API (see
 * test_corner_detector.cpp): every parameter is independently
 * range-validated below, so a swapped call is rejected by that
 * validation, not merely by hoping the caller orders arguments
 * correctly. */
FeatureTrackingStatus ShiTomasiCornerDetector::initialize(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    int         width_in,
    int         height_in,
    std::size_t maximumFeatures_in,
    float       qualityLevel_in,
    float       minimumDistancePx_in) noexcept
{
    /* Reject a resolution outside the configured sanity ceiling before
     * ever touching it, rather than allocating first and discovering the
     * problem later. */
    if (width_in <= 0 ||
        width_in > feature_tracking::MAXIMUM_SUPPORTED_WIDTH_PX ||
        height_in <= 0 ||
        height_in > feature_tracking::MAXIMUM_SUPPORTED_HEIGHT_PX)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Reject a feature cap outside the compile-time capacity every output
     * buffer is sized to. */
    if (maximumFeatures_in == 0U ||
        maximumFeatures_in > MAXIMUM_SUPPORTED_FEATURES)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Quality level selects a fraction of the frame's strongest response;
     * only (0, 1] is meaningful. */
    if (!std::isfinite(qualityLevel_in) || qualityLevel_in <= 0.0F ||
        qualityLevel_in > 1.0F)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* A non-positive minimum distance would accept every candidate,
     * defeating the purpose of the separation constraint. */
    if (!std::isfinite(minimumDistancePx_in) || minimumDistancePx_in <= 0.0F)
    {
        return FeatureTrackingStatus::
            FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION;
    }

    /* Every parameter validated; commit the configuration. */
    width_ = width_in;

    /* Store the validated height. */
    height_ = height_in;

    /* Store the validated feature cap. */
    maximumFeatures_ = maximumFeatures_in;

    /* Store the validated quality level. */
    qualityLevel_ = qualityLevel_in;

    /* Store the validated minimum separation. */
    minimumDistancePx_ = minimumDistancePx_in;

    /* This is the one heap allocation this object ever performs: three
     * full-resolution buffers, sized exactly once here and never resized
     * by detect(). */
    const std::size_t pixelCount =
        static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);

    /* Allocate (and zero-initialize) the horizontal-gradient buffer. */
    gradientX_.assign(pixelCount, 0.0F);

    /* Allocate (and zero-initialize) the vertical-gradient buffer. */
    gradientY_.assign(pixelCount, 0.0F);

    /* Allocate (and zero-initialize) the per-pixel response buffer. */
    response_.assign(pixelCount, 0.0F);

    /* The defined initialization phase is now complete. */
    isInitialized_ = true;

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
