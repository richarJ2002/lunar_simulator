/*!
 * @File:         detect.cc
 *
 * @Brief:        Implements Shi-Tomasi corner detection over one image.
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
#include <array>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

FeatureTrackingStatus ShiTomasiCornerDetector::detect(
    const ImageView                                 &image_in,
    std::array<Point2D, MAXIMUM_SUPPORTED_FEATURES> &features_out,
    std::size_t                                     &featureCount_out) noexcept
{
    /* Report zero features on every early-return path, so the caller
     * never sees a stale count from a previous call. */
    featureCount_out = 0U;

    /* Reject a call before a successful initialize(). */
    if (!isInitialized_)
    {
        return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_NOT_INITIALIZED;
    }

    /* Reject a null buffer or a resolution mismatch against what
     * initialize() allocated buffers for. */
    if (image_in.p_pixels == nullptr || image_in.width != width_ ||
        image_in.height != height_)
    {
        return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_INVALID_INPUT;
    }

    /* Step 1: horizontal/vertical Sobel gradients into gradientX_/
     * gradientY_. */
    computeGradients(image_in);

    /* Step 2: per-pixel minimum structure-tensor eigenvalue into
     * response_, and the frame's strongest response. */
    const float globalMaxResponse = computeStructureTensorAndResponse();

    /*!
     * A minimum eigenvalue of a positive-semidefinite matrix is never
     * negative, so when globalMaxResponse is exactly zero (a perfectly
     * flat image with no gradient energy anywhere), every pixel's
     * response is also exactly zero. The threshold below is then exactly
     * zero too, and the strict "greater than" comparison inside
     * collectCandidates() naturally rejects every pixel -- no special
     * case is required to handle a flat image correctly.
     */
    const float responseThreshold = qualityLevel_ * globalMaxResponse;

    /* Step 3: bounded local-maxima extraction above threshold. */
    const std::size_t candidateCount = collectCandidates(responseThreshold);

    /* Step 4: rank and greedily select subject to minimum separation. */
    featureCount_out = selectByMinimumDistance(candidateCount, features_out);

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
