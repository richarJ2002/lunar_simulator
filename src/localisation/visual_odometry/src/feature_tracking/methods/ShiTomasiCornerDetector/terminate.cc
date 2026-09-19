/*!
 * @File:         terminate.cc
 *
 * @Brief:        Implements Shi-Tomasi corner detector teardown.
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
/* None */

namespace localisation::visual_odometry::feature_tracking
{

FeatureTrackingStatus ShiTomasiCornerDetector::terminate() noexcept
{
    /* Release the initialize()-time buffers rather than merely clearing
     * their logical size, since "no heap allocation after
     * initialization" only has teeth if terminate() actually gives the
     * memory back. */
    gradientX_.clear();
    gradientX_.shrink_to_fit();

    /* Release the vertical-gradient buffer. */
    gradientY_.clear();
    gradientY_.shrink_to_fit();

    /* Release the response buffer. */
    response_.clear();
    response_.shrink_to_fit();

    /* Return every configuration field to its default, uninitialized
     * value. */
    width_             = 0;
    height_            = 0;
    maximumFeatures_   = 0U;
    qualityLevel_      = 0.0F;
    minimumDistancePx_ = 0.0F;

    /* A subsequent detect() call must once again be rejected until the
     * next successful initialize(). */
    isInitialized_ = false;

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
