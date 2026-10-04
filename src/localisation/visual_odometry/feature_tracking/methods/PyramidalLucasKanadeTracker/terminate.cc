/*!
 * @File:         terminate.cc
 *
 * @Brief:        Implements pyramidal Lucas-Kanade tracker teardown.
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
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

FeatureTrackingStatus PyramidalLucasKanadeTracker::terminate() noexcept
{
    /* Release every pyramid level's buffers rather than merely clearing
     * their logical size, since "no heap allocation after
     * initialization" only has teeth if terminate() actually gives the
     * memory back. */
    for (int level = 0; level <= maximumPyramidLevel_ &&
                        level < MAXIMUM_SUPPORTED_PYRAMID_LEVELS;
         ++level)
    {
        PyramidLevel &previousLevel =
            previousPyramid_[static_cast<std::size_t>(level)];
        previousLevel.intensity.clear();
        previousLevel.intensity.shrink_to_fit();
        previousLevel.gradientX.clear();
        previousLevel.gradientX.shrink_to_fit();
        previousLevel.gradientY.clear();
        previousLevel.gradientY.shrink_to_fit();
        previousLevel.width  = 0;
        previousLevel.height = 0;

        PyramidLevel &currentLevel =
            currentPyramid_[static_cast<std::size_t>(level)];
        currentLevel.intensity.clear();
        currentLevel.intensity.shrink_to_fit();
        currentLevel.width  = 0;
        currentLevel.height = 0;
    }

    /* Return every configuration field to its default, uninitialized
     * value. */
    width_                      = 0;
    height_                     = 0;
    maximumPyramidLevel_        = 0;
    windowSizePx_               = 0;
    maximumIterations_          = 0;
    epsilonPx_                  = 0.0F;
    minimumEigenvalueThreshold_ = 0.0F;

    /* A subsequent track() call must once again be rejected until the
     * next successful initialize(). */
    isInitialized_ = false;

    return FeatureTrackingStatus::FEATURE_TRACKING_STATUS_SUCCESS;
}

} /* namespace localisation::visual_odometry::feature_tracking */
