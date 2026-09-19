/*!
 * @File:         refineFeatureAtLevel.cc
 *
 * @Brief:        Implements iterative Gauss-Newton position refinement at
 *                one pyramid level.
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
#include <cmath>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

/* This function's cognitive-complexity finding is accepted: it is one
 * cohesive algorithmic step (Bouguet's inverse-compositional Gauss-Newton
 * refinement) matching the reference algorithm's own structure; splitting
 * it further would reduce traceability without reducing the underlying
 * mathematical complexity -- see DEVIATION_LOG.md. */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool PyramidalLucasKanadeTracker::refineFeatureAtLevel(
    const PyramidLevel &currentLevel_in,
    const PyramidLevel &previousLevel_in,
    Point2D             templatePosition_in,
    Point2D            &searchPosition_inout,
    float              &error_out) const noexcept
{
    /* Half-width of the square tracking window; windowSizePx_ is odd, so
     * the window spans [-half, +half] around its center inclusive. */
    const int half = (windowSizePx_ - 1) / 2;

    /* Fixed-capacity stack storage for one feature's template window,
     * bounded by MAXIMUM_WINDOW_SIZE_PX regardless of the configured
     * (smaller) windowSizePx_; this is ordinary automatic storage, not a
     * heap allocation, and does not persist beyond this call. */
    constexpr std::size_t maximumWindowArea =
        static_cast<std::size_t>(MAXIMUM_WINDOW_SIZE_PX) *
        static_cast<std::size_t>(MAXIMUM_WINDOW_SIZE_PX);
    std::array<float, maximumWindowArea> templateIntensity{};
    std::array<float, maximumWindowArea> templateGradientX{};
    std::array<float, maximumWindowArea> templateGradientY{};

    /*!
     * Precompute the fixed template window's intensity and gradients
     * once, sampled from previousLevel_in at templatePosition_in, plus
     * the structure tensor they imply. Neither the template position nor
     * these values change across the iterations below (this is the
     * "inverse compositional" formulation Bouguet's pyramidal
     * Lucas-Kanade paper describes, and what `cv::calcOpticalFlowPyrLK`
     * itself implements): only searchPosition_inout, the estimate of
     * where this fixed template appears in the current/target image, is
     * refined.
     */
    float sumIxIx = 0.0F;
    float sumIyIy = 0.0F;
    float sumIxIy = 0.0F;

    std::size_t windowIndex = 0U;
    for (int windowRow = -half; windowRow <= half; ++windowRow)
    {
        for (int windowColumn = -half; windowColumn <= half; ++windowColumn)
        {
            const Point2D samplePosition{
                templatePosition_in.x + static_cast<float>(windowColumn),
                templatePosition_in.y + static_cast<float>(windowRow)};

            float      intensity = 0.0F;
            float      gradientX = 0.0F;
            float      gradientY = 0.0F;
            const bool intensityInBounds =
                sampleBilinear(previousLevel_in.intensity,
                               previousLevel_in.width,
                               previousLevel_in.height,
                               samplePosition,
                               intensity);
            const bool gradientXInBounds =
                sampleBilinear(previousLevel_in.gradientX,
                               previousLevel_in.width,
                               previousLevel_in.height,
                               samplePosition,
                               gradientX);
            const bool gradientYInBounds =
                sampleBilinear(previousLevel_in.gradientY,
                               previousLevel_in.width,
                               previousLevel_in.height,
                               samplePosition,
                               gradientY);

            /* The fixed template window itself leaves the image; there
             * is nothing meaningful to track from here at this level. */
            if (!intensityInBounds || !gradientXInBounds || !gradientYInBounds)
            {
                error_out = 0.0F;
                return false;
            }

            templateIntensity[windowIndex] = intensity;
            templateGradientX[windowIndex] = gradientX;
            templateGradientY[windowIndex] = gradientY;

            sumIxIx += gradientX * gradientX;
            sumIyIy += gradientY * gradientY;
            sumIxIy += gradientX * gradientY;

            ++windowIndex;
        }
    }

    /*!
     * Closed-form minimum eigenvalue of the fixed template window's
     * structure tensor, using the same formula as
     * `ShiTomasiCornerDetector::computeStructureTensorAndResponse()`. A
     * too-low-texture window (near-uniform intensity in every direction)
     * cannot be tracked reliably and is rejected here rather than
     * allowed to produce an arbitrary, numerically unstable refinement.
     */
    const float trace      = sumIxIx + sumIyIy;
    const float difference = sumIxIx - sumIyIy;
    const float discriminant =
        std::sqrt((difference * difference) + (4.0F * sumIxIy * sumIxIy));
    const float minimumEigenvalue = 0.5F * (trace - discriminant);

    if (minimumEigenvalue < minimumEigenvalueThreshold_)
    {
        error_out = 0.0F;
        return false;
    }

    /* Closed-form inverse of the fixed 2x2 structure tensor, reused by
     * every iteration below; guarded against a near-singular matrix even
     * though the eigenvalue gate above already makes this exceedingly
     * unlikely to trigger except when minimumEigenvalueThreshold_ is
     * configured as exactly zero. */
    const float determinant = (sumIxIx * sumIyIy) - (sumIxIy * sumIxIy);
    if (!(std::abs(determinant) > 0.0F))
    {
        error_out = 0.0F;
        return false;
    }
    const float inverseDeterminant = 1.0F / determinant;

    /* Iterative Gauss-Newton refinement of searchPosition_inout. */
    for (int iteration = 0; iteration < maximumIterations_; ++iteration)
    {
        float sumGradientXMismatch = 0.0F;
        float sumGradientYMismatch = 0.0F;

        bool windowInBounds = true;
        windowIndex         = 0U;
        for (int windowRow = -half; windowRow <= half && windowInBounds;
             ++windowRow)
        {
            for (int windowColumn = -half; windowColumn <= half; ++windowColumn)
            {
                const Point2D currentSamplePosition{
                    searchPosition_inout.x + static_cast<float>(windowColumn),
                    searchPosition_inout.y + static_cast<float>(windowRow)};

                float currentIntensity = 0.0F;
                if (!sampleBilinear(currentLevel_in.intensity,
                                    currentLevel_in.width,
                                    currentLevel_in.height,
                                    currentSamplePosition,
                                    currentIntensity))
                {
                    windowInBounds = false;
                    break;
                }

                /* Image mismatch at this window sample, weighted by the
                 * fixed template gradient (Bouguet's b vector). */
                const float mismatch =
                    templateIntensity[windowIndex] - currentIntensity;
                sumGradientXMismatch +=
                    templateGradientX[windowIndex] * mismatch;
                sumGradientYMismatch +=
                    templateGradientY[windowIndex] * mismatch;

                ++windowIndex;
            }
        }

        /* The search window left the image partway through this
         * iteration; nothing meaningful can be refined further. */
        if (!windowInBounds)
        {
            error_out = 0.0F;
            return false;
        }

        /* Closed-form 2x2 solve: delta = G^-1 * b. */
        const float deltaX =
            inverseDeterminant * ((sumIyIy * sumGradientXMismatch) -
                                  (sumIxIy * sumGradientYMismatch));
        const float deltaY =
            inverseDeterminant * ((sumIxIx * sumGradientYMismatch) -
                                  (sumIxIy * sumGradientXMismatch));

        searchPosition_inout.x += deltaX;
        searchPosition_inout.y += deltaY;

        /* Converged once the per-iteration update is smaller than the
         * configured epsilon; the remaining iteration budget is simply
         * not spent. */
        const float deltaMagnitude =
            std::sqrt((deltaX * deltaX) + (deltaY * deltaY));
        if (deltaMagnitude < epsilonPx_)
        {
            break;
        }
    }

    /*!
     * Final residual pass at the converged (or iteration-budget-
     * exhausted) position: a fresh pass rather than reusing the last
     * iteration's mismatch values, since those were computed against the
     * position BEFORE that iteration's own update was applied.
     */
    float sumAbsoluteResidual = 0.0F;
    windowIndex               = 0U;
    for (int windowRow = -half; windowRow <= half; ++windowRow)
    {
        for (int windowColumn = -half; windowColumn <= half; ++windowColumn)
        {
            const Point2D finalSamplePosition{
                searchPosition_inout.x + static_cast<float>(windowColumn),
                searchPosition_inout.y + static_cast<float>(windowRow)};

            float finalIntensity = 0.0F;
            if (!sampleBilinear(currentLevel_in.intensity,
                                currentLevel_in.width,
                                currentLevel_in.height,
                                finalSamplePosition,
                                finalIntensity))
            {
                error_out = 0.0F;
                return false;
            }

            sumAbsoluteResidual +=
                std::abs(templateIntensity[windowIndex] - finalIntensity);
            ++windowIndex;
        }
    }

    const std::size_t windowArea = static_cast<std::size_t>(windowSizePx_) *
                                   static_cast<std::size_t>(windowSizePx_);
    error_out = sumAbsoluteResidual / static_cast<float>(windowArea);

    return true;
}

} /* namespace localisation::visual_odometry::feature_tracking */
