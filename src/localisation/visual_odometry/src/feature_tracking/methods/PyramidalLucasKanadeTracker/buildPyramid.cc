/*!
 * @File:         buildPyramid.cc
 *
 * @Brief:        Implements Gaussian pyramid construction, with optional
 *                per-level gradients.
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

namespace localisation::visual_odometry::feature_tracking
{

namespace
{

/*!
 * @brief           Mirrors an out-of-range 1-D index back into range
 *                  without duplicating the edge index (matches OpenCV's
 *                  `BORDER_REFLECT_101`).
 *
 * @param[in]       index_in
 *                  Index to mirror; may be up to two steps outside
 *                  `[0, size_in)` in either direction, which is the
 *                  widest range this file's 5-tap-kernel callers ever
 *                  pass.
 * @param[in]       size_in
 *                  Valid index count along this axis.
 *
 * @return          `index_in` reflected into `[0, size_in)`.
 */
// index_in/size_in are unambiguous by name and role; a swap is a
// compile-time-obvious misuse this file-local helper's own call sites
// don't exhibit.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] int reflectIndex(int index_in, int size_in) noexcept
{
    int reflected = index_in;

    /* A single reflectIndex() call only ever needs to fold an index back
     * at most once for this file's callers (offsets of at most 2 against
     * a level whose smallest dimension is always > 4), but folding in a
     * loop keeps this helper correct for any offset magnitude. */
    while (reflected < 0 || reflected >= size_in)
    {
        if (reflected < 0)
        {
            reflected = -reflected;
        }
        else
        {
            reflected = (2 * (size_in - 1)) - reflected;
        }
    }

    return reflected;
}

/*!
 * @brief       Binomial 5-tap approximation to a Gaussian, `[1,4,6,4,1]`,
 *              unnormalized; the 2-D separable kernel this file applies
 *              is this vector's outer product with itself, normalized by
 *              its sum-of-sums (256).
 */
constexpr std::array<float, 5U> GAUSSIAN_KERNEL_5TAP = {1.0F,
                                                        4.0F,
                                                        6.0F,
                                                        4.0F,
                                                        1.0F};

} /* anonymous namespace */

/* This function's cognitive-complexity finding is accepted: it is one
 * cohesive algorithmic step (Gaussian pyramid construction plus optional
 * per-level Sobel gradients) matching the reference algorithm's own
 * structure; splitting it further would reduce traceability without
 * reducing the underlying mathematical complexity -- see
 * DEVIATION_LOG.md. */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void PyramidalLucasKanadeTracker::buildPyramid(
    const ImageView &image_in,
    bool             computeGradients_in,
    std::array<PyramidLevel, MAXIMUM_SUPPORTED_PYRAMID_LEVELS> &pyramid_inout)
    const noexcept
{
    /* Level 0 is the input image itself, copied in as float; every
     * coarser level is derived from this one below. */
    PyramidLevel &level0 = pyramid_inout[0];
    for (int row = 0; row < level0.height; ++row)
    {
        for (int column = 0; column < level0.width; ++column)
        {
            const std::size_t offset =
                (static_cast<std::size_t>(row) *
                 static_cast<std::size_t>(level0.width)) +
                static_cast<std::size_t>(column);
            level0.intensity[offset] =
                static_cast<float>(image_in.at(row, column));
        }
    }

    /*!
     * Build each coarser level from the one immediately below it via a
     * fused 5x5-binomial-blur-then-decimate-by-2 pass (matching
     * `cv::pyrDown`'s size and blur convention), evaluated directly per
     * OUTPUT pixel as a 25-tap 2-D convolution rather than materializing
     * a full-resolution intermediate blurred image -- this needs no
     * scratch buffer beyond the pyramid levels `initialize()` already
     * allocated, keeping this call allocation-free.
     */
    for (int level = 0; level < maximumPyramidLevel_; ++level)
    {
        const PyramidLevel &fineLevel =
            pyramid_inout[static_cast<std::size_t>(level)];
        /* level is bounded to [0, maximumPyramidLevel_), itself bounded
         * to [0, MAXIMUM_SUPPORTED_PYRAMID_LEVELS) by initialize(); level
         * + 1 cannot overflow int before the widening cast. */
        PyramidLevel &coarseLevel =
            // NOLINTNEXTLINE(bugprone-misplaced-widening-cast)
            pyramid_inout[static_cast<std::size_t>(level + 1)];

        for (int outputRow = 0; outputRow < coarseLevel.height; ++outputRow)
        {
            /* Output pixel (outputRow, *) samples fineLevel centered on
             * row 2*outputRow, per cv::pyrDown's convention. */
            const int centerRow = 2 * outputRow;

            for (int outputColumn = 0; outputColumn < coarseLevel.width;
                 ++outputColumn)
            {
                /* Output pixel (*, outputColumn) samples fineLevel
                 * centered on column 2*outputColumn. */
                const int centerColumn = 2 * outputColumn;

                /* Accumulate the 5x5 weighted sum around the center. */
                float accumulator = 0.0F;
                for (int tapRow = -2; tapRow <= 2; ++tapRow)
                {
                    const int sampleRow =
                        reflectIndex(centerRow + tapRow, fineLevel.height);
                    /* tapRow is bounded to [-2, 2]; tapRow + 2 cannot
                     * overflow int before the widening cast. */
                    const float rowWeight = GAUSSIAN_KERNEL_5TAP
                        // NOLINTNEXTLINE(bugprone-misplaced-widening-cast)
                        [static_cast<std::size_t>(tapRow + 2)];

                    for (int tapColumn = -2; tapColumn <= 2; ++tapColumn)
                    {
                        const int sampleColumn =
                            reflectIndex(centerColumn + tapColumn,
                                         fineLevel.width);
                        /* tapColumn is bounded to [-2, 2]; tapColumn + 2
                         * cannot overflow int before the widening cast. */
                        const float columnWeight = GAUSSIAN_KERNEL_5TAP
                            // NOLINTNEXTLINE(bugprone-misplaced-widening-cast)
                            [static_cast<std::size_t>(tapColumn + 2)];

                        const std::size_t sampleOffset =
                            (static_cast<std::size_t>(sampleRow) *
                             static_cast<std::size_t>(fineLevel.width)) +
                            static_cast<std::size_t>(sampleColumn);

                        accumulator += rowWeight * columnWeight *
                                       fineLevel.intensity[sampleOffset];
                    }
                }

                /* The 5x5 binomial kernel's weights sum to 256 (16 for
                 * each 1-D pass, squared for the 2-D outer product). */
                const std::size_t outputOffset =
                    (static_cast<std::size_t>(outputRow) *
                     static_cast<std::size_t>(coarseLevel.width)) +
                    static_cast<std::size_t>(outputColumn);
                coarseLevel.intensity[outputOffset] = accumulator / 256.0F;
            }
        }
    }

    /* The template/previous pyramid additionally needs each level's
     * Sobel gradients; the target/current pyramid only needs the
     * intensity levels built above. */
    if (computeGradients_in)
    {
        for (int level = 0; level <= maximumPyramidLevel_; ++level)
        {
            PyramidLevel &currentLevel =
                pyramid_inout[static_cast<std::size_t>(level)];

            for (int row = 0; row < currentLevel.height; ++row)
            {
                const int rowAbove = reflectIndex(row - 1, currentLevel.height);
                const int rowBelow = reflectIndex(row + 1, currentLevel.height);

                for (int column = 0; column < currentLevel.width; ++column)
                {
                    const int columnLeft =
                        reflectIndex(column - 1, currentLevel.width);
                    const int columnRight =
                        reflectIndex(column + 1, currentLevel.width);

                    const auto widthSizeT =
                        static_cast<std::size_t>(currentLevel.width);

                    const float topLeft =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowAbove) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnLeft)];
                    const float topCenter =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowAbove) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(column)];
                    const float topRight =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowAbove) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnRight)];
                    const float middleLeft =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(row) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnLeft)];
                    const float middleRight =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(row) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnRight)];
                    const float bottomLeft =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowBelow) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnLeft)];
                    const float bottomCenter =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowBelow) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(column)];
                    const float bottomRight =
                        currentLevel
                            .intensity[(static_cast<std::size_t>(rowBelow) *
                                        widthSizeT) +
                                       static_cast<std::size_t>(columnRight)];

                    /* Horizontal Sobel kernel: [-1 0 1; -2 0 2; -1 0 1].
                     * gx/gy match the reference algorithm's own Ix/Iy
                     * gradient notation, standard in this domain. */
                    // NOLINTNEXTLINE(readability-identifier-length)
                    const float gx =
                        (topRight + (2.0F * middleRight) + bottomRight) -
                        (topLeft + (2.0F * middleLeft) + bottomLeft);

                    /* Vertical Sobel kernel: [-1 -2 -1; 0 0 0; 1 2 1]. */
                    // NOLINTNEXTLINE(readability-identifier-length)
                    const float gy =
                        (bottomLeft + (2.0F * bottomCenter) + bottomRight) -
                        (topLeft + (2.0F * topCenter) + topRight);

                    const std::size_t offset =
                        (static_cast<std::size_t>(row) * widthSizeT) +
                        static_cast<std::size_t>(column);
                    currentLevel.gradientX[offset] = gx;
                    currentLevel.gradientY[offset] = gy;
                }
            }
        }
    }
}

} /* namespace localisation::visual_odometry::feature_tracking */
