/*!
 * @File:         computeStructureTensorAndResponse.cc
 *
 * @Brief:        Implements the structure-tensor minimum-eigenvalue
 *                response pass.
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

namespace
{

/*!
 * @brief           Mirrors an out-of-range 1-D index back into range
 *                  without duplicating the edge index (matches OpenCV's
 *                  `BORDER_REFLECT_101`).
 *
 * Duplicated from `computeGradients.cc`'s identical helper rather than
 * shared, since both are small, private, file-local implementation
 * details, not part of any public contract.
 *
 * @param[in]       index_in
 *                  Index to mirror; may be one step outside `[0, size_in)`
 *                  in either direction, which is the only range this
 *                  function's 3x3-window caller ever passes.
 * @param[in]       size_in
 *                  Valid index count along this axis.
 *
 * @return          `index_in` reflected into `[0, size_in)`.
 */
[[nodiscard]] int reflectIndex(int index_in, int size_in) noexcept
{
    if (index_in < 0)
    {
        return -index_in;
    }
    if (index_in >= size_in)
    {
        return (2 * (size_in - 1)) - index_in;
    }
    return index_in;
}

} /* anonymous namespace */

float ShiTomasiCornerDetector::computeStructureTensorAndResponse() noexcept
{
    /* Tracks the strongest response seen anywhere in the frame, which
     * detect() scales by qualityLevel_ to form the acceptance
     * threshold. */
    float globalMaxResponse = 0.0F;

    for (int row = 0; row < height_; ++row)
    {
        for (int column = 0; column < width_; ++column)
        {
            /* Accumulate the local structure tensor's three independent
             * entries (it is symmetric, so Syx == Sxy is never stored
             * separately). */
            float sumIxIx = 0.0F;
            float sumIyIy = 0.0F;
            float sumIxIy = 0.0F;

            /*!
             * Uniform-mean box filter over the 3x3 structure-tensor
             * window, matching `cv::cornerMinEigenVal`'s default
             * (non-Gaussian-weighted) window.
             */
            for (int windowRow = -1; windowRow <= 1; ++windowRow)
            {
                /* Mirrored sample row for this window row. */
                const int sampleRow = reflectIndex(row + windowRow, height_);

                for (int windowColumn = -1; windowColumn <= 1; ++windowColumn)
                {
                    /* Mirrored sample column for this window column. */
                    const int sampleColumn =
                        reflectIndex(column + windowColumn, width_);

                    /* Row-major offset of this window sample. */
                    const std::size_t sampleOffset =
                        (static_cast<std::size_t>(sampleRow) *
                         static_cast<std::size_t>(width_)) +
                        static_cast<std::size_t>(sampleColumn);

                    /* This window sample's precomputed gradients; ix/iy
                     * match the reference algorithm's own Ix/Iy
                     * notation, standard in this domain. */
                    // NOLINTNEXTLINE(readability-identifier-length)
                    const float ix = gradientX_[sampleOffset];
                    // NOLINTNEXTLINE(readability-identifier-length)
                    const float iy = gradientY_[sampleOffset];

                    /* Accumulate this sample's contribution to Sxx. */
                    sumIxIx += ix * ix;

                    /* Accumulate this sample's contribution to Syy. */
                    sumIyIy += iy * iy;

                    /* Accumulate this sample's contribution to Sxy. */
                    sumIxIy += ix * iy;
                }
            }

            /*!
             * Closed-form minimum eigenvalue of the symmetric 2x2
             * structure tensor `[[Sxx,Sxy],[Sxy,Syy]]`:
             *   lambda_min = ((Sxx+Syy) - sqrt((Sxx-Syy)^2 + 4*Sxy^2)) / 2
             * Both eigenvalues of this positive-semidefinite matrix are
             * real and non-negative, so lambda_min is never negative and
             * the discriminant under the square root is never negative
             * either.
             */
            const float trace        = sumIxIx + sumIyIy;
            const float difference   = sumIxIx - sumIyIy;
            const float discriminant = std::sqrt((difference * difference) +
                                                 (4.0F * sumIxIy * sumIxIy));
            const float minimumEigenvalue = 0.5F * (trace - discriminant);

            /* Row-major offset into the response buffer. */
            const std::size_t offset = (static_cast<std::size_t>(row) *
                                        static_cast<std::size_t>(width_)) +
                                       static_cast<std::size_t>(column);

            /* Store this pixel's response for collectCandidates() to
             * threshold and rank later. */
            response_[offset] = minimumEigenvalue;

            /* Track the strongest response seen so far this frame. */
            if (minimumEigenvalue > globalMaxResponse)
            {
                globalMaxResponse = minimumEigenvalue;
            }
        }
    }

    return globalMaxResponse;
}

} /* namespace localisation::visual_odometry::feature_tracking */
