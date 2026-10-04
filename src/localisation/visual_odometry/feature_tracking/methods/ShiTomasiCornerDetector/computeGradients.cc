/*!
 * @File:         computeGradients.cc
 *
 * @Brief:        Implements the Sobel gradient pass over one image.
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
 *                  Index to mirror; may be one step outside `[0, size_in)`
 *                  in either direction, which is the only range this
 *                  function's 3x3-neighborhood callers ever pass.
 * @param[in]       size_in
 *                  Valid index count along this axis.
 *
 * @return          `index_in` reflected into `[0, size_in)`.
 */
[[nodiscard]] int reflectIndex(int index_in, int size_in) noexcept
{
    /* One step below the first valid index mirrors to index 1, not
     * index 0 (the edge index itself is never duplicated). */
    if (index_in < 0)
    {
        return -index_in;
    }

    /* One step beyond the last valid index mirrors symmetrically about
     * that last index. */
    if (index_in >= size_in)
    {
        return (2 * (size_in - 1)) - index_in;
    }

    /* Already in range. */
    return index_in;
}

} /* anonymous namespace */

void ShiTomasiCornerDetector::computeGradients(
    const ImageView &image_in) noexcept
{
    /* Sweep every pixel; gradients are needed everywhere response_ will
     * later be read, including the outermost row/column (even though
     * collectCandidates() never proposes an outermost pixel as a
     * candidate itself, its box-filtered structure tensor still reads
     * one ring of neighbors around it). */
    for (int row = 0; row < height_; ++row)
    {
        /* Mirrored row indices for this row's 3x3 neighborhood. */
        const int rowAbove = reflectIndex(row - 1, height_);
        const int rowBelow = reflectIndex(row + 1, height_);

        for (int column = 0; column < width_; ++column)
        {
            /* Mirrored column indices for this pixel's 3x3
             * neighborhood. */
            const int columnLeft  = reflectIndex(column - 1, width_);
            const int columnRight = reflectIndex(column + 1, width_);

            /* Sample all eight neighbors (the center pixel itself does
             * not appear in either Sobel kernel). */
            const auto topLeft =
                static_cast<float>(image_in.at(rowAbove, columnLeft));
            const auto topCenter =
                static_cast<float>(image_in.at(rowAbove, column));
            const auto topRight =
                static_cast<float>(image_in.at(rowAbove, columnRight));
            const auto middleLeft =
                static_cast<float>(image_in.at(row, columnLeft));
            const auto middleRight =
                static_cast<float>(image_in.at(row, columnRight));
            const auto bottomLeft =
                static_cast<float>(image_in.at(rowBelow, columnLeft));
            const auto bottomCenter =
                static_cast<float>(image_in.at(rowBelow, column));
            const auto bottomRight =
                static_cast<float>(image_in.at(rowBelow, columnRight));

            /*!
             * Horizontal Sobel kernel:
             *   [-1  0  1]
             *   [-2  0  2]
             *   [-1  0  1]
             * gx/gy match the reference algorithm's own Ix/Iy gradient
             * notation, standard in this domain.
             */
            // NOLINTNEXTLINE(readability-identifier-length)
            const float gx = (topRight + (2.0F * middleRight) + bottomRight) -
                             (topLeft + (2.0F * middleLeft) + bottomLeft);

            /*!
             * Vertical Sobel kernel:
             *   [-1 -2 -1]
             *   [ 0  0  0]
             *   [ 1  2  1]
             */
            // NOLINTNEXTLINE(readability-identifier-length)
            const float gy =
                (bottomLeft + (2.0F * bottomCenter) + bottomRight) -
                (topLeft + (2.0F * topCenter) + topRight);

            /* Row-major offset shared by both gradient buffers. */
            const std::size_t offset = (static_cast<std::size_t>(row) *
                                        static_cast<std::size_t>(width_)) +
                                       static_cast<std::size_t>(column);

            /* Store this pixel's horizontal gradient. */
            gradientX_[offset] = gx;

            /* Store this pixel's vertical gradient. */
            gradientY_[offset] = gy;
        }
    }
}

} /* namespace localisation::visual_odometry::feature_tracking */
