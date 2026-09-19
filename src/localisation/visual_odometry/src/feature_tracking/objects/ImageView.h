/*!
 * @file            ImageView.h
 *
 * @brief           Declares a non-owning view over a single-channel 8-bit
 *                  image buffer, independent of any third-party image type.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_IMAGE_VIEW_H
#define LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_IMAGE_VIEW_H

/* C++ Standard Library Includes */
#include <cstddef>
#include <cstdint>

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief           Borrowed, non-owning view over one greyscale image.
 *
 * Deliberately independent of `cv::Mat` so neither feature-tracking
 * engine's public API depends on OpenCV; `VisualOdometryNode` constructs
 * one of these directly from a `cv::Mat`'s `data`/`cols`/`rows`/`step`
 * fields at its own boundary, without copying pixel data. The referenced
 * buffer must outlive every call that takes this view, and must be
 * single-channel 8-bit (`CV_8UC1` on the OpenCV side); neither is checked
 * here (see `at()` below).
 */
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
// ImageView is a deliberate plain aggregate (POD) view type with no
// invariants to protect -- matching Point2D's own design -- not a
// candidate for encapsulation.
struct ImageView
{
    /*!
     * @brief       Borrowed pointer to the first pixel of the first row.
     *              Never owned or freed by the viewer.
     */
    const std::uint8_t *p_pixels{nullptr};

    /*!
     * @brief       Image width, in pixels.
     */
    int width{0};

    /*!
     * @brief       Image height, in pixels.
     */
    int height{0};

    /*!
     * @brief       Number of bytes between the first pixel of one row and
     *              the first pixel of the next. May exceed `width` when
     *              the source buffer is row-padded (e.g. `cv::Mat::step`);
     *              never assumed equal to `width`.
     */
    std::size_t strideBytes{0U};

    /*!
     * @brief           Reads one pixel intensity.
     *
     * Precondition (unchecked; see `DEVIATION_LOG.md` DEV-FT-003): `row_in`
     * is in `[0, height)` and `column_in` is in `[0, width)`. Left
     * unchecked deliberately for hot-loop performance; every call site in
     * this module is structurally bounded (loop limits derived from
     * `width`/`height`, or a pre-validated feature position) rather than
     * relying on this accessor to reject an out-of-range index.
     *
     * @param[in]       row_in
     *                  Zero-based row index.
     * @param[in]       column_in
     *                  Zero-based column index.
     *
     * @return          The pixel intensity at `(row_in, column_in)`.
     */
    [[nodiscard]] std::uint8_t at(int row_in, int column_in) const noexcept
    {
        /* Row-major offset into the borrowed buffer, honoring stride
         * rather than assuming a contiguous, unpadded layout. */
        const std::size_t offset =
            (static_cast<std::size_t>(row_in) * strideBytes) +
            static_cast<std::size_t>(column_in);

        /* Unchecked pointer arithmetic is DEV-FT-003 (see
         * DEVIATION_LOG.md): this accessor's precondition, above, is the
         * caller's responsibility for hot-loop performance. */
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        return p_pixels[offset];
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_IMAGE_VIEW_H */
