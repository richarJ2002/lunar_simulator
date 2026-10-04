/*!
 * @File:         sampleBilinear.cc
 *
 * @Brief:        Implements bounded bilinear sub-pixel sampling of an
 *                arbitrary row-major float buffer.
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
#include <cmath>
#include <cstddef>
#include <vector>

namespace localisation::visual_odometry::feature_tracking
{

bool PyramidalLucasKanadeTracker::sampleBilinear(
    const std::vector<float> &buffer_in,
    int                       width_in,
    int                       height_in,
    Point2D                   position_in,
    float                    &value_out) noexcept
{
    /* The bilinear footprint spans the 2x2 pixel block whose top-left
     * corner is the sampled position's floor. */
    const float floorX = std::floor(position_in.x);
    const float floorY = std::floor(position_in.y);
    const int   lowX   = static_cast<int>(floorX);
    const int   lowY   = static_cast<int>(floorY);
    const int   highX  = lowX + 1;
    const int   highY  = lowY + 1;

    /*!
     * Per this tracker's no-extrapolation boundary policy (see
     * `DEVIATION_LOG.md` DEV-FT-004), a footprint that would touch a
     * pixel outside `[0, width_in) x [0, height_in)` is rejected rather
     * than fabricated by mirroring or clamping.
     */
    if (lowX < 0 || lowY < 0 || highX >= width_in || highY >= height_in)
    {
        value_out = 0.0F;
        return false;
    }

    /* Fractional offset within the 2x2 footprint. */
    const float fractionX = position_in.x - floorX;
    const float fractionY = position_in.y - floorY;

    const auto widthSizeT = static_cast<std::size_t>(width_in);

    /* The four surrounding pixel values. */
    const float topLeft =
        buffer_in[(static_cast<std::size_t>(lowY) * widthSizeT) +
                  static_cast<std::size_t>(lowX)];
    const float topRight =
        buffer_in[(static_cast<std::size_t>(lowY) * widthSizeT) +
                  static_cast<std::size_t>(highX)];
    const float bottomLeft =
        buffer_in[(static_cast<std::size_t>(highY) * widthSizeT) +
                  static_cast<std::size_t>(lowX)];
    const float bottomRight =
        buffer_in[(static_cast<std::size_t>(highY) * widthSizeT) +
                  static_cast<std::size_t>(highX)];

    /* Interpolate along rows first, then between the two row results. */
    const float top    = topLeft + (fractionX * (topRight - topLeft));
    const float bottom = bottomLeft + (fractionX * (bottomRight - bottomLeft));
    value_out          = top + (fractionY * (bottom - top));

    return true;
}

} /* namespace localisation::visual_odometry::feature_tracking */
