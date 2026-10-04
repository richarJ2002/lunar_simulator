/*!
 * @file            FeatureTrackingLimits.h
 *
 * @brief           Declares the compile-time capacity ceilings shared by
 *                  every feature-tracking engine.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_LIMITS_H
#define LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_LIMITS_H

/* C++ Standard Library Includes */
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief       Hard upper bound on the number of features either engine may
 *              report in one call. Every fixed-size output buffer in this
 *              module is sized to this constant so no engine ever needs a
 *              `std::vector` or reallocation in its public API.
 */
inline constexpr std::size_t MAXIMUM_SUPPORTED_FEATURES = 1024U;

/*!
 * @brief       Sanity ceiling on the image width an engine may be
 *              `initialize()`d with, in pixels. Guards against a
 *              misconfigured `image_width_px` ROS parameter driving an
 *              unbounded internal allocation.
 */
inline constexpr int MAXIMUM_SUPPORTED_WIDTH_PX = 4096;

/*!
 * @brief       Sanity ceiling on the image height an engine may be
 *              `initialize()`d with, in pixels. See
 *              `MAXIMUM_SUPPORTED_WIDTH_PX` for the rationale.
 */
inline constexpr int MAXIMUM_SUPPORTED_HEIGHT_PX = 4096;

/*!
 * @brief       Sanity ceiling on the number of Gaussian pyramid levels
 *              `PyramidalLucasKanadeTracker` may be configured with.
 */
inline constexpr int MAXIMUM_SUPPORTED_PYRAMID_LEVELS = 8;

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_LIMITS_H */
