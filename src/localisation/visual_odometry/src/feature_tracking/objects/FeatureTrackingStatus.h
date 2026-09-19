/*!
 * @file            FeatureTrackingStatus.h
 *
 * @brief           Declares the shared lifecycle/operation result values
 *                  used by every feature-tracking engine in this module.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_STATUS_H
#define LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_STATUS_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief           Reports the outcome of a feature-tracking engine
 *                  lifecycle or operation call.
 *
 * Shared by `ShiTomasiCornerDetector` and `PyramidalLucasKanadeTracker`
 * rather than duplicated per engine, since both engines have identical
 * failure-mode shapes (lifecycle and input validation only; neither engine
 * performs the kind of numerical solve that can diverge the way the EKF's
 * `FilterStatus::NUMERICAL_FAILURE` covers).
 */
enum class FeatureTrackingStatus : std::uint8_t
{
    /*!
     * @brief       Operation completed successfully.
     */
    FEATURE_TRACKING_STATUS_SUCCESS = 0U,
    /*!
     * @brief       The operation was requested before a successful
     *              `initialize()` call.
     */
    FEATURE_TRACKING_STATUS_NOT_INITIALIZED = 1U,
    /*!
     * @brief       `initialize()` was called with a configuration outside
     *              its valid or supported range.
     */
    FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION = 2U,
    /*!
     * @brief       An operation call's input does not match the
     *              configuration `initialize()` established, or is
     *              otherwise invalid.
     */
    FEATURE_TRACKING_STATUS_INVALID_INPUT = 3U
};

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_STATUS_H */
