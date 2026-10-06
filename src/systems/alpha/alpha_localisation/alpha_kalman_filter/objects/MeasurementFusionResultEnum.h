/*!
 * @file            MeasurementFusionResultEnum.h
 *
 * @brief           Declares the outcome of applying one odometry update.
 *
 * @date            25/09/2026
 */

#ifndef SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_FUSION_RESULT_ENUM_H
#define SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_FUSION_RESULT_ENUM_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Distinguishes a statistical rejection from a numerical
 *                  one, because the node counts them separately.
 */
enum class MeasurementFusionResult : std::uint8_t
{
    /*!
     * @brief           The update and error injection both succeeded.
     */
    MEASUREMENT_FUSION_RESULT_FUSED = 0U,

    /*!
     * @brief           The normalized innovation squared exceeded the gate;
     *                  the filter was not modified.
     */
    MEASUREMENT_FUSION_RESULT_NIS_REJECTED = 1U,

    /*!
     * @brief           The innovation covariance, update or injection
     *                  failed numerically.
     */
    MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED = 2U
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* SRS_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_FUSION_RESULT_ENUM_H \
        */
