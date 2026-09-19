/*!
 * @File:         MeasurementKind.h
 *
 * @Brief:        Declares which odometry source produced a measurement.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_ALPHA_MEASUREMENT_KIND_H
#define LUNAR_SIMULATOR_ALPHA_MEASUREMENT_KIND_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Identifies which upstream odometry topic produced one
 *                  measurement passed into
 *                  AlphaKalmanFilterNode::handleMeasurementCallBack.
 *
 * Each value selects a distinct EKF observation matrix and variance source;
 * see AlphaKalmanFilterNode.h for the corresponding fusion contract.
 */
enum class MeasurementKind : std::uint8_t
{
    /*!
     * @brief       Measurement originates from inertial_odometry.
     */
    MEASUREMENT_KIND_INERTIAL = 0U,
    /*!
     * @brief       Measurement originates from visual_odometry.
     */
    MEASUREMENT_KIND_VISUAL = 1U,
    /*!
     * @brief       Measurement originates from wheel_odometry.
     */
    MEASUREMENT_KIND_WHEEL = 2U
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_ALPHA_MEASUREMENT_KIND_H */
