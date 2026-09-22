/**
 * @file            ErrorStateIndex.h
 *
 * @brief           Declares indices for Alpha's multiplicative EKF error state.
 *
 * @date            20/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_ERROR_STATE_INDEX_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_ERROR_STATE_INDEX_H

/* C++ Standard Library Includes */
#include <cstdint>

/* C Standard Library Includes */
/* None */

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Object Includes */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/**
 * @brief           Identifies each component of the Euclidean error state.
 */
enum class ErrorStateIndex : std::uint8_t
{
    /**
     * @brief           Index of map-frame x-position error.
     */
    ERROR_STATE_INDEX_POSITION_X = 0U,

    /**
     * @brief           Index of map-frame y-position error.
     */
    ERROR_STATE_INDEX_POSITION_Y = 1U,

    /**
     * @brief           Index of map-frame z-position error.
     */
    ERROR_STATE_INDEX_POSITION_Z = 2U,

    /**
     * @brief           Index of body-frame x attitude error.
     */
    ERROR_STATE_INDEX_ATTITUDE_X = 3U,

    /**
     * @brief           Index of body-frame y attitude error.
     */
    ERROR_STATE_INDEX_ATTITUDE_Y = 4U,

    /**
     * @brief           Index of body-frame z attitude error.
     */
    ERROR_STATE_INDEX_ATTITUDE_Z = 5U,

    /**
     * @brief           Index of fixed-frame x linear-velocity error.
     */
    ERROR_STATE_INDEX_LINEAR_VELOCITY_X = 6U,

    /**
     * @brief           Index of fixed-frame y linear-velocity error.
     */
    ERROR_STATE_INDEX_LINEAR_VELOCITY_Y = 7U,

    /**
     * @brief           Index of fixed-frame z linear-velocity error.
     */
    ERROR_STATE_INDEX_LINEAR_VELOCITY_Z = 8U,

    /**
     * @brief           Index of body-frame x accelerometer-bias error.
     */
    ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X = 9U,

    /**
     * @brief           Index of body-frame y accelerometer-bias error.
     */
    ERROR_STATE_INDEX_ACCELEROMETER_BIAS_Y = 10U,

    /**
     * @brief           Index of body-frame z accelerometer-bias error.
     */
    ERROR_STATE_INDEX_ACCELEROMETER_BIAS_Z = 11U,

    /**
     * @brief           Index of body-frame x gyroscope-bias error.
     */
    ERROR_STATE_INDEX_GYROSCOPE_BIAS_X = 12U,

    /**
     * @brief           Index of body-frame y gyroscope-bias error.
     */
    ERROR_STATE_INDEX_GYROSCOPE_BIAS_Y = 13U,

    /**
     * @brief           Index of body-frame z gyroscope-bias error.
     */
    ERROR_STATE_INDEX_GYROSCOPE_BIAS_Z = 14U,

    /**
     * @brief           Number of scalar components in the error state.
     */
    ERROR_STATE_INDEX_COUNT = 15U
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_ERROR_STATE_INDEX_H \
        */
