/**
 * @file            StateIndex.h
 *
 * @brief           Declares indices for Alpha's bias-aware nominal state.
 *
 * @date            20/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_STATE_INDEX_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_STATE_INDEX_H

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
 * @brief           Identifies each component of the nominal ESKF state.
 */
enum class StateIndex : std::uint8_t
{
    /**
     * @brief           Index of map-frame x position.
     */
    STATE_INDEX_POSITION_X = 0U,

    /**
     * @brief           Index of map-frame y position.
     */
    STATE_INDEX_POSITION_Y = 1U,

    /**
     * @brief           Index of map-frame z position.
     */
    STATE_INDEX_POSITION_Z = 2U,

    /**
     * @brief           Index of the body-to-map quaternion x coefficient.
     */
    STATE_INDEX_QUATERNION_X = 3U,

    /**
     * @brief           Index of the body-to-map quaternion y coefficient.
     */
    STATE_INDEX_QUATERNION_Y = 4U,

    /**
     * @brief           Index of the body-to-map quaternion z coefficient.
     */
    STATE_INDEX_QUATERNION_Z = 5U,

    /**
     * @brief           Index of the body-to-map quaternion w coefficient.
     */
    STATE_INDEX_QUATERNION_W = 6U,

    /**
     * @brief           Index of fixed-frame x linear velocity.
     */
    STATE_INDEX_LINEAR_VELOCITY_X = 7U,

    /**
     * @brief           Index of fixed-frame y linear velocity.
     */
    STATE_INDEX_LINEAR_VELOCITY_Y = 8U,

    /**
     * @brief           Index of fixed-frame z linear velocity.
     */
    STATE_INDEX_LINEAR_VELOCITY_Z = 9U,

    /**
     * @brief           Index of body-frame x accelerometer bias.
     */
    STATE_INDEX_ACCELEROMETER_BIAS_X = 10U,

    /**
     * @brief           Index of body-frame y accelerometer bias.
     */
    STATE_INDEX_ACCELEROMETER_BIAS_Y = 11U,

    /**
     * @brief           Index of body-frame z accelerometer bias.
     */
    STATE_INDEX_ACCELEROMETER_BIAS_Z = 12U,

    /**
     * @brief           Index of body-frame x gyroscope bias.
     */
    STATE_INDEX_GYROSCOPE_BIAS_X = 13U,

    /**
     * @brief           Index of body-frame y gyroscope bias.
     */
    STATE_INDEX_GYROSCOPE_BIAS_Y = 14U,

    /**
     * @brief           Index of body-frame z gyroscope bias.
     */
    STATE_INDEX_GYROSCOPE_BIAS_Z = 15U,

    /**
     * @brief           Number of scalar components in the nominal state.
     */
    STATE_INDEX_COUNT = 16U
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_STATE_INDEX_H \
        */
