/*!
 * @File:         FilterStatus.h
 *
 * @Brief:        Declares continuous EKF lifecycle result values.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_FILTER_STATUS_H
#define LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_FILTER_STATUS_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace lunar_simulator::localisation::kalman_filter
{

/*!
 * @brief           Reports the outcome of a filter lifecycle operation.
 */
enum class FilterStatus : std::uint8_t
{
    /*! Operation completed successfully. */
    FILTER_STATUS_SUCCESS = 0U,
    /*! Configuration contains non-finite or invalid covariance values. */
    FILTER_STATUS_INVALID_CONFIGURATION = 1U,
    /*! Step input contains invalid time, masks, or measurement values. */
    FILTER_STATUS_INVALID_INPUT = 2U,
    /*! Step was requested before the filter was initialized. */
    FILTER_STATUS_NOT_INITIALIZED = 3U,
    /*! Matrix operations produced a non-finite result. */
    FILTER_STATUS_NUMERICAL_FAILURE = 4U
};

} /* namespace lunar_simulator::localisation::kalman_filter */

#endif /* LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_FILTER_STATUS_H */
