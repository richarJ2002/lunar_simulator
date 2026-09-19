/*!
 * @File:         FilterStatus.h
 *
 * @Brief:        Declares continuous EKF lifecycle result values.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_FILTER_STATUS_H
#define LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_FILTER_STATUS_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
#include <cstdint>

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

/*!
 * @brief           Reports the outcome of a filter lifecycle operation.
 */
enum class FilterStatus : std::uint8_t
{
    /*!
     * @brief       Operation completed successfully.
     */
    FILTER_STATUS_SUCCESS = 0U,
    /*!
     * @brief       Configuration contains a non-positive state size, a
     *              dimension mismatch, or non-finite/invalid covariance
     *              values.
     */
    FILTER_STATUS_INVALID_CONFIGURATION = 1U,
    /*!
     * @brief       Call input contains a dimension mismatch against the
     *              configured state size, or non-finite/invalid values.
     */
    FILTER_STATUS_INVALID_INPUT = 2U,
    /*!
     * @brief       Operation was requested before the filter was
     *              initialized.
     */
    FILTER_STATUS_NOT_INITIALIZED = 3U,
    /*!
     * @brief       Matrix operations produced a non-finite result.
     */
    FILTER_STATUS_NUMERICAL_FAILURE = 4U
};

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */

#endif /* LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_FILTER_STATUS_H */
