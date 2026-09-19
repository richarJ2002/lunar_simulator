/*!
 * @File:         terminate.cc
 *
 * @Brief:        Terminates the continuous extended Kalman filter lifecycle.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

FilterStatus ContinuousExtendedKalmanFilter::terminate() noexcept
{
    /*!
     * Return every field to its just-constructed value, freeing the
     * dynamically-sized matrices, so the filter is indistinguishable from
     * a freshly created, uninitialized one.
     */
    state      = Eigen::VectorXd();
    covariance = Eigen::MatrixXd();
    stateSize  = 0;

    /*!
     * Only now that every field above is reset does the filter refuse
     * predict()/update() calls until initialize() is called again.
     */
    isInitialized = false;

    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */
