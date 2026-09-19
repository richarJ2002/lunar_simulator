/*!
 * @File:         ContinuousExtendedKalmanFilter.h
 *
 * @Brief:        Declares a reusable, model-agnostic continuous-discrete
 *                EKF math engine.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_EKF_H
#define LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_EKF_H

/* Function Includes */
/* None */

/* Object Include */
#include "objects/FilterStatus.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
/* None */

namespace localisation::kalman_filter::ekf_continuous_kalman_filter
{

/*!
 * @brief           Executes continuous-time state/covariance prediction and
 *                   discrete Joseph-form measurement correction for an
 *                   arbitrary, runtime-sized nonlinear state-space model.
 *
 * This engine holds no knowledge of what any state or measurement means: the
 * caller evaluates its own process model (state derivative and Jacobian) and
 * observation model (observation matrix) at the engine's current state --
 * read via getState() -- and hands the resulting matrices to predict() and
 * update(). The engine performs only the matrix algebra: Euler integration
 * of the continuous covariance (Riccati) equation for predict(), and a
 * Joseph-form correction for update(). It does not track wall/sim time,
 * bound integration step size, or wrap any state component (e.g. an angle)
 * back into a canonical range -- all of that is the caller's model-specific
 * responsibility, exercised by calling predict() repeatedly with a bounded
 * dt and by calling setState() after any wrapping.
 *
 * State and covariance are sized once by initialize() and are not resized
 * afterward; every subsequent call is dimension-checked against that size.
 * The object owns no resources beyond its own dynamically-sized Eigen
 * matrices and performs no allocation once initialized (predict()/update()
 * only allocate the local intermediates their math requires). It is not
 * thread-safe; its owner shall serialize lifecycle calls.
 */
class ContinuousExtendedKalmanFilter
{
  public:
    /*!
     * @brief           Creates a valid, uninitialized filter.
     */
    ContinuousExtendedKalmanFilter() noexcept = default;

    /*! @brief Releases the filter without external side effects. */
    ~ContinuousExtendedKalmanFilter() noexcept = default;

    ContinuousExtendedKalmanFilter(
        const ContinuousExtendedKalmanFilter &otherFilter_in) = delete;
    ContinuousExtendedKalmanFilter &
    operator=(const ContinuousExtendedKalmanFilter &otherFilter_in) = delete;
    ContinuousExtendedKalmanFilter(
        ContinuousExtendedKalmanFilter &&otherFilter_in) = delete;
    ContinuousExtendedKalmanFilter &
    operator=(ContinuousExtendedKalmanFilter &&otherFilter_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Sizes and seeds the filter's state and covariance.
     *
     * @param[in]       stateSize_in
     *                  Number of scalar states; must be positive.
     * @param[in]       initialState_in
     *                  Initial state, of length stateSize_in.
     * @param[in]       initialCovariance_in
     *                  Initial state covariance, stateSize_in x
     *                  stateSize_in.
     * @return          Lifecycle status.
     * @post            A successful call permits predict(), update(),
     *                  getState(), getCovariance(), setState() and
     *                  terminate() calls.
     */
    [[nodiscard]] FilterStatus
    initialize(Eigen::Index stateSize_in,
              const Eigen::VectorXd &initialState_in,
              const Eigen::MatrixXd &initialCovariance_in) noexcept;

    /*!
     * @brief           Advances state and covariance by one bounded
     *                   continuous-time step using a caller-evaluated
     *                   linearization of the process model at the current
     *                   state.
     *
     * @param[in]       dtS_in
     *                  Non-negative step duration in seconds. The caller is
     *                  responsible for bounding this (e.g. to a few tens of
     *                  milliseconds) so its own nonlinear model's
     *                  linearization stays valid across the step, and for
     *                  calling this repeatedly to cover a longer gap,
     *                  re-evaluating stateDerivative_in/processJacobian_in
     *                  at the new state (via getState()) before each call.
     * @param[in]       stateDerivative_in
     *                  f(x) evaluated at the current state, length
     *                  stateSize.
     * @param[in]       processJacobian_in
     *                  F = df/dx evaluated at the current state, stateSize
     *                  x stateSize.
     * @param[in]       processNoise_in
     *                  Continuous process-noise density, stateSize x
     *                  stateSize, in state units squared per second.
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] FilterStatus
    predict(double dtS_in, const Eigen::VectorXd &stateDerivative_in,
           const Eigen::MatrixXd &processJacobian_in,
           const Eigen::MatrixXd &processNoise_in) noexcept;

    /*!
     * @brief           Applies one discrete Joseph-form measurement
     *                   correction using a caller-evaluated observation
     *                   model.
     *
     * @param[in]       innovation_in
     *                  Measurement minus predicted measurement (z - h(x)),
     *                  already wrapped/normalized by the caller where the
     *                  observed quantity requires it (e.g. an angle); length
     *                  m.
     * @param[in]       observationMatrix_in
     *                  H = dh/dx evaluated at the current state, m x
     *                  stateSize.
     * @param[in]       measurementNoise_in
     *                  Measurement-noise covariance, m x m.
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] FilterStatus
    update(const Eigen::VectorXd &innovation_in,
          const Eigen::MatrixXd &observationMatrix_in,
          const Eigen::MatrixXd &measurementNoise_in) noexcept;

    /*!
     * @brief           Clears state and returns to the uninitialized
     *                   lifecycle, freeing the dynamically-sized matrices.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] FilterStatus terminate() noexcept;

    /*!
     * @brief           Returns the current state.
     *
     * @return          Current state, empty (size 0) if never initialized.
     */
    [[nodiscard]] const Eigen::VectorXd &getState() const noexcept;

    /*!
     * @brief           Returns the current state covariance.
     *
     * @return          Current covariance, empty if never initialized.
     */
    [[nodiscard]] const Eigen::MatrixXd &getCovariance() const noexcept;

    /*!
     * @brief           Overwrites the current state, e.g. after the caller
     *                   wraps an angular component into its canonical
     *                   range.
     *
     * @param[in]       state_in
     *                  Replacement state, length stateSize.
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] FilterStatus setState(const Eigen::VectorXd &state_in) noexcept;

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Current state estimate.
     */
    Eigen::VectorXd state;

    /*!
     * @brief       Current state covariance.
     */
    Eigen::MatrixXd covariance;

    /*!
     * @brief       Configured number of scalar states; 0 until initialize()
     *              succeeds.
     */
    Eigen::Index stateSize{0};

    /*!
     * @brief       True after a successful initialize() call.
     */
    bool isInitialized{false};
};

} /* namespace localisation::kalman_filter::ekf_continuous_kalman_filter */

#endif /* LUNAR_SIMULATOR_LOCALISATION_EKF_CONTINUOUS_KALMAN_FILTER_CONTINUOUS_EKF_H */
