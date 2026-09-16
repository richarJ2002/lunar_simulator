/*!
 * @File:         ContinuousExtendedKalmanFilter.h
 *
 * @Brief:        Declares a reusable twelve-state continuous-discrete EKF.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_CONTINUOUS_EKF_H
#define LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_CONTINUOUS_EKF_H

/* Function Includes */
/* None */

/* Object Include */
#include "kalman_filter/objects/FilterStatus.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
#include <array>

namespace lunar_simulator::localisation::kalman_filter
{

/*!
 * @brief           Estimates pose and derivatives with continuous prediction.
 *
 * State order is position XYZ in the fixed frame, roll/pitch/yaw from fixed to
 * body, linear velocity XYZ in the fixed frame, then body angular rate XYZ.
 * The object owns all state and performs no allocation after initialization.
 * It is not thread-safe; its owner shall serialize lifecycle calls.
 */
class ContinuousExtendedKalmanFilter
{
  public:
    /*! Number of scalar states. */
    static constexpr Eigen::Index stateSize = 12;

    /*! Fixed-size state vector type. */
    using StateVector = Eigen::Matrix<double, stateSize, 1>;

    /*! Fixed-size state covariance and process-noise type. */
    using StateMatrix = Eigen::Matrix<double, stateSize, stateSize>;

    /*! Mask selecting directly observed states in one measurement. */
    using MeasurementMask = std::array<bool, stateSize>;

    /*!
     * @brief           Creates a valid, uninitialized filter.
     */
    ContinuousExtendedKalmanFilter() noexcept
        : state(StateVector::Zero()), covariance(StateMatrix::Identity()),
          processNoise(StateMatrix::Zero())
    {
    }

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

    /*!
     * @brief           Initializes or reinitializes the filter.
     * @param[in]       processNoise_in
     *                  Continuous process-noise density in state units squared
     *                  per second.
     * @param[in]       initialCovariance_in
     *                  Initial state covariance in squared state units.
     * @return          Lifecycle status.
     * @post            A successful call permits step and terminate calls.
     */
    [[nodiscard]] FilterStatus
    init(const StateMatrix &processNoise_in,
         const StateMatrix &initialCovariance_in) noexcept;

    /*!
     * @brief           Predicts to a time and optionally applies a measurement.
     * @param[in]       timestampS_in
     *                  Monotonic measurement or prediction time in seconds.
     * @param[in]       p_measurement_in
     *                  Optional borrowed direct state measurement. Null requests
     *                  prediction only.
     * @param[in]       measurementMask_in
     *                  States observed when p_measurement_in is non-null.
     * @param[in]       measurementVariances_in
     *                  Per-state positive measurement variances. Entries for
     *                  unobserved states are ignored.
     * @param[out]      state_out
     *                  Posterior state after this step.
     * @param[out]      covariance_out
     *                  Posterior covariance after this step.
     * @return          Lifecycle or numerical status.
     */
    [[nodiscard]] FilterStatus step(double timestampS_in,
                                    const StateVector *p_measurement_in,
                                    const MeasurementMask &measurementMask_in,
                                    const StateVector &measurementVariances_in,
                                    StateVector &state_out,
                                    StateMatrix &covariance_out) noexcept;

    /*!
     * @brief           Clears state and returns to the uninitialized lifecycle.
     * @return          Lifecycle status.
     */
    [[nodiscard]] FilterStatus terminate() noexcept;

  private:
    /*!
     * @brief           Propagates state and covariance to a target time.
     * @param[in]       targetTimestampS_in Target monotonic time in seconds.
     * @return          Operation status.
     */
    [[nodiscard]] FilterStatus predictTo(double targetTimestampS_in) noexcept;

    /*!
     * @brief           Applies one bounded continuous-model Euler step.
     * @param[in]       timeStepS_in Step duration in seconds.
     * @return          Operation status.
     */
    [[nodiscard]] FilterStatus predictStep(double timeStepS_in) noexcept;

    /*!
     * @brief           Applies one direct-observation Joseph-form update.
     * @param[in]       measurement_in Direct state measurement.
     * @param[in]       measurementMask_in Observed-state mask.
     * @param[in]       measurementVariances_in Per-state variances.
     * @return          Operation status.
     */
    [[nodiscard]] FilterStatus
    correct(const StateVector &measurement_in,
            const MeasurementMask &measurementMask_in,
            const StateVector &measurementVariances_in) noexcept;

    /*!
     * @brief           Wraps an angle to [-pi, pi].
     * @param[in]       angleRad_in Angle in radians.
     * @return          Wrapped angle in radians.
     */
    [[nodiscard]] static double wrapAngle(double angleRad_in) noexcept;

    /*! Posterior state in documented state order. */
    StateVector state;

    /*! Posterior state covariance. */
    StateMatrix covariance;

    /*! Continuous process-noise density. */
    StateMatrix processNoise;

    /*! Time of the posterior state in seconds. */
    double stateTimestampS{0.0};

    /*! True after a successful init call. */
    bool isInitialized{false};

    /*! True once the first step establishes a time origin. */
    bool hasTimestamp{false};
};

} /* namespace lunar_simulator::localisation::kalman_filter */

#endif /* LUNAR_SIMULATOR_LOCALISATION_KALMAN_FILTER_CONTINUOUS_EKF_H */
