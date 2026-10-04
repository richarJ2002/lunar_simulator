/*!
 * @file            SourceDiagnosticsStruct.h
 *
 * @brief           Declares bounded estimator source diagnostics.
 *
 * @date            21/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_STRUCT_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_STRUCT_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Accumulates one source's counts and latest timing values.
 */
struct SourceDiagnostics
{
  public:
    /*!
     * @brief           Number of callbacks admitted.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t receivedCount{0U};

    /*!
     * @brief           Number of measurements passing boundary and age
     *                  validation.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t acceptedCount{0U};

    /*!
     * @brief           Number of measurements rejected for age or timestamp
     *                  order.
     *
     *                  For the visual and wheel sources this is always the
     *                  sum of the six reason-specific counters below, kept
     *                  so existing consumers still see one total. The IMU
     *                  source instead counts backwards clock jumps here.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t ageRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected before the
     *                  IMU-seeded initial state existed.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t preInitRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  led this node's ROS clock by more than
     *                  maximum_future_stamp_s.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t negativeAgeRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  was older than the maximum measurement age.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t tooOldRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because their stamp
     *                  was too far from the estimator epoch.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t stateGapRejectedCount{0U};

    /*!
     * @brief           Number of measurements rejected because no rollback
     *                  checkpoint could be restored and replayed to their
     *                  stamp.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t rollbackFailedCount{0U};

    /*!
     * @brief           Number of measurements rejected because forward
     *                  prediction to their stamp failed.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t predictFailedCount{0U};

    /*!
     * @brief           Number of measurements rejected by a statistical gate.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t nisRejectedCount{0U};

    /*!
     * @brief           Number of malformed or numerically rejected
     *                  measurements.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t numericalRejectedCount{0U};

    /*!
     * @brief           Number of measurements successfully applied to the
     *                  estimate.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t fusedCount{0U};

    /*!
     * @brief           Number of already-fused measurements re-applied after
     *                  an older measurement caused a rollback.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t replayedCount{0U};

    /*!
     * @brief           Number of replayed measurements that the statistical
     *                  gate rejected against the rolled-back state; they
     *                  stay retained for later replays.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t replayRejectedCount{0U};

    /*!
     * @brief           Number of IMU samples held as single-sample contact
     *                  shocks; used by the IMU source only.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t shockHeldCount{0U};

    /*!
     * @brief           Latest source publication timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double publicationTimestamp_s{0.0};

    /*!
     * @brief           Latest callback admission timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double callbackAdmissionTimestamp_s{0.0};

    /*!
     * @brief           Latest processing-start timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double processingStartTimestamp_s{0.0};

    /*!
     * @brief           Latest callback completion timestamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double processingEndTimestamp_s{0.0};

    /*!
     * @brief           Estimator epoch after the latest callback.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double estimatorTimestamp_s{0.0};

    /*!
     * @brief           Latest normalized innovation squared value.
     *
     * @frame           N/A
     * @units           dimensionless
     */
    double normalizedInnovationSquared{0.0};

    /*!
     * @brief           Norm of the latest posterior error-state correction.
     *
     * @frame           Mixed; see ErrorStateIndex
     * @units           Mixed; see ErrorStateIndex
     */
    double correctionNorm{0.0};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_STRUCT_H */
