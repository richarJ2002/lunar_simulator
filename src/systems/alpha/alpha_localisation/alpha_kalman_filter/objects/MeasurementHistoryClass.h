/*!
 * @file            MeasurementHistoryClass.h
 *
 * @brief           Declares Alpha's fixed-capacity, stamp-ordered history of
 *                  fused odometry measurements.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_HISTORY_CLASS_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_HISTORY_CLASS_H

/* C++ Standard Library Includes */
#include <array>
#include <cstddef>
#include <optional>

/* C Standard Library Includes */
/* None */

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/FusedMeasurementStruct.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Retains recently fused measurements in timestamp order so
 *                  a rollback can re-apply everything after the delayed
 *                  measurement.
 *
 *                  The filter node owns one history and accesses it only
 *                  from its mutually exclusive callback group. Records are
 *                  copied into fixed storage, so no allocation occurs while
 *                  a replay is running. When capacity is reached the oldest
 *                  record is discarded; at the configured rates the
 *                  capacity spans more than twice the maximum measurement
 *                  age, so a discarded record is already older than any
 *                  rollback target.
 */
class MeasurementHistory
{
  public:
    /*!
     * @brief           Constructs an empty history.
     */
    MeasurementHistory() noexcept = default;

    /*!
     * @brief           Destroys the history without external effects.
     */
    ~MeasurementHistory() noexcept = default;

    MeasurementHistory(const MeasurementHistory &otherHistory_in) = delete;
    MeasurementHistory &
        operator=(const MeasurementHistory &otherHistory_in) = delete;
    MeasurementHistory(MeasurementHistory &&otherHistory_in) = delete;
    MeasurementHistory &
        operator=(MeasurementHistory &&otherHistory_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Inserts one record at its timestamp position.
     *
     *                  A record whose stamp equals retained stamps is
     *                  placed after them, preserving arrival order. When
     *                  the history is full, a record older than every
     *                  retained record is refused; otherwise the oldest
     *                  record is discarded to make room.
     *
     * @param[in]       record_in
     *                  Fused measurement with a finite timestamp.
     *
     * @return          True when the record was stored.
     */
    [[nodiscard]] bool insert(const FusedMeasurement &record_in) noexcept;

    /*!
     * @brief           Returns one record by chronological position.
     *
     * @param[in]       recordIndex_in
     *                  Zero-based index from the oldest retained record.
     *
     * @return          Requested record, or no value when the index is out
     *                  of range.
     */
    [[nodiscard]] std::optional<FusedMeasurement>
        getRecord(std::size_t recordIndex_in) const noexcept;

    /*!
     * @brief           Returns the number of retained records.
     *
     * @return          Number of records in chronological storage.
     */
    [[nodiscard]] std::size_t getRecordCount() const noexcept;

    /*!
     * @brief           Removes every retained record.
     */
    void clear() noexcept;

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Maximum number of retained records.
     *
     *                  Wheel odometry arrives at 50 Hz and visual odometry
     *                  at up to 10 Hz, so 128 records cover about two
     *                  seconds against a 0.75 s maximum measurement age.
     *
     * @frame           N/A
     * @units           records
     */
    static constexpr std::size_t CAPACITY = 128U;

    /*!
     * @brief           Fixed storage; the first recordCount entries are the
     *                  retained records in timestamp order.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::array<FusedMeasurement, CAPACITY> records{};

    /*!
     * @brief           Number of valid records currently retained.
     *
     * @frame           N/A
     * @units           records
     */
    std::size_t recordCount{0U};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_MEASUREMENT_HISTORY_CLASS_H \
        */
