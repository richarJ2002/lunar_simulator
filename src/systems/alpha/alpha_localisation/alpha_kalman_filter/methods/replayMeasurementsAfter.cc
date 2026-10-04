/*!
 * @file            replayMeasurementsAfter.cc
 *
 * @brief           Implements replay of fused measurements after a rollback.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

/* C++ Standard Library Includes */
#include <cstddef>
#include <optional>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

AlphaKalmanFilterNode::FilterStatus
    AlphaKalmanFilterNode::replayMeasurementsAfter(
        double rollbackTimestamp_s_in, double presentTimestamp_s_in)
{
    /* Records at the rollback epoch itself were fused before the restored
     * checkpoint was saved, so only strictly later ones are re-applied. */
    constexpr double TIMESTAMP_TOLERANCE_S = 1.0e-9;
    for (std::size_t recordIndex = 0U;
         recordIndex < measurementHistory.getRecordCount();
         recordIndex++)
    {
        const std::optional<FusedMeasurement> record =
            measurementHistory.getRecord(recordIndex);
        if (!record.has_value())
        {
            return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
        }
        if (record->timestamp_s <=
            rollbackTimestamp_s_in + TIMESTAMP_TOLERANCE_S)
        {
            continue;
        }

        const FilterStatus predictionStatus = predictTo(record->timestamp_s);
        if (predictionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
        {
            return predictionStatus;
        }

        SourceDiagnostics &diagnostics =
            record->kind == MeasurementKind::MEASUREMENT_KIND_VISUAL
                ? visualDiagnostics
                : wheelDiagnostics;
        double                        replayNis            = 0.0;
        double                        replayCorrectionNorm = 0.0;
        const MeasurementFusionResult fusionResult =
            fuseMeasurementRecord(*record, replayNis, replayCorrectionNorm);
        if (fusionResult == MeasurementFusionResult::
                                MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED)
        {
            return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
        }
        if (fusionResult ==
            MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_NIS_REJECTED)
        {
            diagnostics.replayRejectedCount++;
        }
        else
        {
            diagnostics.replayedCount++;
        }
        saveFilterCheckpoint();
    }
    return predictTo(presentTimestamp_s_in);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
