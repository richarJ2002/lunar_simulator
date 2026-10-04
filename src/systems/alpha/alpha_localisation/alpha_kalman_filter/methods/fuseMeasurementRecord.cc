/*!
 * @file            fuseMeasurementRecord.cc
 *
 * @brief           Implements gating and application of one measurement
 *                  record at the current filter epoch.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

/* Object Includes */
#include "objects/ErrorStateIndexEnum.h"
#include "objects/StateIndexEnum.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

MeasurementFusionResult AlphaKalmanFilterNode::fuseMeasurementRecord(
    const FusedMeasurement &record_in,
    double                 &nis_out,
    double                 &correctionNorm_out)
{
    const Eigen::Index positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);

    Eigen::VectorXd innovation;
    Eigen::MatrixXd observation;
    Eigen::MatrixXd measurementNoise;
    if (record_in.isVisualIncrement)
    {
        Eigen::VectorXd predictedIncrement;
        calculateVisualIncrementObservation(nominalState,
                                            record_in.meanRawYawRate_radPs,
                                            record_in.hasYawRate,
                                            predictedIncrement,
                                            observation);
        const Eigen::Index rowCount = predictedIncrement.size();
        innovation =
            record_in.measuredValues.head(rowCount) - predictedIncrement;
        measurementNoise = record_in.noiseVariances.head(rowCount).asDiagonal();
    }
    else if (record_in.kind == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        const Eigen::Quaterniond measuredQuaternion_bodyToFixed(
            record_in.measuredValues(6),
            record_in.measuredValues(3),
            record_in.measuredValues(4),
            record_in.measuredValues(5));
        const Eigen::Quaterniond predictedQuaternion_bodyToFixed(
            nominalState(quaternionIndex + 3),
            nominalState(quaternionIndex),
            nominalState(quaternionIndex + 1),
            nominalState(quaternionIndex + 2));

        innovation               = Eigen::VectorXd::Zero(6);
        innovation.segment<3>(0) = record_in.measuredValues.head<3>() -
                                   nominalState.segment<3>(positionIndex);
        innovation.segment<3>(3) =
            calculateQuaternionError(measuredQuaternion_bodyToFixed,
                                     predictedQuaternion_bodyToFixed);
        observation = Eigen::MatrixXd::Zero(6, ERROR_STATE_SIZE);
        observation.block<3, 3>(0, errorPositionIndex) =
            Eigen::Matrix3d::Identity();
        observation.block<3, 3>(3, errorAttitudeIndex) =
            Eigen::Matrix3d::Identity();
        measurementNoise = record_in.noiseVariances.asDiagonal();
    }
    else
    {
        Eigen::Vector3d                predictedVelocity_body_mPs;
        WheelVelocityObservationMatrix wheelObservation;
        calculateWheelVelocityObservation(nominalState,
                                          predictedVelocity_body_mPs,
                                          wheelObservation);
        innovation =
            record_in.measuredValues.head<3>() - predictedVelocity_body_mPs;
        observation      = wheelObservation;
        measurementNoise = record_in.noiseVariances.head<3>().asDiagonal();
    }

    if (!calculateNormalizedInnovationSquared(innovation,
                                              observation,
                                              filter.getCovariance(),
                                              measurementNoise,
                                              nis_out))
    {
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }
    const double configuredNisThreshold =
        record_in.kind == MeasurementKind::MEASUREMENT_KIND_VISUAL
            ? visualNisThreshold
            : wheelNisThreshold;
    if (nis_out > selectNisThreshold(configuredNisThreshold, innovation.size()))
    {
        return MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_NIS_REJECTED;
    }

    const FilterStatus updateStatus =
        filter.update(innovation, observation, measurementNoise);
    if (updateStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        logStepFailure(updateStatus);
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }
    correctionNorm_out                 = filter.getState().norm();
    const FilterStatus injectionStatus = injectErrorState();
    if (injectionStatus != FilterStatus::FILTER_STATUS_SUCCESS)
    {
        logStepFailure(injectionStatus);
        return MeasurementFusionResult::
            MEASUREMENT_FUSION_RESULT_NUMERICAL_REJECTED;
    }
    return MeasurementFusionResult::MEASUREMENT_FUSION_RESULT_FUSED;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
