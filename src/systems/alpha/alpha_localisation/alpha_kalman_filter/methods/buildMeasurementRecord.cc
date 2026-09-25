/*!
 * @file            buildMeasurementRecord.cc
 *
 * @brief           Implements conversion of an odometry message into a
 *                  replayable measurement record.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* C++ Standard Library Includes */
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

/* Object Includes */
#include "objects/ImuSample.h"
#include "objects/StateIndex.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

bool AlphaKalmanFilterNode::buildMeasurementRecord(
    const nav_msgs::msg::Odometry &message_in,
    MeasurementKind                kind_in,
    double                         measurementTimestampS_in,
    double                         intervalStartS_in,
    FusedMeasurement              &record_out) const
{
    FusedMeasurement record;
    record.kind        = kind_in;
    record.timestamp_s = measurementTimestampS_in;
    record.isVisualIncrement =
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL &&
        isVisualIncrementMode;
    const MeasurementVarianceVector variances = measurementVariances(
        message_in,
        kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL ? visualVariance
                                                            : wheelVariance);

    if (record.isVisualIncrement)
    {
        record.measuredValues.head<4>() =
            Eigen::Vector4d(message_in.twist.twist.linear.x,
                            message_in.twist.twist.linear.y,
                            message_in.twist.twist.linear.z,
                            message_in.twist.twist.angular.z);

        /* The yaw-rate row compares the increment with the mean raw gyro
         * over the same interval; capture it now so a replay does not need
         * the IMU samples to still be buffered. */
        double      rawYawRateSum_radPerS = 0.0;
        std::size_t rawYawRateCount       = 0U;
        for (std::size_t sampleIndex = 0U;
             sampleIndex < imuBuffer.getSampleCount();
             ++sampleIndex)
        {
            const std::optional<ImuSample> sample =
                imuBuffer.getSample(sampleIndex);
            if (sample.has_value() && sample->timestamp_s > intervalStartS_in &&
                sample->timestamp_s <= measurementTimestampS_in)
            {
                rawYawRateSum_radPerS +=
                    sample->angularVelocity_body_radPerS.z();
                ++rawYawRateCount;
            }
        }
        record.hasYawRate = rawYawRateCount > 0U;
        record.meanRawYawRate_radPerS =
            record.hasYawRate
                ? rawYawRateSum_radPerS / static_cast<double>(rawYawRateCount)
                : 0.0;

        /* Increment noise comes from VO's own twist covariance, floored;
         * the pose variance floor does not apply to a velocity. */
        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            const double reported =
                message_in.twist
                    .covariance[static_cast<std::size_t>(axis * 6 + axis)];
            record.noiseVariances(axis) =
                std::isfinite(reported)
                    ? std::max(visualIncrementVelocityVariance, reported)
                    : visualIncrementVelocityVariance;
        }
        const double reportedYawRate = message_in.twist.covariance[35];
        record.noiseVariances(3) =
            std::isfinite(reportedYawRate)
                ? std::max(visualIncrementYawRateVariance, reportedYawRate)
                : visualIncrementYawRateVariance;
    }
    else if (kind_in == MeasurementKind::MEASUREMENT_KIND_VISUAL)
    {
        const NominalStateVector measurement = odometryToState(message_in);
        const Eigen::Index       positionIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
        const Eigen::Index quaternionIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
        record.measuredValues.head<3>() = measurement.segment<3>(positionIndex);
        record.measuredValues.segment<4>(3) =
            measurement.segment<4>(quaternionIndex);

        /* Roll and pitch use the configured attitude variance; visual
         * odometry only reports a meaningful yaw variance. */
        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            record.noiseVariances(axis) = variances(axis);
            record.noiseVariances(axis + 3) =
                axis < 2 ? visualAttitudeVariance : variances(axis + 3);
        }
    }
    else
    {
        record.measuredValues.head<3>() =
            Eigen::Vector3d(message_in.twist.twist.linear.x,
                            message_in.twist.twist.linear.y,
                            message_in.twist.twist.linear.z);
        record.noiseVariances.head<3>() = variances.segment<3>(6);
    }

    if (!record.measuredValues.allFinite())
    {
        return false;
    }
    record_out = record;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
