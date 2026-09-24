/*!
 * @file            publishDiagnosticsCallBack.cc
 *
 * @brief           Implements periodic estimator diagnostics: a recorded
 *                  DiagnosticArray every simulated second and a compact
 *                  console health line every five.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* External Library Includes */
#include <Eigen/Eigenvalues>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "console/console.h"
#include "diagnostics/diagnostics.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

namespace
{

/*!
 * Builds one per-source status. The keys are the contract
 * post_processing/python_tools/diagnostics/bag_diagnostics.py parses.
 */
diagnostic_msgs::msg::DiagnosticStatus
    makeSourceStatus(const std::string       &sourceName_in,
                     const SourceDiagnostics &source_in)
{
    namespace diagnostics = common::diagnostics;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name    = "continuous_ekf/" + sourceName_in;
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = common::console::formatText(
        "%s fused %llu of %llu",
        sourceName_in.c_str(),
        static_cast<unsigned long long>(source_in.fusedCount),
        static_cast<unsigned long long>(source_in.receivedCount));
    diagnostics::addCountValue("received", source_in.receivedCount, status);
    diagnostics::addCountValue("accepted", source_in.acceptedCount, status);
    diagnostics::addCountValue("age_rejected",
                               source_in.ageRejectedCount,
                               status);
    diagnostics::addCountValue("nis_rejected",
                               source_in.nisRejectedCount,
                               status);
    diagnostics::addCountValue("numerical_rejected",
                               source_in.numericalRejectedCount,
                               status);
    diagnostics::addCountValue("fused", source_in.fusedCount, status);
    diagnostics::addRealValue("publication",
                              source_in.publicationTimestamp_s,
                              status);
    diagnostics::addRealValue("admission",
                              source_in.callbackAdmissionTimestamp_s,
                              status);
    diagnostics::addRealValue("start",
                              source_in.processingStartTimestamp_s,
                              status);
    diagnostics::addRealValue("end",
                              source_in.processingEndTimestamp_s,
                              status);
    diagnostics::addRealValue("estimator_epoch",
                              source_in.estimatorTimestamp_s,
                              status);
    diagnostics::addRealValue("nis",
                              source_in.normalizedInnovationSquared,
                              status);
    diagnostics::addRealValue("correction_norm",
                              source_in.correctionNorm,
                              status);
    diagnostics::addCountValue("pre_init",
                               source_in.preInitRejectedCount,
                               status);
    diagnostics::addCountValue("negative_age",
                               source_in.negativeAgeRejectedCount,
                               status);
    diagnostics::addCountValue("too_old",
                               source_in.tooOldRejectedCount,
                               status);
    diagnostics::addCountValue("state_gap",
                               source_in.stateGapRejectedCount,
                               status);
    diagnostics::addCountValue("rollback_failed",
                               source_in.rollbackFailedCount,
                               status);
    diagnostics::addCountValue("predict_failed",
                               source_in.predictFailedCount,
                               status);
    return status;
}

/*!
 * Counts every rejection of one source that the console line reports.
 */
std::uint64_t countRejections(const SourceDiagnostics &source_in)
{
    return source_in.ageRejectedCount + source_in.nisRejectedCount +
           source_in.numericalRejectedCount;
}

} /* anonymous namespace */

void AlphaKalmanFilterNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Filter-wide covariance health, evaluated only once a state exists. */
    double covarianceTrace             = 0.0;
    double minimumCovarianceEigenvalue = 0.0;
    double minimumCovarianceDiagonal   = 0.0;
    double maximumCovarianceDiagonal   = 0.0;
    if (hasInitialState && latestCovariance.allFinite())
    {
        covarianceTrace           = latestCovariance.trace();
        minimumCovarianceDiagonal = latestCovariance.diagonal().minCoeff();
        maximumCovarianceDiagonal = latestCovariance.diagonal().maxCoeff();
        const Eigen::SelfAdjointEigenSolver<ErrorStateMatrix> decomposition(
            0.5 * (latestCovariance + latestCovariance.transpose()),
            Eigen::EigenvaluesOnly);
        if (decomposition.info() == Eigen::Success)
        {
            minimumCovarianceEigenvalue =
                decomposition.eigenvalues().minCoeff();
        }
    }

    double          quaternionNorm    = 0.0;
    Eigen::Vector3d accelerometerBias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeBias     = Eigen::Vector3d::Zero();
    if (hasInitialState)
    {
        const Eigen::Index quaternionIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
        quaternionNorm    = latestState.segment<4>(quaternionIndex).norm();
        accelerometerBias = latestState.segment<3>(static_cast<Eigen::Index>(
            StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X));
        gyroscopeBias     = latestState.segment<3>(static_cast<Eigen::Index>(
            StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X));
    }

    /* Console deltas since the previous health line; also the headline of
     * the filter-wide status. */
    const std::uint64_t rejectedCount =
        countRejections(visualDiagnostics) + countRejections(wheelDiagnostics);
    const std::string healthLine =
        hasInitialState ? common::console::formatText(
                              "EKF fused v%llu w%llu rej %llu",
                              static_cast<unsigned long long>(
                                  visualDiagnostics.fusedCount -
                                  previousConsoleVisualFusedCount),
                              static_cast<unsigned long long>(
                                  wheelDiagnostics.fusedCount -
                                  previousConsoleWheelFusedCount),
                              static_cast<unsigned long long>(
                                  rejectedCount - previousConsoleRejectedCount))
                        : std::string("EKF waiting for IMU init");

    /* Filter-wide snapshot shared by every source's record. */
    diagnostic_msgs::msg::DiagnosticStatus filterStatus;
    filterStatus.name        = "continuous_ekf";
    filterStatus.hardware_id = baseFrame;
    filterStatus.level       = hasInitialState
                                   ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                   : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    filterStatus.message     = healthLine;
    diagnostics::addFlagValue("initialized", hasInitialState, filterStatus);
    diagnostics::addRealValue("covariance_trace",
                              covarianceTrace,
                              filterStatus);
    diagnostics::addRealValue("covariance_min_eigenvalue",
                              minimumCovarianceEigenvalue,
                              filterStatus);
    diagnostics::addRealValue("covariance_diagonal_min",
                              minimumCovarianceDiagonal,
                              filterStatus);
    diagnostics::addRealValue("covariance_diagonal_max",
                              maximumCovarianceDiagonal,
                              filterStatus);
    diagnostics::addRealValue("quaternion_norm", quaternionNorm, filterStatus);
    diagnostics::addRealValue("accel_bias_x_mps2",
                              accelerometerBias.x(),
                              filterStatus);
    diagnostics::addRealValue("accel_bias_y_mps2",
                              accelerometerBias.y(),
                              filterStatus);
    diagnostics::addRealValue("accel_bias_z_mps2",
                              accelerometerBias.z(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_x_radps",
                              gyroscopeBias.x(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_y_radps",
                              gyroscopeBias.y(),
                              filterStatus);
    diagnostics::addRealValue("gyro_bias_z_radps",
                              gyroscopeBias.z(),
                              filterStatus);

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = now();
    record.status.push_back(filterStatus);
    record.status.push_back(makeSourceStatus("imu", imuDiagnostics));
    record.status.push_back(makeSourceStatus("visual", visualDiagnostics));
    record.status.push_back(makeSourceStatus("wheel", wheelDiagnostics));
    p_diagnosticsPublisher->publish(record);
    ++diagnosticsRecordCount;

    /* The console carries only the short health line, every fifth record. */
    if (diagnosticsRecordCount % CONSOLE_HEALTH_PERIOD_TICKS != 0U)
    {
        return;
    }
    LUNAR_LOG_INFO(get_logger(), "%s", healthLine.c_str());
    previousConsoleVisualFusedCount = visualDiagnostics.fusedCount;
    previousConsoleWheelFusedCount  = wheelDiagnostics.fusedCount;
    previousConsoleRejectedCount    = rejectedCount;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
