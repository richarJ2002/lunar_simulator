/*!
 * @file            publishDiagnosticsCallBack.cc
 *
 * @brief           Implements the periodic inertial calibration status.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "objects/InertialOdometryNode.h"

/* External Library Includes */
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

/* Other Project Module Includes */
#include "diagnostics/diagnostics.h"

namespace localisation::inertial_odometry
{

void InertialOdometryNode::publishDiagnosticsCallBack()
{
    namespace diagnostics = common::diagnostics;

    /* Calibration is complete once the full stationary window was seen. */
    const bool isCalibrated = calibrationSampleCount >= calibrationSampleTarget;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name        = "inertial_odometry";
    status.hardware_id = baseFrame;
    status.level   = isCalibrated ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                  : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = isCalibrated
                         ? std::string("IMU calibrated")
                         : common::console::formatText("IMU calibrating %d/%d",
                                                       calibrationSampleCount,
                                                       calibrationSampleTarget);
    /* Readiness contract read by the start-up supervisor: this node is
     * ready exactly when its stationary calibration is complete. */
    diagnostics::addFlagValue("ready", isCalibrated, status);
    diagnostics::addTextValue("reason", status.message, status);
    diagnostics::addFlagValue("calibrated", isCalibrated, status);
    diagnostics::addCountValue(
        "calibration_samples",
        static_cast<std::uint64_t>(calibrationSampleCount),
        status);
    diagnostics::addCountValue(
        "calibration_target",
        static_cast<std::uint64_t>(calibrationSampleTarget),
        status);
    if (isCalibrated)
    {
        diagnostics::addRealValue("calibration_complete_stamp_s",
                                  calibrationCompleteStampS,
                                  status);
    }

    diagnostic_msgs::msg::DiagnosticArray record;
    record.header.stamp = now();
    record.status.push_back(status);
    p_diagnosticsPublisher->publish(record);
}

} /* namespace localisation::inertial_odometry */
