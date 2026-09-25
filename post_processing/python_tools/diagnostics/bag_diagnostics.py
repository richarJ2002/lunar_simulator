"""!
@brief  Converts the recorded /alpha/diagnostics DiagnosticArray statuses
        (WP-01 Phase 1 onward) into the same VisualDiagnosticRecord and
        LocalisationDiagnosticRecord types the legacy log parser produces,
        but timed in bag-elapsed simulation seconds so the report can plot
        them on the same axis as every other bag-derived series.

        The status names and keys below are the contract with the C++
        publishers: visual_odometry's publishPipelineDiagnostics.cc and
        alpha_kalman_filter's publishDiagnostics.cc. Pure conversion; no
        ROS or bag access here.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

from python_tools.data.models import (
    AgeRejectionBreakdown,
    DiagnosticLog,
    DiagnosticStatusSample,
    DiagnosticStatusSeries,
    DiagnosticTimeBasis,
    LocalisationDiagnosticRecord,
    VisualDiagnosticRecord,
)
from python_tools.diagnostics.log_parser import DIAGNOSTIC_TIME_AXIS_TITLE

# Topic every project node publishes its DiagnosticArray on.
DIAGNOSTICS_TOPIC = "/alpha/diagnostics"

# Status published once per tick by visual_odometry.
VISUAL_STATUS_NAME = "visual_odometry"

# Topic the start-up supervisor publishes its latched state on.
SYSTEM_STATE_TOPIC = "/alpha/system/state"

# The supervisor's status name on SYSTEM_STATE_TOPIC.
SYSTEM_STATE_STATUS_NAME = "system_state"

# Status carrying the driver's command-gate counters on DIAGNOSTICS_TOPIC.
DRIVER_STATUS_NAME = "alpha_driver_node"

# Calibration status published once per tick by inertial_odometry.
INERTIAL_STATUS_NAME = "inertial_odometry"

# Filter-wide status published once per tick by continuous_ekf.
FILTER_STATUS_NAME = "continuous_ekf"

# Per-source estimator statuses are named "continuous_ekf/<source>".
SOURCE_STATUS_PREFIX = FILTER_STATUS_NAME + "/"

# x-axis title for plots of bag-sourced diagnostic records.
BAG_DIAGNOSTIC_TIME_AXIS_TITLE = "Elapsed simulation time (s)"


class _MissingDiagnosticValueError(ValueError):
    """!
    @brief  Raised internally when a status lacks a required key or holds a
            value that is not a number, so the caller can count the status
            as malformed rather than crash.
    """


def _integer(values: dict[str, str], key: str) -> int:
    """!
    @brief   Reads one required integer value.

    @param   values
             A status's key/value text.
    @param   key
             The required key.

    @return  The parsed integer.

    @throws  _MissingDiagnosticValueError
             If the key is missing or not an integer.
    """
    # Both a missing key and malformed text make the status unusable.
    try:
        return int(values[key])
    except (KeyError, ValueError) as error:
        raise _MissingDiagnosticValueError(key) from error


def _real(values: dict[str, str], key: str) -> float:
    """!
    @brief   Reads one required floating-point value.

    @param   values
             A status's key/value text.
    @param   key
             The required key.

    @return  The parsed value.

    @throws  _MissingDiagnosticValueError
             If the key is missing or not a number.
    """
    # Both a missing key and malformed text make the status unusable.
    try:
        return float(values[key])
    except (KeyError, ValueError) as error:
        raise _MissingDiagnosticValueError(key) from error


def _visual_record(sample: DiagnosticStatusSample) -> VisualDiagnosticRecord:
    """!
    @brief   Builds one visual record from a "visual_odometry" status.

    @param   sample
             The recorded status.

    @return  The record, timed in bag-elapsed seconds.

    @throws  _MissingDiagnosticValueError
             If any required key is missing or malformed.
    """
    # Short alias keeps the long field list readable.
    values = sample.values
    return VisualDiagnosticRecord(
        time_s=sample.time_s,
        received=_integer(values, "received"),
        accepted=_integer(values, "accepted"),
        failed=_integer(values, "failed"),
        reception_rate_hz=_real(values, "reception_rate_hz"),
        accepted_rate_hz=_real(values, "accepted_rate_hz"),
        age_s=_real(values, "age_s"),
        processing_ms=_real(values, "processing_ms"),
        conversion_ms=_real(values, "conversion_ms"),
        disparity_ms=_real(values, "disparity_ms"),
        detection_ms=_real(values, "detection_ms"),
        tracking_ms=_real(values, "tracking_ms"),
        reconstruction_ms=_real(values, "reconstruction_ms"),
        pnp_ms=_real(values, "pnp_ms"),
        detected=_integer(values, "detected"),
        tracked=_integer(values, "tracked"),
        stereo_valid=_integer(values, "stereo_valid"),
        correspondences=_integer(values, "correspondences"),
        inliers=_integer(values, "inliers"),
        occupancy=_real(values, "occupancy"),
        near_mid_ratio=_real(values, "near_mid_ratio"),
        disparity_p10_px=_real(values, "disparity_p10_px"),
        disparity_p50_px=_real(values, "disparity_p50_px"),
        disparity_p90_px=_real(values, "disparity_p90_px"),
        depth_p10_m=_real(values, "depth_p10_m"),
        depth_p50_m=_real(values, "depth_p50_m"),
        depth_p90_m=_real(values, "depth_p90_m"),
        reprojection_rms_px=_real(values, "reprojection_rms_px"),
        normal_condition=_real(values, "normal_condition"),
        accepted_interval_s=_real(values, "accepted_interval_s"),
        consecutive_failures=_integer(values, "consecutive_failures"),
    )


def _localisation_record(
    source_sample: DiagnosticStatusSample, filter_sample: DiagnosticStatusSample
) -> LocalisationDiagnosticRecord:
    """!
    @brief   Builds one estimator record by joining a per-source status with
             the same tick's filter-wide status.

    @param   source_sample
             A "continuous_ekf/<source>" status.
    @param   filter_sample
             The "continuous_ekf" status published in the same array.

    @return  The record, timed in bag-elapsed seconds.

    @throws  _MissingDiagnosticValueError
             If any required key is missing or malformed.
    """
    # Per-source counters and timing, then the shared filter snapshot.
    source = source_sample.values
    shared = filter_sample.values
    return LocalisationDiagnosticRecord(
        time_s=source_sample.time_s,
        source=source_sample.name[len(SOURCE_STATUS_PREFIX):],
        received=_integer(source, "received"),
        accepted=_integer(source, "accepted"),
        age_rejected=_integer(source, "age_rejected"),
        nis_rejected=_integer(source, "nis_rejected"),
        numerical_rejected=_integer(source, "numerical_rejected"),
        fused=_integer(source, "fused"),
        publication=_real(source, "publication"),
        admission=_real(source, "admission"),
        start=_real(source, "start"),
        end=_real(source, "end"),
        estimator_epoch=_real(source, "estimator_epoch"),
        nis=_real(source, "nis"),
        correction_norm=_real(source, "correction_norm"),
        covariance_trace=_real(shared, "covariance_trace"),
        covariance_min_eigenvalue=_real(shared, "covariance_min_eigenvalue"),
        covariance_diagonal_min=_real(shared, "covariance_diagonal_min"),
        covariance_diagonal_max=_real(shared, "covariance_diagonal_max"),
        quaternion_norm=_real(shared, "quaternion_norm"),
        accel_bias_body_mps2=(
            _real(shared, "accel_bias_x_mps2"),
            _real(shared, "accel_bias_y_mps2"),
            _real(shared, "accel_bias_z_mps2"),
        ),
        gyro_bias_body_radps=(
            _real(shared, "gyro_bias_x_radps"),
            _real(shared, "gyro_bias_y_radps"),
            _real(shared, "gyro_bias_z_radps"),
        ),
        age_rejection_breakdown=AgeRejectionBreakdown(
            pre_init=_integer(source, "pre_init"),
            negative_age=_integer(source, "negative_age"),
            too_old=_integer(source, "too_old"),
            state_gap=_integer(source, "state_gap"),
            rollback_failed=_integer(source, "rollback_failed"),
            predict_failed=_integer(source, "predict_failed"),
        ),
    )


def diagnostic_log_from_bag(series: DiagnosticStatusSeries, bag_path: Path) -> DiagnosticLog:
    """!
    @brief   Converts every recorded pipeline and estimator status into a
             bag-elapsed DiagnosticLog.

    Statuses from other nodes (readiness, wheel solve, supervisor) are
    ignored here; they are read directly by the pages that need them.

    @param   series
             The recorded /alpha/diagnostics statuses, time-sorted.
    @param   bag_path
             The bag the statuses came from, recorded as the log's source.

    @return  The assembled log. `unparsed_line_count` counts pipeline or
             estimator statuses that lacked a required key.
    """
    visual_records: list[VisualDiagnosticRecord] = []
    localisation_records: list[LocalisationDiagnosticRecord] = []
    malformed_count = 0
    # Filter-wide status of each tick, looked up by the array's stamp.
    filter_samples = {
        sample.time_ns: sample for sample in series.samples if sample.name == FILTER_STATUS_NAME
    }
    for sample in series.samples:
        try:
            if sample.name == VISUAL_STATUS_NAME:
                visual_records.append(_visual_record(sample))
            elif sample.name.startswith(SOURCE_STATUS_PREFIX):
                # A source status without its tick's filter snapshot is
                # incomplete, exactly like a missing key.
                filter_sample = filter_samples.get(sample.time_ns)
                if filter_sample is None:
                    raise _MissingDiagnosticValueError(FILTER_STATUS_NAME)
                localisation_records.append(_localisation_record(sample, filter_sample))
        except _MissingDiagnosticValueError:
            malformed_count += 1
    return DiagnosticLog(
        log_path=bag_path,
        visual_records=tuple(visual_records),
        localisation_records=tuple(localisation_records),
        unparsed_line_count=malformed_count,
        time_basis=DiagnosticTimeBasis.BAG_ELAPSED,
    )


def inertial_calibration_complete_elapsed_s(
    series: DiagnosticStatusSeries, start_time_ns: int
) -> Optional[float]:
    """!
    @brief   Finds when inertial_odometry's stationary calibration completed,
             on the bag's elapsed-simulation-time axis.

    @param   series
             The recorded /alpha/diagnostics statuses, time-sorted.
    @param   start_time_ns
             The bag's elapsed-zero reference, nanoseconds.

    @return  Elapsed seconds of the calibrating IMU sample's stamp, or
             `None` if no recorded status reports calibration complete.
    """
    # The first calibrated status carries the stamp of the completing sample.
    for sample in series.samples:
        if sample.name != INERTIAL_STATUS_NAME or sample.values.get("calibrated") != "true":
            continue
        try:
            complete_stamp_s = float(sample.values["calibration_complete_stamp_s"])
        except (KeyError, ValueError):
            return None
        # Convert the ROS-seconds stamp onto the bag's elapsed axis.
        return complete_stamp_s - start_time_ns / 1e9
    return None


def system_state_transitions(
    series: DiagnosticStatusSeries,
) -> list[tuple[float, str, str]]:
    """!
    @brief   Extracts every change of the supervisor's system state.

    @param   series
             The recorded /alpha/system/state statuses, time-sorted.

    @return  `(elapsed_s, state, message)` for the first state and each
             later change, in time order; heartbeats that repeat the
             current state are omitted.
    """
    transitions: list[tuple[float, str, str]] = []
    # Keep only samples whose state differs from the previous one.
    for sample in series.samples:
        state = sample.values.get("state")
        if sample.name != SYSTEM_STATE_STATUS_NAME or state is None:
            continue
        if not transitions or transitions[-1][1] != state:
            transitions.append((sample.time_s, state, sample.message))
    return transitions


def latest_status_values(series: DiagnosticStatusSeries, status_name: str) -> dict[str, str]:
    """!
    @brief   Returns the key/values of the latest status with a given name.

    @param   series
             Recorded diagnostics, time-sorted.
    @param   status_name
             Status name, e.g. "alpha_driver_node".

    @return  The latest matching status's values, or an empty dict.
    """
    # Walk backwards so the first match is the latest.
    for sample in reversed(series.samples):
        if sample.name == status_name:
            return sample.values
    return {}


def diagnostic_time_axis_title(log: DiagnosticLog) -> str:
    """!
    @brief   Chooses the x-axis title matching a DiagnosticLog's clock.

    @param   log
             The diagnostic log whose records are being plotted.

    @return  The bag-elapsed title for recorded diagnostics, or the
             log-relative title carrying its "not bag-elapsed" caveat.
    """
    # Only recorded diagnostics share the bag's elapsed-time axis.
    if log.time_basis is DiagnosticTimeBasis.BAG_ELAPSED:
        return BAG_DIAGNOSTIC_TIME_AXIS_TITLE
    return DIAGNOSTIC_TIME_AXIS_TITLE


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints how many pipeline and estimator records
             a test run's recorded diagnostics contain.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success, `1` if the run has no
             recorded diagnostics.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    # Imported here so the pure conversion above never needs rosbag2_py.
    from python_tools.bag.reader import discover_bag_path, read_bag

    bag_path = discover_bag_path(args.test_run, None)
    bag = read_bag(bag_path)
    series = bag.diagnostic_arrays.get(DIAGNOSTICS_TOPIC)
    # Runs before WP-01 Phase 1 have no recorded diagnostics topic.
    if series is None:
        print(f"{DIAGNOSTICS_TOPIC} was not recorded in {bag_path}")
        return 1
    log = diagnostic_log_from_bag(series, bag_path)
    print(f"visual records: {len(log.visual_records)}")
    print(f"localisation records: {len(log.localisation_records)}")
    print(f"malformed statuses: {log.unparsed_line_count}")
    return 0


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Summarise a test run's recorded /alpha/diagnostics records."
    )
    parser.add_argument("test_run", type=Path, help="Captured test run directory.")
    return parser


if __name__ == "__main__":
    raise SystemExit(main())
