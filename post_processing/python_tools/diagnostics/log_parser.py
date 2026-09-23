"""!
@brief  Parses the periodic "visual_diag" (visual_odometry) and
        "localisation_diag" (alpha_kalman_filter) five-second diagnostic
        log records out of a node's ROS log file.

        Every parsed record's `log_relative_time_s` is elapsed seconds
        since this log file's own first parsed record -- NOT the same axis
        as a bag-derived plot's elapsed simulation seconds. RCLCPP_INFO
        stamps its own console/file output in wall-clock time regardless of
        a node's `use_sim_time` setting, and a `--use-sim-time` recording
        (as `scripts/launch_simulator.sh`'s recorder uses) makes the bag's
        own header stamps *and* its receive timestamps both simulated time,
        so there is no reliable wall-clock<->simulated-time anchor anywhere
        in a captured run to convert one axis into the other. An earlier
        version of this module attempted to derive such a conversion from
        (header stamp - bag receive time); that offset is invalid under
        `--use-sim-time` (both operands are already simulated time) and
        produced diagnostic timestamps off by the wall-clock epoch (see the
        plan's 2026-09-22 audit). Every caller must display
        `log_relative_time_s` on its own explicitly-labeled axis and must
        not overlay it on a bag-elapsed-time plot. Pure text parsing; no
        ROS or bag access here.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Optional, Sequence

from python_tools.data.models import (
    DiagnosticLog,
    LocalisationDiagnosticRecord,
    VisualDiagnosticRecord,
)

# Shared x-axis title for any figure plotting a diagnostic record's
# log_relative_time_s, so every caller states the same explicit caveat
# rather than letting a figure imply this is the bag's elapsed-time axis.
DIAGNOSTIC_TIME_AXIS_TITLE = "Time since first log record (s) - not bag-elapsed time"

# One rclcpp log line's fixed prefix: "[LEVEL] [sec.nanosec] [node]: message".
_LOG_LINE_PATTERN = re.compile(
    r"^\[(?P<level>\w+)\] \[(?P<timestamp>\d+\.\d+)\] \[(?P<node>[^\]]+)\]: (?P<message>.*)$"
)

# Matches a plain decimal or %e-style scientific-notation float, covering
# every numeric field format used by both diagnostic log lines.
_FLOAT = r"[-+]?\d+\.\d+(?:[eE][-+]?\d+)?"

_VISUAL_DIAG_PATTERN = re.compile(
    r"^visual_diag "
    rf"received=(?P<received>\d+) accepted=(?P<accepted>\d+) failed=(?P<failed>\d+) "
    rf"reception_rate_hz=(?P<reception_rate_hz>{_FLOAT}) "
    rf"accepted_rate_hz=(?P<accepted_rate_hz>{_FLOAT}) age_s=(?P<age_s>{_FLOAT}) "
    rf"processing_ms=(?P<processing_ms>{_FLOAT}) conversion_ms=(?P<conversion_ms>{_FLOAT}) "
    rf"disparity_ms=(?P<disparity_ms>{_FLOAT}) detection_ms=(?P<detection_ms>{_FLOAT}) "
    rf"tracking_ms=(?P<tracking_ms>{_FLOAT}) reconstruction_ms=(?P<reconstruction_ms>{_FLOAT}) "
    rf"pnp_ms=(?P<pnp_ms>{_FLOAT}) "
    r"detected=(?P<detected>\d+) tracked=(?P<tracked>\d+) stereo_valid=(?P<stereo_valid>\d+) "
    r"correspondences=(?P<correspondences>\d+) inliers=(?P<inliers>\d+) "
    rf"occupancy=(?P<occupancy>{_FLOAT}) near_mid_ratio=(?P<near_mid_ratio>{_FLOAT}) "
    rf"disparity_p10_p50_p90=\[(?P<disparity_p10>{_FLOAT}),(?P<disparity_p50>{_FLOAT}),(?P<disparity_p90>{_FLOAT})\] "
    rf"depth_p10_p50_p90_m=\[(?P<depth_p10>{_FLOAT}),(?P<depth_p50>{_FLOAT}),(?P<depth_p90>{_FLOAT})\] "
    rf"reprojection_rms_px=(?P<reprojection_rms_px>{_FLOAT}) "
    rf"normal_condition=(?P<normal_condition>{_FLOAT}) "
    rf"accepted_interval_s=(?P<accepted_interval_s>{_FLOAT}) "
    r"consecutive_failures=(?P<consecutive_failures>\d+)$"
)

# One-time (not periodic) marker: inertial_odometry's handleImuCallBack.cc
# logs this exactly once, the instant its startup stationary calibration
# window completes.
_INERTIAL_CALIBRATION_COMPLETE_PATTERN = re.compile(
    r"^IMU stationary calibration complete \((?P<sample_count>\d+) samples\)$"
)

_LOCALISATION_DIAG_PATTERN = re.compile(
    r"^localisation_diag source=(?P<source>\w+) "
    r"received=(?P<received>\d+) accepted=(?P<accepted>\d+) "
    r"age_rejected=(?P<age_rejected>\d+) nis_rejected=(?P<nis_rejected>\d+) "
    r"numerical_rejected=(?P<numerical_rejected>\d+) fused=(?P<fused>\d+) "
    rf"publication=(?P<publication>{_FLOAT}) admission=(?P<admission>{_FLOAT}) "
    rf"start=(?P<start>{_FLOAT}) end=(?P<end>{_FLOAT}) "
    rf"estimator_epoch=(?P<estimator_epoch>{_FLOAT}) nis=(?P<nis>{_FLOAT}) "
    rf"correction_norm=(?P<correction_norm>{_FLOAT}) "
    rf"covariance_trace=(?P<covariance_trace>{_FLOAT}) "
    rf"covariance_min_eigenvalue=(?P<covariance_min_eigenvalue>{_FLOAT}) "
    rf"covariance_diagonal_range=\[(?P<covariance_diagonal_min>{_FLOAT}),(?P<covariance_diagonal_max>{_FLOAT})\] "
    rf"quaternion_norm=(?P<quaternion_norm>{_FLOAT}) "
    rf"accel_bias_body_mps2=\[(?P<accel_bias_x>{_FLOAT}),(?P<accel_bias_y>{_FLOAT}),(?P<accel_bias_z>{_FLOAT})\] "
    rf"gyro_bias_body_radps=\[(?P<gyro_bias_x>{_FLOAT}),(?P<gyro_bias_y>{_FLOAT}),(?P<gyro_bias_z>{_FLOAT})\]$"
)


def _log_relative_elapsed_s(
    log_wall_clock_s: float, first_log_wall_clock_s: float
) -> float:
    """!
    @brief   Converts one log line's wall-clock timestamp to elapsed
             seconds since this log file's own first parsed record.

    @param   log_wall_clock_s
             The log line's own wall-clock timestamp, seconds since epoch.
    @param   first_log_wall_clock_s
             The log file's own first line's wall-clock timestamp.

    @return  Elapsed wall-clock seconds since the log file's own first
             line. This preserves ordering and real-time spacing between
             diagnostic records, but is NOT on the same axis as a
             bag-derived plot's elapsed simulation seconds -- see this
             module's docstring for why no such conversion is possible.
    """
    return log_wall_clock_s - first_log_wall_clock_s


def parse_log_file(log_path: Path) -> DiagnosticLog:
    """!
    @brief   Parses every visual_diag/localisation_diag record out of one
             node's ROS log file.

    @param   log_path
             Path to the node's log file (e.g.
             `<run>/ros/logs/alpha_node_*.log`).

    @return  The assembled `DiagnosticLog`, whose records'
             `log_relative_time_s` is relative to this log file's own
             first parsed record (see this module's docstring).
    """
    visual_records: list[VisualDiagnosticRecord] = []
    localisation_records: list[LocalisationDiagnosticRecord] = []
    unparsed_line_count = 0
    first_log_wall_clock_s: Optional[float] = None

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            line_match = _LOG_LINE_PATTERN.match(line.rstrip("\n"))
            if line_match is None:
                continue
            log_wall_clock_s = float(line_match.group("timestamp"))
            if first_log_wall_clock_s is None:
                first_log_wall_clock_s = log_wall_clock_s
            message = line_match.group("message")

            # Only attempt the expensive diagnostic-specific regexes on a
            # line whose message actually starts with one of the tags.
            if message.startswith("visual_diag "):
                record_match = _VISUAL_DIAG_PATTERN.match(message)
                if record_match is None:
                    unparsed_line_count += 1
                    continue
                visual_records.append(
                    _build_visual_record(
                        record_match,
                        _log_relative_elapsed_s(log_wall_clock_s, first_log_wall_clock_s),
                    )
                )
            elif message.startswith("localisation_diag "):
                record_match = _LOCALISATION_DIAG_PATTERN.match(message)
                if record_match is None:
                    unparsed_line_count += 1
                    continue
                localisation_records.append(
                    _build_localisation_record(
                        record_match,
                        _log_relative_elapsed_s(log_wall_clock_s, first_log_wall_clock_s),
                    )
                )

    return DiagnosticLog(
        log_path=log_path,
        visual_records=tuple(visual_records),
        localisation_records=tuple(localisation_records),
        unparsed_line_count=unparsed_line_count,
    )


def _build_visual_record(
    match: re.Match, log_relative_time_s: float
) -> VisualDiagnosticRecord:
    """!
    @brief   Builds a `VisualDiagnosticRecord` from a matched regex group
             dict.

    @param   match
             A successful `_VISUAL_DIAG_PATTERN` match.
    @param   log_relative_time_s
             The record's elapsed time since the log file's own first
             record, seconds.

    @return  The assembled `VisualDiagnosticRecord`.
    """
    g = match.groupdict()
    return VisualDiagnosticRecord(
        log_relative_time_s=log_relative_time_s,
        received=int(g["received"]),
        accepted=int(g["accepted"]),
        failed=int(g["failed"]),
        reception_rate_hz=float(g["reception_rate_hz"]),
        accepted_rate_hz=float(g["accepted_rate_hz"]),
        age_s=float(g["age_s"]),
        processing_ms=float(g["processing_ms"]),
        conversion_ms=float(g["conversion_ms"]),
        disparity_ms=float(g["disparity_ms"]),
        detection_ms=float(g["detection_ms"]),
        tracking_ms=float(g["tracking_ms"]),
        reconstruction_ms=float(g["reconstruction_ms"]),
        pnp_ms=float(g["pnp_ms"]),
        detected=int(g["detected"]),
        tracked=int(g["tracked"]),
        stereo_valid=int(g["stereo_valid"]),
        correspondences=int(g["correspondences"]),
        inliers=int(g["inliers"]),
        occupancy=float(g["occupancy"]),
        near_mid_ratio=float(g["near_mid_ratio"]),
        disparity_p10_px=float(g["disparity_p10"]),
        disparity_p50_px=float(g["disparity_p50"]),
        disparity_p90_px=float(g["disparity_p90"]),
        depth_p10_m=float(g["depth_p10"]),
        depth_p50_m=float(g["depth_p50"]),
        depth_p90_m=float(g["depth_p90"]),
        reprojection_rms_px=float(g["reprojection_rms_px"]),
        normal_condition=float(g["normal_condition"]),
        accepted_interval_s=float(g["accepted_interval_s"]),
        consecutive_failures=int(g["consecutive_failures"]),
    )


def _build_localisation_record(
    match: re.Match, log_relative_time_s: float
) -> LocalisationDiagnosticRecord:
    """!
    @brief   Builds a `LocalisationDiagnosticRecord` from a matched regex
             group dict.

    @param   match
             A successful `_LOCALISATION_DIAG_PATTERN` match.
    @param   log_relative_time_s
             The record's elapsed time since the log file's own first
             record, seconds.

    @return  The assembled `LocalisationDiagnosticRecord`.
    """
    g = match.groupdict()
    return LocalisationDiagnosticRecord(
        log_relative_time_s=log_relative_time_s,
        source=g["source"],
        received=int(g["received"]),
        accepted=int(g["accepted"]),
        age_rejected=int(g["age_rejected"]),
        nis_rejected=int(g["nis_rejected"]),
        numerical_rejected=int(g["numerical_rejected"]),
        fused=int(g["fused"]),
        publication=float(g["publication"]),
        admission=float(g["admission"]),
        start=float(g["start"]),
        end=float(g["end"]),
        estimator_epoch=float(g["estimator_epoch"]),
        nis=float(g["nis"]),
        correction_norm=float(g["correction_norm"]),
        covariance_trace=float(g["covariance_trace"]),
        covariance_min_eigenvalue=float(g["covariance_min_eigenvalue"]),
        covariance_diagonal_min=float(g["covariance_diagonal_min"]),
        covariance_diagonal_max=float(g["covariance_diagonal_max"]),
        quaternion_norm=float(g["quaternion_norm"]),
        accel_bias_body_mps2=(
            float(g["accel_bias_x"]),
            float(g["accel_bias_y"]),
            float(g["accel_bias_z"]),
        ),
        gyro_bias_body_radps=(
            float(g["gyro_bias_x"]),
            float(g["gyro_bias_y"]),
            float(g["gyro_bias_z"]),
        ),
    )


def find_inertial_calibration_complete_time_s(log_path: Path) -> Optional[float]:
    """!
    @brief   Finds inertial_odometry's one-time stationary-calibration-
             complete marker in a node's log file.

    @param   log_path
             Path to the node's log file.

    @return  The marker's elapsed time since the log file's own first
             parsed record, seconds (see this module's docstring for why
             this is not the bag's elapsed-time axis), or `None` if the
             log file carries no such marker (calibration never completed,
             or inertial_odometry did not run).
    """
    first_log_wall_clock_s: Optional[float] = None
    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            line_match = _LOG_LINE_PATTERN.match(line.rstrip("\n"))
            if line_match is None:
                continue
            log_wall_clock_s = float(line_match.group("timestamp"))
            if first_log_wall_clock_s is None:
                first_log_wall_clock_s = log_wall_clock_s
            if _INERTIAL_CALIBRATION_COMPLETE_PATTERN.match(line_match.group("message")):
                return _log_relative_elapsed_s(log_wall_clock_s, first_log_wall_clock_s)
    return None


def find_latest_log(logs_dir: Path, node_name_prefix: str) -> Optional[Path]:
    """!
    @brief   Finds the most recently modified log file for a node in a
             run's `ros/logs/` directory, matching the
             `<node_name_prefix>_<pid>_<timestamp>.log` naming
             `rclcpp`/`ROS_LOG_DIR` produces.

    @param   logs_dir
             The run's `ros/logs/` directory.
    @param   node_name_prefix
             The node executable/name prefix to match (e.g. "alpha_node").

    @return  The matching log path with the latest modification time, or
             `None` if no matching log file exists.
    """
    if not logs_dir.is_dir():
        return None
    candidates = sorted(
        logs_dir.glob(f"{node_name_prefix}_*.log"),
        key=lambda path: path.stat().st_mtime,
    )
    return candidates[-1] if candidates else None


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone log-summary mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Print counts of parsed visual_diag/localisation_diag records in a log file."
    )
    parser.add_argument("log_path", type=Path, help="Path to a node's ROS log file.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints parsed diagnostic record counts for a
             log file.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    log = parse_log_file(args.log_path)
    print(f"visual_diag records: {len(log.visual_records)}")
    print(f"localisation_diag records: {len(log.localisation_records)}")
    print(f"unparsed tagged lines: {log.unparsed_line_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
