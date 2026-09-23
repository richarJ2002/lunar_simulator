"""!
@brief  Generates the inertial-odometry subsystem report page: all three
        IMU stages, calibration status, and inertial odometry's pose/body
        twist compared against ground truth. Standalone entry point and
        importable `generate_inertial_odometry_report()`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from python_tools.context import build_common_parser, build_report_context
from python_tools.data import alignment, metrics
from python_tools.data.models import ReportContext, ReportPage
from python_tools.diagnostics.log_parser import find_inertial_calibration_complete_time_s
from python_tools.reporting import figures, html, style

TOPIC_GROUND_TRUTH = "/alpha/localisation/ground_truth/odometry"
TOPIC_ESTIMATE = "/alpha/localisation/inertial/odometry"
TOPIC_RAW_IMU = "/alpha/drivers/imu"
TOPIC_NOISY_IMU = "/alpha/imu"
TOPIC_FILTERED_IMU = "/alpha/localisation/inertial/filtered_imu"


def generate_inertial_odometry_report(context: ReportContext) -> ReportPage:
    """!
    @brief   Builds the inertial-odometry report page.

    @param   context
             The report context to build the page from.

    @return  The assembled `ReportPage`.
    """
    warnings: list[str] = []
    sections: list[str] = []
    summary_stats: dict[str, str] = {}

    params = context.parameters.get("inertial_odometry")
    if params is not None:
        p = params.parameters
        calibration_time_s = None
        if context.diagnostics.log_path.is_file():
            calibration_time_s = find_inertial_calibration_complete_time_s(
                context.diagnostics.log_path
            )
        calibration_status = (
            # Relative to the node log's own first record, not this page's
            # bag-elapsed-time axis -- see log_parser's module docstring.
            f"complete {calibration_time_s:.2f}s into the node log"
            if calibration_time_s is not None
            else "not observed in the node log"
        )
        sections.append(
            "<section class='plot-section'><h2>Calibration &amp; configuration</h2>"
            "<dl>"
            f"<dt>Calibration samples (target)</dt><dd>{p.get('calibration_samples', 'n/a')}</dd>"
            f"<dt>Calibration status</dt><dd>{html.escape(calibration_status)}</dd>"
            f"<dt>Low-pass cutoff</dt><dd>{p.get('low_pass_cutoff_hz', 'n/a')} Hz</dd>"
            f"<dt>Gravity</dt><dd>{p.get('gravity_mps2', 'n/a')} m/s^2</dd>"
            f"<dt>Remove gravity</dt><dd>{p.get('remove_gravity', 'n/a')}</dd>"
            f"<dt>Acceleration deadband</dt><dd>{p.get('acceleration_deadband_mps2', 'n/a')} m/s^2</dd>"
            f"<dt>Angular rate deadband</dt><dd>{p.get('angular_rate_deadband_radps', 'n/a')} rad/s</dd>"
            "</dl></section>"
        )
    else:
        warnings.append("inertial_odometry parameter snapshot not found")

    raw_imu = context.bag.imu.get(TOPIC_RAW_IMU)
    noisy_imu = context.bag.imu.get(TOPIC_NOISY_IMU)
    filtered_imu = context.bag.imu.get(TOPIC_FILTERED_IMU)

    # Raw and noisy specific force (gravity still present) are directly
    # comparable and shown together; the filtered, gravity-free stage is
    # kept in its own clearly labeled plot per the plan.
    specific_force_series = []
    if raw_imu is not None:
        specific_force_series.append(
            ("Raw (Gazebo)", raw_imu.linear_acceleration_mps2, style.COLOR_RAW_SOURCE)
        )
    if noisy_imu is not None:
        specific_force_series.append(
            ("Noisy (driver)", noisy_imu.linear_acceleration_mps2, style.COLOR_PRIMARY_ESTIMATE)
        )
    if specific_force_series:
        reference_times = raw_imu.times_s if raw_imu is not None else noisy_imu.times_s
        # Only overlay stages that share the same sample times as the
        # reference (raw and noisy IMU publish at slightly different
        # counts in a real run -- see the sibling angular_rate_series
        # block below, whose same guard this was missing).
        figure = figures.three_axis_time_series(
            reference_times,
            [
                (label, values, color)
                for label, values, color in specific_force_series
                if values.shape[0] == reference_times.shape[0]
            ],
            ("X", "Y", "Z"),
            "m/s^2",
        )
        sections.append(
            "<section class='plot-section'><h2>Specific force (raw &amp; noisy, gravity present)</h2>"
            f"{html.figure_to_fragment(figure, 'inertial-specific-force')}</section>"
        )
    else:
        warnings.append(f"{TOPIC_RAW_IMU} and {TOPIC_NOISY_IMU} published no messages")

    if filtered_imu is not None:
        figure = figures.three_axis_time_series(
            filtered_imu.times_s,
            [("Filtered (bias-corrected, gravity-free)", filtered_imu.linear_acceleration_mps2, style.COLOR_TERTIARY_ESTIMATE)],
            ("X", "Y", "Z"),
            "m/s^2",
        )
        sections.append(
            "<section class='plot-section'><h2>Filtered acceleration (low-pass, gravity removed)</h2>"
            f"{html.figure_to_fragment(figure, 'inertial-filtered-accel')}</section>"
        )
    else:
        warnings.append(f"{TOPIC_FILTERED_IMU} published no messages")

    # Angular rate frame IDs agree across all three stages, so they can be
    # overlaid directly.
    angular_rate_series = []
    for label, imu_series, color in (
        ("Raw (Gazebo)", raw_imu, style.COLOR_RAW_SOURCE),
        ("Noisy (driver)", noisy_imu, style.COLOR_PRIMARY_ESTIMATE),
        ("Filtered", filtered_imu, style.COLOR_TERTIARY_ESTIMATE),
    ):
        if imu_series is not None:
            angular_rate_series.append((label, imu_series.angular_velocity_radps, color))
    if angular_rate_series:
        reference_times = (
            raw_imu.times_s
            if raw_imu is not None
            else noisy_imu.times_s if noisy_imu is not None else filtered_imu.times_s
        )
        # Only overlay stages that share the same sample times as the
        # reference; a mismatched-length series is plotted on its own axis
        # by three_axis_time_series's own x argument instead.
        figure = figures.three_axis_time_series(
            reference_times,
            [
                (label, values, color)
                for label, values, color in angular_rate_series
                if values.shape[0] == reference_times.shape[0]
            ],
            ("X", "Y", "Z"),
            "rad/s",
        )
        sections.append(
            "<section class='plot-section'><h2>Angular rate (all stages)</h2>"
            f"{html.figure_to_fragment(figure, 'inertial-angular-rate')}</section>"
        )

    # Pose/twist comparison against ground truth.
    truth = context.bag.odometry.get(TOPIC_GROUND_TRUTH)
    estimate = context.bag.odometry.get(TOPIC_ESTIMATE)
    if truth is not None and estimate is not None:
        position, orientation, linear_velocity, angular_velocity, valid = (
            alignment.interpolate_ground_truth(
                estimate.times_s, truth, context.maximum_alignment_gap_s
            )
        )
        position_error = metrics.position_error_by_axis(
            estimate.position_m, position, valid
        )
        attitude_error = metrics.attitude_error_deg(
            estimate.orientation_xyzw, orientation, valid
        )
        horizontal_error = metrics.horizontal_norm(np.nan_to_num(position_error, nan=0.0))
        horizontal_error = np.where(valid, horizontal_error, np.nan)
        norm_3d_error = metrics.norm_3d(np.nan_to_num(position_error, nan=0.0))
        norm_3d_error = np.where(valid, norm_3d_error, np.nan)

        position_summary = metrics.summarize_errors(norm_3d_error)
        attitude_summary = metrics.summarize_errors(attitude_error)
        summary_stats["3-D position RMSE"] = f"{position_summary.rmse:.3f} m"
        summary_stats["Attitude RMSE"] = f"{attitude_summary.rmse:.3f} deg"
        summary_stats["Sample rate"] = (
            f"{metrics.sample_rate_hz(estimate.times_s):.1f} Hz"
            if metrics.sample_rate_hz(estimate.times_s)
            else "n/a"
        )

        position_figure = figures.three_axis_time_series(
            estimate.times_s,
            [("Position error", position_error, style.COLOR_ERROR)],
            ("X", "Y", "Z"),
            "m",
        )
        error_summary_table = figures.summary_table(
            ["Metric", "RMSE", "Median", "P95", "Max", "N"],
            [
                [
                    "3-D position error (m)",
                    f"{position_summary.rmse:.4f}",
                    f"{position_summary.median:.4f}",
                    f"{position_summary.p95:.4f}",
                    f"{position_summary.maximum:.4f}",
                    str(position_summary.sample_count),
                ],
                [
                    "Attitude error (deg)",
                    f"{attitude_summary.rmse:.4f}",
                    f"{attitude_summary.median:.4f}",
                    f"{attitude_summary.p95:.4f}",
                    f"{attitude_summary.maximum:.4f}",
                    str(attitude_summary.sample_count),
                ],
            ],
        )
        trajectory_figure = figures.trajectory_xy(
            [
                ("Ground truth", truth.position_m, style.COLOR_GROUND_TRUTH),
                ("Inertial estimate", estimate.position_m, style.COLOR_PRIMARY_ESTIMATE),
            ]
        )
        sections.append(
            "<section class='plot-section'><h2>Position error vs ground truth</h2>"
            f"{html.figure_to_fragment(position_figure, 'inertial-position-error')}"
            f"{html.figure_to_fragment(error_summary_table, 'inertial-error-summary')}"
            "</section>"
            "<section class='plot-section'><h2>Trajectory (X-Y)</h2>"
            f"{html.figure_to_fragment(trajectory_figure, 'inertial-trajectory')}</section>"
        )

        gaps = metrics.inter_sample_gaps_s(estimate.times_s)
        if gaps.size:
            gap_figure = figures.rate_count_plot(
                estimate.times_s[1:], gaps, "Inter-sample gap (s)"
            )
            sections.append(
                "<section class='plot-section'><h2>Sample health</h2>"
                f"{html.figure_to_fragment(gap_figure, 'inertial-gaps')}</section>"
            )
    else:
        warnings.append(
            f"Cannot compare against ground truth: {TOPIC_GROUND_TRUTH} or "
            f"{TOPIC_ESTIMATE} published no messages"
        )

    body = "".join(sections) if sections else figures.empty_state_card_html(
        "No inertial odometry data available in this run."
    )
    page_html = html.render_page(
        "Inertial Odometry", "inertial_odometry.html", body, warnings
    )
    return ReportPage(
        filename="inertial_odometry.html",
        title="Inertial Odometry",
        html=page_html,
        summary_stats=summary_stats,
        warnings=tuple(warnings),
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this script.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Generate the standalone inertial-odometry report page.",
        parents=[build_common_parser()],
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: builds the report context for one test run and
             writes just the inertial-odometry page and shared assets.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success, `1` on a validation error.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    try:
        context = build_report_context(
            args.test_run,
            args.output_dir,
            args.bag,
            args.maximum_alignment_gap_s,
            args.maximum_image_frames,
        )
        page = generate_inertial_odometry_report(context)
        html.write_report([page], context.output_dir, context)
    except Exception as error:  # noqa: BLE001 -- top-level CLI error boundary
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {context.output_dir / page.filename}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
