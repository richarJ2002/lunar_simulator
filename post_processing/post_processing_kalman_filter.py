"""!
@brief  Generates the Kalman-filter subsystem report page: the fused
        estimate overlaid on every measurement source and ground truth
        (each clearly labeled by fusion role), fused error with +/-3-sigma
        covariance bands, and parsed localisation_diag periodic
        diagnostics (per-source counts, NIS, correction norm, covariance
        health, biases). Standalone entry point and importable
        `generate_kalman_filter_report()`.
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
from python_tools.diagnostics.log_parser import DIAGNOSTIC_TIME_AXIS_TITLE
from python_tools.reporting import figures, html, style

TOPIC_GROUND_TRUTH = "/alpha/localisation/ground_truth/odometry"
TOPIC_ESTIMATE = "/alpha/localisation/kalman_filter/odometry"
TOPIC_VISUAL = "/alpha/localisation/visual/odometry"
TOPIC_WHEEL = "/alpha/localisation/wheel/odometry"
TOPIC_INERTIAL = "/alpha/localisation/inertial/odometry"
INERTIAL_TRAJECTORY_LABEL = "Inertial (diagnostic only -- ESKF consumes raw IMU, not this)"

# Chi-square 99th-percentile thresholds by degrees of freedom, matching
# selectNisThreshold.cc's fixed table, used to display the effective
# threshold when a *_nis_threshold parameter is 0 (its "automatic" value).
# 2026-09-23 correction: 6 DoF is 16.812 -- this table previously listed
# the 7-DoF value (18.475) there, so the page overstated the visual gate.
_CHI_SQUARE_99_PERCENT = {
    1: 6.635,
    2: 9.210,
    3: 11.345,
    4: 13.277,
    5: 15.086,
    6: 16.812,
}


def _effective_nis_threshold(configured: float, degrees_of_freedom: int) -> float:
    """!
    @brief   Resolves the effective NIS threshold, matching
             selectNisThreshold.cc: a positive configured value is used
             verbatim; 0 means the automatic 99% chi-square threshold for
             the measurement's degrees of freedom.

    @param   configured
             The snapshotted `*_nis_threshold` parameter value.
    @param   degrees_of_freedom
             The measurement's dimension (6 for visual pose, 2 for wheel
             twist).

    @return  The effective threshold.
    """
    if configured > 0.0:
        return configured
    return _CHI_SQUARE_99_PERCENT[degrees_of_freedom]


def generate_kalman_filter_report(context: ReportContext) -> ReportPage:
    """!
    @brief   Builds the Kalman-filter report page.

    @param   context
             The report context to build the page from.

    @return  The assembled `ReportPage`.
    """
    warnings: list[str] = []
    sections: list[str] = []
    summary_stats: dict[str, str] = {}

    params = context.parameters.get("continuous_ekf")
    if params is not None:
        p = params.parameters
        visual_threshold = _effective_nis_threshold(
            float(p.get("visual_nis_threshold", 0.0)), 6
        )
        wheel_threshold = _effective_nis_threshold(
            float(p.get("wheel_nis_threshold", 0.0)), 2
        )
        sections.append(
            "<section class='plot-section'><h2>Configuration</h2><dl>"
            f"<dt>Prediction rate</dt><dd>{p.get('prediction_rate_hz', 'n/a')} Hz</dd>"
            f"<dt>fuse_visual_pose</dt><dd>{p.get('fuse_visual_pose', 'n/a')}</dd>"
            f"<dt>fuse_wheel_twist</dt><dd>{p.get('fuse_wheel_twist', 'n/a')}</dd>"
            f"<dt>Effective visual NIS threshold</dt><dd>{visual_threshold:.3f} (configured {p.get('visual_nis_threshold', 0.0)})</dd>"
            f"<dt>Effective wheel NIS threshold</dt><dd>{wheel_threshold:.3f} (configured {p.get('wheel_nis_threshold', 0.0)})</dd>"
            f"<dt>Maximum visual measurement age</dt><dd>{p.get('maximum_visual_measurement_age_s', 'n/a')} s</dd>"
            f"<dt>Maximum IMU measurement age</dt><dd>{p.get('maximum_imu_measurement_age_s', 'n/a')} s</dd>"
            "</dl></section>"
        )
    else:
        warnings.append("alpha_kalman_filter parameter snapshot not found")

    truth = context.bag.odometry.get(TOPIC_GROUND_TRUTH)
    estimate = context.bag.odometry.get(TOPIC_ESTIMATE)

    # Overlay every source at its own native sample times -- each is
    # resampled onto the fused estimate's own X-Y trajectory plot, never
    # implying every source shares the fused rate or fusion form.
    trajectory_series = []
    if truth is not None:
        trajectory_series.append(("Ground truth", truth.position_m, style.COLOR_GROUND_TRUTH))
    if estimate is not None:
        trajectory_series.append(("Fused estimate", estimate.position_m, style.COLOR_PRIMARY_ESTIMATE))
    visual = context.bag.odometry.get(TOPIC_VISUAL)
    if visual is not None:
        trajectory_series.append(("Visual (measurement)", visual.position_m, style.COLOR_SECONDARY_ESTIMATE))
    else:
        warnings.append(f"{TOPIC_VISUAL} published no messages: no visual measurement was available to fuse this run")
    wheel = context.bag.odometry.get(TOPIC_WHEEL)
    if wheel is not None:
        trajectory_series.append(("Wheel (measurement)", wheel.position_m, style.COLOR_TERTIARY_ESTIMATE))
    else:
        warnings.append(f"{TOPIC_WHEEL} published no messages: no wheel measurement was available to fuse this run")
    inertial = context.bag.odometry.get(TOPIC_INERTIAL)
    if inertial is not None:
        trajectory_series.append((INERTIAL_TRAJECTORY_LABEL, inertial.position_m, style.COLOR_RAW_SOURCE))
    else:
        warnings.append(f"{TOPIC_INERTIAL} published no messages (diagnostic-only comparison unavailable)")
    if trajectory_series:
        # The unaided inertial trace can drift kilometres while every other
        # source stays within metres, so it starts legend-only: it no longer
        # drives the initial axes, but one legend click restores it.
        trajectory_figure = figures.trajectory_xy(
            trajectory_series, legend_only_labels=(INERTIAL_TRAJECTORY_LABEL,)
        )
        sections.append(
            "<section class='plot-section'><h2>Trajectory overlay (all sources, X-Y)</h2>"
            "<p>Visual and wheel are fused measurements; inertial odometry "
            "is shown for diagnostic comparison only -- the ESKF consumes "
            "raw IMU directly, not this topic. It starts hidden because its "
            "unaided drift can dwarf every other trace; click it in the "
            "legend to show it.</p>"
            f"{html.figure_to_fragment(trajectory_figure, 'kalman-trajectory')}</section>"
        )

    if truth is not None and estimate is not None:
        position, orientation, linear_velocity, angular_velocity, valid = (
            alignment.interpolate_ground_truth(
                estimate.times_s, truth, context.maximum_alignment_gap_s
            )
        )
        position_error = metrics.position_error_by_axis(estimate.position_m, position, valid)
        attitude_error = metrics.attitude_error_deg(estimate.orientation_xyzw, orientation, valid)
        velocity_error = metrics.velocity_error(estimate.linear_velocity_mps, linear_velocity, valid)
        norm_3d_error = np.where(valid, metrics.norm_3d(np.nan_to_num(position_error, nan=0.0)), np.nan)

        position_summary = metrics.summarize_errors(norm_3d_error)
        attitude_summary = metrics.summarize_errors(attitude_error)
        summary_stats["3-D position RMSE"] = f"{position_summary.rmse:.3f} m"
        summary_stats["Attitude RMSE"] = f"{attitude_summary.rmse:.3f} deg"

        position_std_dev = metrics.covariance_std_dev(estimate.pose_covariance)[:, :3]
        position_figure = figures.estimate_truth_error_panel(
            estimate.times_s,
            metrics.norm_3d(estimate.position_m),
            metrics.norm_3d(position),
            norm_3d_error,
            "Position magnitude (m)",
            "3-D position error (m)",
            sigma_band=metrics.norm_3d(position_std_dev),
        )
        velocity_figure = figures.three_axis_time_series(
            estimate.times_s, [("Body velocity error", velocity_error, style.COLOR_ERROR)], ("X", "Y", "Z"), "m/s"
        )
        error_summary_table = figures.summary_table(
            ["Metric", "RMSE", "Median", "P95", "Max", "N"],
            [
                [
                    "3-D position error (m)", f"{position_summary.rmse:.4f}", f"{position_summary.median:.4f}",
                    f"{position_summary.p95:.4f}", f"{position_summary.maximum:.4f}", str(position_summary.sample_count),
                ],
                [
                    "Attitude error (deg)", f"{attitude_summary.rmse:.4f}", f"{attitude_summary.median:.4f}",
                    f"{attitude_summary.p95:.4f}", f"{attitude_summary.maximum:.4f}", str(attitude_summary.sample_count),
                ],
            ],
        )
        sections.append(
            "<section class='plot-section'><h2>Fused position error with +/-3-sigma covariance band</h2>"
            f"{html.figure_to_fragment(position_figure, 'kalman-position-error')}"
            f"{html.figure_to_fragment(error_summary_table, 'kalman-error-summary')}</section>"
            "<section class='plot-section'><h2>Fused body velocity error</h2>"
            f"{html.figure_to_fragment(velocity_figure, 'kalman-velocity-error')}</section>"
        )
    else:
        warnings.append(
            f"Cannot compare against ground truth: {TOPIC_GROUND_TRUTH} or "
            f"{TOPIC_ESTIMATE} published no messages"
        )

    # localisation_diag periodic diagnostics, one section per source.
    records = context.diagnostics.localisation_records
    if records:
        warnings.append(
            "localisation_diag timing below is relative to the node log's "
            "own first record, not aligned to this page's other "
            "bag-elapsed-time plots -- no reliable wall-clock<->"
            "simulated-time anchor exists in this run to convert one axis "
            "into the other."
        )
        for source in ("imu", "visual", "wheel"):
            source_records = [r for r in records if r.source == source]
            if not source_records:
                continue
            times_s = np.array([r.log_relative_time_s for r in source_records])
            received = np.array([r.received for r in source_records], dtype=float)
            accepted = np.array([r.accepted for r in source_records], dtype=float)
            fused = np.array([r.fused for r in source_records], dtype=float)
            # Counter deltas as interval rates, retaining the cumulative
            # totals in the summary table below, per the plan.
            received_rate = np.diff(received, prepend=received[0])
            fused_rate = np.diff(fused, prepend=fused[0])
            counts_figure = figures.three_axis_time_series(
                times_s,
                [("Received/Accepted/Fused (interval delta)", np.stack([received_rate, np.diff(accepted, prepend=accepted[0]), fused_rate], axis=1), style.COLOR_PRIMARY_ESTIMATE)],
                ("Received", "Accepted", "Fused"),
                "count / interval",
                x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
            )
            nis_figure = figures.rate_count_plot(
                times_s, np.array([r.nis for r in source_records]), "NIS",
                x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
            )
            correction_figure = figures.rate_count_plot(
                times_s, np.array([r.correction_norm for r in source_records]), "Correction norm",
                x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
            )
            latest = source_records[-1]
            summary_table = figures.summary_table(
                ["Metric", "Latest cumulative value"],
                [
                    ["Received", str(latest.received)],
                    ["Accepted", str(latest.accepted)],
                    ["Age-rejected", str(latest.age_rejected)],
                    ["NIS-rejected", str(latest.nis_rejected)],
                    ["Numerical-rejected", str(latest.numerical_rejected)],
                    ["Fused", str(latest.fused)],
                ],
            )
            sections.append(
                f"<section class='plot-section'><h2>{source.capitalize()} measurement diagnostics (5 s periodic)</h2>"
                f"{html.figure_to_fragment(summary_table, f'kalman-diag-table-{source}')}"
                f"{html.figure_to_fragment(counts_figure, f'kalman-diag-counts-{source}')}"
                f"{html.figure_to_fragment(nis_figure, f'kalman-diag-nis-{source}')}"
                f"{html.figure_to_fragment(correction_figure, f'kalman-diag-correction-{source}')}"
                "</section>"
            )
        # Filter-wide snapshot (identical across all three sources' lines
        # per tick -- shown once, not per source).
        latest = records[-1]
        times_s = np.array(sorted({r.log_relative_time_s for r in records}))
        trace_by_time = {r.log_relative_time_s: r.covariance_trace for r in records}
        trace_series = np.array([trace_by_time[t] for t in times_s])
        trace_figure = figures.rate_count_plot(
            times_s, trace_series, "Covariance trace", x_title=DIAGNOSTIC_TIME_AXIS_TITLE
        )
        bias_table = figures.summary_table(
            ["Metric", "Latest value"],
            [
                ["Quaternion norm", f"{latest.quaternion_norm:.9f}"],
                ["Covariance min eigenvalue", f"{latest.covariance_min_eigenvalue:.6e}"],
                ["Covariance diagonal range", f"[{latest.covariance_diagonal_min:.3e}, {latest.covariance_diagonal_max:.3e}]"],
                ["Accel bias (body, m/s^2)", str(tuple(round(v, 6) for v in latest.accel_bias_body_mps2))],
                ["Gyro bias (body, rad/s)", str(tuple(round(v, 6) for v in latest.gyro_bias_body_radps))],
            ],
        )
        sections.append(
            "<section class='plot-section'><h2>Filter-wide covariance &amp; bias snapshot</h2>"
            "<p>Bias states are log-only; the ESKF never publishes them on "
            "any topic.</p>"
            f"{html.figure_to_fragment(bias_table, 'kalman-bias-table')}"
            f"{html.figure_to_fragment(trace_figure, 'kalman-covariance-trace')}</section>"
        )
    else:
        warnings.append("No localisation_diag diagnostic records found in the node log")

    body = "".join(sections) if sections else figures.empty_state_card_html(
        "No kalman filter data available in this run."
    )
    page_html = html.render_page("Kalman Filter", "kalman_filter.html", body, warnings)
    return ReportPage(
        filename="kalman_filter.html",
        title="Kalman Filter",
        html=page_html,
        summary_stats=summary_stats,
        warnings=tuple(warnings),
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this script.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    return argparse.ArgumentParser(
        description="Generate the standalone Kalman-filter report page.",
        parents=[build_common_parser()],
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: builds the report context for one test run and
             writes just the Kalman-filter page and shared assets.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success, `1` on a validation error.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    try:
        context = build_report_context(
            args.test_run, args.output_dir, args.bag, args.maximum_alignment_gap_s, args.maximum_image_frames
        )
        page = generate_kalman_filter_report(context)
        html.write_report([page], context.output_dir, context)
    except Exception as error:  # noqa: BLE001 -- top-level CLI error boundary
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {context.output_dir / page.filename}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
