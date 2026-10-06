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
import numpy.typing as npt
import plotly.graph_objects as go

sys.path.insert(0, str(Path(__file__).resolve().parent))

from python_tools.context import build_common_parser, build_report_context
from python_tools.bag.topic_registry import topic_name_for
from python_tools.data import alignment, metrics
from python_tools.data.models import DiagnosticTimeBasis, OdometrySeries, ReportContext, ReportPage
from python_tools.diagnostics.bag_diagnostics import (
    DRIVER_STATUS_NAME,
    diagnostic_time_axis_title,
    diagnostics_topic_for,
    latest_status_values,
    system_state_topic_for,
    system_state_transitions,
)
from python_tools.reporting import figures, html, style

# Topic suffixes (without the /<system>/ prefix) this page reads; each is
# resolved against the run's own system where the context is available, so
# old runs read via the default system's names.
SUFFIX_GROUND_TRUTH = "localisation/ground_truth/odometry"
SUFFIX_ESTIMATE = "localisation/kalman_filter/odometry"
SUFFIX_VISUAL = "localisation/visual/odometry"
SUFFIX_WHEEL = "localisation/wheel/odometry"
SUFFIX_INERTIAL = "localisation/inertial/odometry"
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


def _smoothness_and_consistency_table(
    estimate: OdometrySeries,
    truth: OdometrySeries,
    truth_position_m: npt.NDArray[np.float64],
    position_error_m: npt.NDArray[np.float64],
    valid: npt.NDArray[np.bool_],
    summary_stats: dict[str, str],
) -> go.Figure:
    """!
    @brief   Builds the fused estimate's smoothness and consistency table
             (the baseline estimate metrics) and records its headline
             values
             in the page summary.

    @param   estimate
             The fused estimate series.
    @param   truth
             The ground-truth series, used for its own jitter baseline.
    @param   truth_position_m
             Ground-truth position interpolated to the estimate's sample
             times, metres, shape (N, 3).
    @param   position_error_m
             Signed estimate-minus-truth position error, metres, shape
             (N, 3); NaN where `valid` is False.
    @param   valid
             Per-sample ground-truth alignment validity, shape (N,).
    @param   summary_stats
             Page summary statistics, updated in place.

    @return  A Plotly summary-table figure.
    """
    # Fast jitter of each series' own position, in millimetres per axis.
    estimate_jitter_mm = 1000.0 * np.nanstd(
        metrics.high_frequency_residual(estimate.times_s, estimate.position_m), axis=0
    )
    truth_jitter_mm = 1000.0 * np.nanstd(
        metrics.high_frequency_residual(truth.times_s, truth.position_m), axis=0
    )
    # Staircase publication shows up as repeated stamps and repeated states.
    repeated_state_count = metrics.identical_sample_count(
        np.hstack(
            [estimate.position_m, estimate.orientation_xyzw, estimate.linear_velocity_mps]
        )
    )
    repeated_state_percent = 100.0 * repeated_state_count / max(1, estimate.times_s.size - 1)
    duplicate_stamps = metrics.duplicate_stamp_count(estimate.times_ns)
    largest_step_m = metrics.maximum_step_norm(estimate.position_m)
    # Coverage of the reported position sigma, per horizontal axis.
    position_std_dev_m = metrics.covariance_std_dev(estimate.pose_covariance)[:, :3]
    coverage = [
        metrics.sigma_coverage(position_error_m[:, axis], position_std_dev_m[:, axis])
        for axis in (0, 1)
    ]
    # Final horizontal error at the last aligned sample.
    valid_indices = np.nonzero(valid)[0]
    final_error_m = (
        float(np.linalg.norm(position_error_m[valid_indices[-1], :2]))
        if valid_indices.size
        else float("nan")
    )
    # Largest direction-of-travel error across all defined windows.
    direction_error_deg = metrics.direction_of_travel_error_deg(
        estimate.times_s, estimate.position_m, truth_position_m, valid
    )
    maximum_direction_error_deg = (
        float(np.nanmax(np.abs(direction_error_deg)))
        if np.any(np.isfinite(direction_error_deg))
        else float("nan")
    )
    # Headline values also appear on the index page.
    summary_stats["Repeated messages"] = f"{repeated_state_percent:.1f} %"
    summary_stats["Y jitter"] = f"{estimate_jitter_mm[1]:.1f} mm"
    summary_stats["Final horizontal error"] = f"{final_error_m:.3f} m"
    return figures.summary_table(
        ["Metric", "Fused estimate", "Ground truth"],
        [
            ["Messages (effective rate)", f"{estimate.times_s.size} ({metrics.sample_rate_hz(estimate.times_s) or float('nan'):.1f} Hz)", "-"],
            ["Duplicate stamps", str(duplicate_stamps), "-"],
            ["Repeated consecutive states", f"{repeated_state_count} ({repeated_state_percent:.1f} %)", "-"],
            ["Largest per-message step (m)", f"{largest_step_m:.4f}", "-"],
            ["Position jitter std X/Y/Z (mm)", "/".join(f"{v:.2f}" for v in estimate_jitter_mm), "/".join(f"{v:.2f}" for v in truth_jitter_mm)],
            ["Body lateral velocity std (m/s)", f"{np.nanstd(estimate.linear_velocity_mps[:, 1]):.4f}", f"{np.nanstd(truth.linear_velocity_mps[:, 1]):.4f}"],
            ["X error inside 1/3 sigma (%)", f"{100 * coverage[0].within_one_sigma:.1f} / {100 * coverage[0].within_three_sigma:.1f}", "-"],
            ["Y error inside 1/3 sigma (%)", f"{100 * coverage[1].within_one_sigma:.1f} / {100 * coverage[1].within_three_sigma:.1f}", "-"],
            ["Final horizontal error (m)", f"{final_error_m:.3f}", "-"],
            ["Max direction-of-travel error, 30 s (deg)", f"{maximum_direction_error_deg:.1f}", "-"],
        ],
    )


def _startup_window_section(context: ReportContext, summary_stats: dict[str, str]) -> str:
    """!
    @brief   Builds the start-up window section: every system state change
             and the driver's final command-gate counters.

    @param   context
             The report context.
    @param   summary_stats
             Page summary statistics, updated in place with the first
             READY time.

    @return  The section HTML, or an empty string for a run without a
             recorded system state.
    """
    state_series = context.bag.diagnostic_arrays.get(
        system_state_topic_for(context.run_metadata.system)
    )
    # Runs before the supervisor existed have nothing to show.
    if state_series is None or not state_series.samples:
        return ""
    transitions = system_state_transitions(state_series)
    rows = [[f"{time_s:.2f}", state, message] for time_s, state, message in transitions]
    # The first READY closes the start-up window.
    ready_times = [time_s for time_s, state, _ in transitions if state == "READY"]
    if ready_times:
        summary_stats["First READY"] = f"{ready_times[0]:.1f} s"
    table = figures.summary_table(["Elapsed (s)", "State", "Supervisor message"], rows)
    # Gate counters from the driver's latest recorded status.
    diagnostics_series = context.bag.diagnostic_arrays.get(
        diagnostics_topic_for(context.run_metadata.system)
    )
    gate_values = (
        latest_status_values(diagnostics_series, DRIVER_STATUS_NAME)
        if diagnostics_series is not None
        else {}
    )
    gate_table = figures.summary_table(
        ["Command gate", "Final count"],
        [
            ["Commands forwarded", gate_values.get("commands_forwarded", "n/a")],
            ["Commands blocked", gate_values.get("commands_blocked", "n/a")],
            ["Stop commands sent", gate_values.get("stop_commands", "n/a")],
        ],
    )
    return (
        "<section class='plot-section'><h2>Start-up window &amp; command gate</h2>"
        "<p>Commands reach the actuators only while the supervisor's state is "
        "READY with a fresh heartbeat; blocked commands are dropped, never "
        "replayed.</p>"
        f"{html.figure_to_fragment(table, 'kalman-system-state')}"
        f"{html.figure_to_fragment(gate_table, 'kalman-command-gate')}</section>"
    )


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
    system = context.run_metadata.system
    topic_ground_truth = topic_name_for(system, SUFFIX_GROUND_TRUTH)
    topic_estimate = topic_name_for(system, SUFFIX_ESTIMATE)
    topic_visual = topic_name_for(system, SUFFIX_VISUAL)
    topic_wheel = topic_name_for(system, SUFFIX_WHEEL)
    topic_inertial = topic_name_for(system, SUFFIX_INERTIAL)

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

    # Start-up window and command gate.
    startup_section = _startup_window_section(context, summary_stats)
    if startup_section:
        sections.append(startup_section)

    truth = context.bag.odometry.get(topic_ground_truth)
    estimate = context.bag.odometry.get(topic_estimate)

    # Overlay every source at its own native sample times -- each is
    # resampled onto the fused estimate's own X-Y trajectory plot, never
    # implying every source shares the fused rate or fusion form.
    trajectory_series = []
    if truth is not None:
        trajectory_series.append(("Ground truth", truth.position_m, style.COLOR_GROUND_TRUTH))
    if estimate is not None:
        trajectory_series.append(("Fused estimate", estimate.position_m, style.COLOR_PRIMARY_ESTIMATE))
    visual = context.bag.odometry.get(topic_visual)
    if visual is not None:
        trajectory_series.append(("Visual (measurement)", visual.position_m, style.COLOR_SECONDARY_ESTIMATE))
    else:
        warnings.append(f"{topic_visual} published no messages: no visual measurement was available to fuse this run")
    wheel = context.bag.odometry.get(topic_wheel)
    if wheel is not None:
        trajectory_series.append(("Wheel (measurement)", wheel.position_m, style.COLOR_TERTIARY_ESTIMATE))
    else:
        warnings.append(f"{topic_wheel} published no messages: no wheel measurement was available to fuse this run")
    inertial = context.bag.odometry.get(topic_inertial)
    if inertial is not None:
        trajectory_series.append((INERTIAL_TRAJECTORY_LABEL, inertial.position_m, style.COLOR_RAW_SOURCE))
    else:
        warnings.append(f"{topic_inertial} published no messages (diagnostic-only comparison unavailable)")
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
        quality_table = _smoothness_and_consistency_table(
            estimate, truth, position, position_error, valid, summary_stats
        )
        sections.append(
            "<section class='plot-section'><h2>Smoothness &amp; consistency</h2>"
            "<p>Publication artifacts (repeated stamps and messages), fast "
            "jitter (residual against a centred 2 s moving mean of each "
            "series' own position), covariance consistency (a consistent "
            "Gaussian error lies inside 1-sigma about 68 % and inside "
            "3-sigma about 99.7 % of the time) and the direction of travel "
            "over 30 s windows with at least 0.25 m of true travel.</p>"
            f"{html.figure_to_fragment(quality_table, 'kalman-quality-table')}</section>"
        )
    else:
        warnings.append(
            f"Cannot compare against ground truth: {topic_ground_truth} or "
            f"{topic_estimate} published no messages"
        )

    # Periodic estimator diagnostics, one section per source: recorded in
    # the bag when present, parsed from the node log's localisation_diag
    # lines otherwise.
    records = context.diagnostics.localisation_records
    diagnostic_axis_title = diagnostic_time_axis_title(context.diagnostics)
    if records:
        if context.diagnostics.time_basis is DiagnosticTimeBasis.LOG_RELATIVE:
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
            times_s = np.array([r.time_s for r in source_records])
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
                x_title=diagnostic_axis_title,
            )
            nis_figure = figures.rate_count_plot(
                times_s, np.array([r.nis for r in source_records]), "NIS",
                x_title=diagnostic_axis_title,
            )
            correction_figure = figures.rate_count_plot(
                times_s, np.array([r.correction_norm for r in source_records]), "Correction norm",
                x_title=diagnostic_axis_title,
            )
            latest = source_records[-1]
            summary_rows = [
                ["Received", str(latest.received)],
                ["Accepted", str(latest.accepted)],
                ["Age-rejected", str(latest.age_rejected)],
                ["NIS-rejected", str(latest.nis_rejected)],
                ["Numerical-rejected", str(latest.numerical_rejected)],
                ["Fused", str(latest.fused)],
            ]
            # Runs captured after the estimator split its age rejections by
            # reason show each reason; older runs keep only the total.
            breakdown = latest.age_rejection_breakdown
            if breakdown is not None:
                summary_rows.extend(
                    [
                        ["Age-rejected: pre-init", str(breakdown.pre_init)],
                        ["Age-rejected: negative age", str(breakdown.negative_age)],
                        ["Age-rejected: too old", str(breakdown.too_old)],
                        ["Age-rejected: state gap", str(breakdown.state_gap)],
                        ["Age-rejected: rollback failed", str(breakdown.rollback_failed)],
                        ["Age-rejected: predict failed", str(breakdown.predict_failed)],
                    ]
                )
            summary_table = figures.summary_table(
                ["Metric", "Latest cumulative value"], summary_rows
            )
            sections.append(
                f"<section class='plot-section'><h2>{source.capitalize()} measurement diagnostics (periodic)</h2>"
                f"{html.figure_to_fragment(summary_table, f'kalman-diag-table-{source}')}"
                f"{html.figure_to_fragment(counts_figure, f'kalman-diag-counts-{source}')}"
                f"{html.figure_to_fragment(nis_figure, f'kalman-diag-nis-{source}')}"
                f"{html.figure_to_fragment(correction_figure, f'kalman-diag-correction-{source}')}"
                "</section>"
            )
        # Filter-wide snapshot (identical across all three sources' lines
        # per tick -- shown once, not per source).
        latest = records[-1]
        times_s = np.array(sorted({r.time_s for r in records}))
        trace_by_time = {r.time_s: r.covariance_trace for r in records}
        trace_series = np.array([trace_by_time[t] for t in times_s])
        trace_figure = figures.rate_count_plot(
            times_s, trace_series, "Covariance trace", x_title=diagnostic_axis_title
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
        warnings.append("No estimator diagnostic records found in the bag or node log")

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
