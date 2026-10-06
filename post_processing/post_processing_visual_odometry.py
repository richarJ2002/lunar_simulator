"""!
@brief  Generates the visual-odometry subsystem report page: sparse
        pose/body-twist comparison against ground truth at accepted visual
        timestamps only, PnP inlier/point-cloud snapshots, parsed
        visual_diag periodic diagnostics, and sampled camera/feature
        frames when `--record-images` data exists. Standalone entry point
        and importable `generate_visual_odometry_report()`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from python_tools.context import build_common_parser, build_report_context
from python_tools.bag.topic_registry import topic_name_for
from python_tools.data import alignment, metrics
from python_tools.data.models import DiagnosticTimeBasis, ReportContext, ReportPage
from python_tools.diagnostics.bag_diagnostics import diagnostic_time_axis_title
from python_tools.reporting import figures, html, style

# Topic suffixes (without the /<system>/ prefix) this page reads; each is
# resolved against the run's own system where the context is available, so
# old runs read via the default system's names.
SUFFIX_GROUND_TRUTH = "localisation/ground_truth/odometry"
SUFFIX_ESTIMATE = "localisation/visual/odometry"
SUFFIX_POINT_CLOUD = "localisation/visual/point_cloud"
SUFFIX_LEFT_IMAGE = "drivers/loccam/left"
SUFFIX_RIGHT_IMAGE = "drivers/loccam/right"
SUFFIX_FEATURES_IMAGE = "localisation/visual/features"


def generate_visual_odometry_report(context: ReportContext) -> ReportPage:
    """!
    @brief   Builds the visual-odometry report page.

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
    topic_point_cloud = topic_name_for(system, SUFFIX_POINT_CLOUD)
    topic_left_image = topic_name_for(system, SUFFIX_LEFT_IMAGE)
    topic_right_image = topic_name_for(system, SUFFIX_RIGHT_IMAGE)
    topic_features_image = topic_name_for(system, SUFFIX_FEATURES_IMAGE)

    truth = context.bag.odometry.get(topic_ground_truth)
    estimate = context.bag.odometry.get(topic_estimate)
    if truth is not None and estimate is not None:
        position, orientation, linear_velocity, angular_velocity, valid = (
            alignment.interpolate_ground_truth(
                estimate.times_s, truth, context.maximum_alignment_gap_s
            )
        )
        position_error = metrics.position_error_by_axis(estimate.position_m, position, valid)
        attitude_error = metrics.attitude_error_deg(estimate.orientation_xyzw, orientation, valid)
        norm_3d_error = np.where(
            valid, metrics.norm_3d(np.nan_to_num(position_error, nan=0.0)), np.nan
        )
        position_summary = metrics.summarize_errors(norm_3d_error)
        attitude_summary = metrics.summarize_errors(attitude_error)
        summary_stats["3-D position RMSE"] = f"{position_summary.rmse:.3f} m"
        summary_stats["Attitude RMSE"] = f"{attitude_summary.rmse:.3f} deg"

        # Output cadence: visual odometry is sparse by nature, so gaps are
        # expected and shown rather than implying missing data is a fault.
        rate = metrics.sample_rate_hz(estimate.times_s)
        summary_stats["Output cadence"] = f"{rate:.2f} Hz" if rate else "n/a"
        gaps = metrics.inter_sample_gaps_s(estimate.times_s)

        position_figure = figures.three_axis_time_series(
            estimate.times_s, [("Position error", position_error, style.COLOR_ERROR)], ("X", "Y", "Z"), "m"
        )
        trajectory_figure = figures.trajectory_xy(
            [
                ("Ground truth", truth.position_m, style.COLOR_GROUND_TRUTH),
                ("Visual estimate", estimate.position_m, style.COLOR_PRIMARY_ESTIMATE),
            ]
        )
        sections.append(
            "<section class='plot-section'><h2>Position error vs ground truth (at accepted visual timestamps)</h2>"
            f"{html.figure_to_fragment(position_figure, 'visual-position-error')}</section>"
            "<section class='plot-section'><h2>Trajectory (X-Y)</h2>"
            f"{html.figure_to_fragment(trajectory_figure, 'visual-trajectory')}</section>"
        )
        # Translation-direction bias: compare where visual odometry says the
        # rover travelled with where it actually travelled, per 30 s window.
        direction_error_deg = metrics.direction_of_travel_error_deg(
            estimate.times_s, estimate.position_m, position, valid
        )
        has_direction = np.isfinite(direction_error_deg)
        if np.any(has_direction):
            summary_stats["Max direction-of-travel error"] = (
                f"{float(np.max(np.abs(direction_error_deg[has_direction]))):.1f} deg"
            )
            direction_figure = figures.rate_count_plot(
                estimate.times_s[has_direction],
                direction_error_deg[has_direction],
                "Direction-of-travel error (deg)",
            )
            sections.append(
                "<section class='plot-section'><h2>Direction of travel vs ground truth (30 s windows)</h2>"
                "<p>Signed angle from the true horizontal displacement to the "
                "visual displacement over the preceding 30 s, shown only for "
                "windows with at least 0.25 m of true travel. A persistent "
                "offset while heading stays accurate indicates sideways "
                "translation mistaken for rotation.</p>"
                f"{html.figure_to_fragment(direction_figure, 'visual-direction-error')}</section>"
            )
        else:
            warnings.append(
                "Direction of travel not assessed: no 30 s window had at least 0.25 m of true travel"
            )
        if gaps.size:
            gap_figure = figures.rate_count_plot(
                estimate.times_s[1:], gaps, "Inter-sample gap (s)"
            )
            sections.append(
                "<section class='plot-section'><h2>Output cadence &amp; gaps</h2>"
                f"{html.figure_to_fragment(gap_figure, 'visual-gaps')}</section>"
            )
    else:
        warnings.append(
            f"Cannot compare against ground truth: {topic_ground_truth} or "
            f"{topic_estimate} published no messages"
        )

    point_cloud = context.bag.point_cloud
    if point_cloud is not None and point_cloud.times_s.size:
        inlier_figure = figures.rate_count_plot(
            point_cloud.times_s, point_cloud.point_counts.astype(float), "PnP inlier count"
        )
        sections.append(
            "<section class='plot-section'><h2>PnP inlier count per frame</h2>"
            "<p>Width equals the accepted PnP inlier count on an accepted "
            "frame, or the full reconstructed correspondence count on a "
            "rejected/unavailable frame (not directly comparable across "
            "both cases).</p>"
            f"{html.figure_to_fragment(inlier_figure, 'visual-inliers')}</section>"
        )
        if point_cloud.snapshots:
            cloud_figure = figures.point_cloud_3d_slider(point_cloud.snapshots)
            sections.append(
                "<section class='plot-section'><h2>Reconstructed point cloud (sampled frames)</h2>"
                f"{html.figure_to_fragment(cloud_figure, 'visual-point-cloud')}</section>"
            )
    else:
        warnings.append(f"{topic_point_cloud} published no messages")

    # Periodic pipeline diagnostics: recorded in the bag when present,
    # parsed from the node log's visual_diag lines otherwise.
    records = context.diagnostics.visual_records
    diagnostic_axis_title = diagnostic_time_axis_title(context.diagnostics)
    if records:
        if context.diagnostics.time_basis is DiagnosticTimeBasis.LOG_RELATIVE:
            warnings.append(
                "visual_diag timing below is relative to the node log's own "
                "first record, not aligned to this page's other bag-elapsed-"
                "time plots -- no reliable wall-clock<->simulated-time anchor "
                "exists in this run to convert one axis into the other."
            )
        times_s = np.array([r.time_s for r in records])
        detected = np.array([r.detected for r in records], dtype=float)
        tracked = np.array([r.tracked for r in records], dtype=float)
        stereo_valid = np.array([r.stereo_valid for r in records], dtype=float)
        correspondences = np.array([r.correspondences for r in records], dtype=float)
        inliers = np.array([r.inliers for r in records], dtype=float)
        counts_figure = figures.three_axis_time_series(
            times_s,
            [
                ("Detected/Tracked/Stereo-valid", np.stack([detected, tracked, stereo_valid], axis=1), style.COLOR_PRIMARY_ESTIMATE),
            ],
            ("Detected", "Tracked", "Stereo-valid"),
            "count",
            x_title=diagnostic_axis_title,
        )
        correspondence_figure = figures.rate_count_plot(
            times_s, correspondences, "Correspondences (count)",
            x_title=diagnostic_axis_title,
        )
        inlier_pipeline_figure = figures.rate_count_plot(
            times_s, inliers, "PnP inliers (count, diagnostic snapshot)",
            x_title=diagnostic_axis_title,
        )
        reprojection_figure = figures.rate_count_plot(
            times_s,
            np.array([r.reprojection_rms_px for r in records]),
            "Reprojection RMS (px)",
            x_title=diagnostic_axis_title,
        )
        condition_figure = figures.rate_count_plot(
            times_s,
            np.array([r.normal_condition for r in records]),
            "Normal matrix condition number",
            x_title=diagnostic_axis_title,
        )
        timing_series = np.stack(
            [
                [r.detection_ms for r in records],
                [r.tracking_ms for r in records],
                [r.pnp_ms for r in records],
            ],
            axis=1,
        )
        timing_figure = figures.three_axis_time_series(
            times_s, [("Detection/Tracking/PnP", timing_series, style.COLOR_PRIMARY_ESTIMATE)],
            ("Detection (ms)", "Tracking (ms)", "PnP (ms)"), "ms",
            x_title=diagnostic_axis_title,
        )
        diag_table = figures.summary_table(
            ["Metric", "Latest value"],
            [
                ["Received / Accepted / Failed", f"{records[-1].received} / {records[-1].accepted} / {records[-1].failed}"],
                ["Consecutive failures", str(records[-1].consecutive_failures)],
                ["Occupancy", f"{records[-1].occupancy:.3f}"],
                ["Near/mid ratio", f"{records[-1].near_mid_ratio:.3f}"],
                ["Disparity p10/p50/p90 (px)", f"{records[-1].disparity_p10_px:.2f} / {records[-1].disparity_p50_px:.2f} / {records[-1].disparity_p90_px:.2f}"],
                ["Depth p10/p50/p90 (m)", f"{records[-1].depth_p10_m:.2f} / {records[-1].depth_p50_m:.2f} / {records[-1].depth_p90_m:.2f}"],
            ],
        )
        sections.append(
            "<section class='plot-section'><h2>Feature pipeline diagnostics (periodic snapshots)</h2>"
            "<p>These summarize roughly the preceding five seconds, not a per-frame measurement.</p>"
            f"{html.figure_to_fragment(diag_table, 'visual-diag-table')}"
            f"{html.figure_to_fragment(counts_figure, 'visual-diag-counts')}"
            f"{html.figure_to_fragment(correspondence_figure, 'visual-diag-correspondences')}"
            f"{html.figure_to_fragment(inlier_pipeline_figure, 'visual-diag-inliers')}"
            f"{html.figure_to_fragment(timing_figure, 'visual-diag-timing')}"
            f"{html.figure_to_fragment(reprojection_figure, 'visual-diag-reprojection')}"
            f"{html.figure_to_fragment(condition_figure, 'visual-diag-condition')}"
            "</section>"
        )
    else:
        warnings.append("No visual pipeline diagnostic records found in the bag or node log")

    # Sampled images, only present when --record-images was used.
    any_images = False
    for label, topic in (
        ("Left camera", topic_left_image),
        ("Right camera", topic_right_image),
        ("Annotated features", topic_features_image),
    ):
        image_series = context.bag.images.get(topic)
        if image_series is None or image_series.total_message_count == 0:
            continue
        any_images = True
        if image_series.rejected_encoding_count:
            warnings.append(
                f"{topic}: {image_series.rejected_encoding_count} frame(s) had "
                "an unsupported encoding and were not decoded"
            )
        if image_series.frames:
            image_figure = figures.image_slider_figure(image_series.frames)
            sections.append(
                f"<section class='plot-section'><h2>{html.escape(label)} (sampled)</h2>"
                f"{html.figure_to_fragment(image_figure, f'visual-images-{topic}')}</section>"
            )
    if not any_images:
        sections.append(
            figures.empty_state_card_html(
                "No image data in this run. Capture with "
                "<code>./scripts/launch_simulator.sh --record-images</code> "
                "to include sampled left/right/annotated frames on this page.",
                severity="warning",
            )
        )

    body = "".join(sections) if sections else figures.empty_state_card_html(
        "No visual odometry data available in this run."
    )
    page_html = html.render_page("Visual Odometry", "visual_odometry.html", body, warnings)
    return ReportPage(
        filename="visual_odometry.html",
        title="Visual Odometry",
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
        description="Generate the standalone visual-odometry report page.",
        parents=[build_common_parser()],
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: builds the report context for one test run and
             writes just the visual-odometry page and shared assets.

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
        page = generate_visual_odometry_report(context)
        html.write_report([page], context.output_dir, context)
    except Exception as error:  # noqa: BLE001 -- top-level CLI error boundary
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {context.output_dir / page.filename}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
