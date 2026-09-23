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
from python_tools.data import alignment, metrics
from python_tools.data.models import ReportContext, ReportPage
from python_tools.diagnostics.log_parser import DIAGNOSTIC_TIME_AXIS_TITLE
from python_tools.reporting import figures, html, style

TOPIC_GROUND_TRUTH = "/alpha/localisation/ground_truth/odometry"
TOPIC_ESTIMATE = "/alpha/localisation/visual/odometry"
TOPIC_POINT_CLOUD = "/alpha/localisation/visual/point_cloud"
TOPIC_LEFT_IMAGE = "/alpha/drivers/loccam/left"
TOPIC_RIGHT_IMAGE = "/alpha/drivers/loccam/right"
TOPIC_FEATURES_IMAGE = "/alpha/localisation/visual/features"


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

    truth = context.bag.odometry.get(TOPIC_GROUND_TRUTH)
    estimate = context.bag.odometry.get(TOPIC_ESTIMATE)
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
            f"Cannot compare against ground truth: {TOPIC_GROUND_TRUTH} or "
            f"{TOPIC_ESTIMATE} published no messages"
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
        warnings.append(f"{TOPIC_POINT_CLOUD} published no messages")

    # visual_diag periodic diagnostics.
    records = context.diagnostics.visual_records
    if records:
        warnings.append(
            "visual_diag timing below is relative to the node log's own "
            "first record, not aligned to this page's other bag-elapsed-"
            "time plots -- no reliable wall-clock<->simulated-time anchor "
            "exists in this run to convert one axis into the other."
        )
        times_s = np.array([r.log_relative_time_s for r in records])
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
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
        )
        correspondence_figure = figures.rate_count_plot(
            times_s, correspondences, "Correspondences (count)",
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
        )
        inlier_pipeline_figure = figures.rate_count_plot(
            times_s, inliers, "PnP inliers (count, diagnostic snapshot)",
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
        )
        reprojection_figure = figures.rate_count_plot(
            times_s,
            np.array([r.reprojection_rms_px for r in records]),
            "Reprojection RMS (px)",
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
        )
        condition_figure = figures.rate_count_plot(
            times_s,
            np.array([r.normal_condition for r in records]),
            "Normal matrix condition number",
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
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
            x_title=DIAGNOSTIC_TIME_AXIS_TITLE,
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
            "<section class='plot-section'><h2>Feature pipeline diagnostics (5 s periodic snapshots)</h2>"
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
        warnings.append("No visual_diag diagnostic records found in the node log")

    # Sampled images, only present when --record-images was used.
    any_images = False
    for label, topic in (
        ("Left camera", TOPIC_LEFT_IMAGE),
        ("Right camera", TOPIC_RIGHT_IMAGE),
        ("Annotated features", TOPIC_FEATURES_IMAGE),
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
