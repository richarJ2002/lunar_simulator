"""!
@brief  Aggregate post-processing entry point. Validates a test run once,
        reads its bag once, parses its diagnostic log once, calls all four
        subsystem `generate_*_report()` functions directly against the
        same context, then writes the index and every subsystem page in
        one atomic report write.

        Usage:
            source /opt/ros/jazzy/setup.bash
            source install/setup.bash
            python3 post_processing/post_processing.py --test-run test_runs/<run>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from post_processing_inertial_odometry import generate_inertial_odometry_report
from post_processing_kalman_filter import generate_kalman_filter_report
from post_processing_visual_odometry import generate_visual_odometry_report
from post_processing_wheel_odometry import generate_wheel_odometry_report
from python_tools.context import build_common_parser, build_report_context
from python_tools.data.models import ReportContext, ReportPage
from python_tools.reporting import html


def generate_all_reports(context: ReportContext) -> list[ReportPage]:
    """!
    @brief   Generates every report page for one context: the four
             subsystem pages plus the index that links and summarizes
             them.

    @param   context
             The report context, built once from a single bag read and
             log parse, shared by every subsystem page.

    @return  Every page to write, index first.
    """
    subsystem_pages = [
        generate_kalman_filter_report(context),
        generate_visual_odometry_report(context),
        generate_inertial_odometry_report(context),
        generate_wheel_odometry_report(context),
    ]
    index_page = html.render_index_page(context, subsystem_pages)
    return [index_page, *subsystem_pages]


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this script.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    return argparse.ArgumentParser(
        description=(
            "Generate the complete offline post-processing report site "
            "(index plus all four subsystem pages) for one captured test run."
        ),
        parents=[build_common_parser()],
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: builds the report context for one test run
             once, generates every page, and writes the complete site.

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
        pages = generate_all_reports(context)
        html.write_report(pages, context.output_dir, context)
    except Exception as error:  # noqa: BLE001 -- top-level CLI error boundary
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {len(pages)} pages to {context.output_dir}")
    for page in pages:
        if page.warnings:
            print(f"  {page.filename}: {len(page.warnings)} warning(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
