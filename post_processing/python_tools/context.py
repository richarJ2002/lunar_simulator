"""!
@brief  Shared command-line building blocks and `ReportContext`
        construction for every post-processing entry point
        (post_processing.py and the four post_processing_*.py scripts), so
        the aggregate and standalone scripts build an identical context
        from one place rather than duplicating validation and parsing, or
        shelling out to each other. Not runnable standalone: this module
        defines only shared parser/context-building glue for other
        scripts' own `main()` functions to call, per the project's
        standard argument-parsing pattern for shared flags.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional

import yaml

from python_tools.bag.reader import (
    DEFAULT_MAXIMUM_ALIGNMENT_GAP_S,
    DEFAULT_MAXIMUM_IMAGE_FRAMES,
    BagValidationError,
    discover_bag_path,
    read_bag,
    validate_test_run_dir,
)
from python_tools.diagnostics.log_parser import find_latest_log, parse_log_file
from python_tools.data.models import DiagnosticLog, ParameterSnapshot, ReportContext, RunMetadata

# Snapshotted node parameter files this tool reads, and each file's
# registered node name (the YAML top-level key), which AGENTS.md notes is
# not always the directory name (continuous_ekf is the example).
PARAMETER_FILES: tuple[tuple[str, str], ...] = (
    ("parameters/systems/alpha/alpha_drivers/alpha_drivers.yaml", "alpha_driver_node"),
    ("parameters/systems/alpha/alpha_localisation/alpha_kalman_filter.yaml", "continuous_ekf"),
    ("parameters/systems/alpha/alpha_localisation/ground_truth.yaml", "ground_truth"),
    ("parameters/systems/alpha/alpha_localisation/inertial_odometry.yaml", "inertial_odometry"),
    ("parameters/systems/alpha/alpha_localisation/visual_odometry.yaml", "visual_odometry"),
    ("parameters/systems/alpha/alpha_localisation/wheel_odometry.yaml", "wheel_odometry"),
    ("parameters/systems/alpha/alpha_control/ackermann_controller.yaml", "ackermann_controller"),
)

# scripts/launch_simulator.sh's start_recorder() writes this file as a
# sibling of the bag directory it names.
RECORDING_MANIFEST_FILENAME = "manifest.json"

# The alpha_node executable's log filename prefix, as ROS_LOG_DIR names it.
ALPHA_NODE_LOG_PREFIX = "alpha_node"


def build_common_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the parent parser for the flags every post-processing
             entry point shares, composed via `parents=[...]` in each
             script's own parser so the flag's name, type, default and
             help text stay identical everywhere.

    @return  A parser with `add_help=False`, so composing it does not
             create a duplicate `-h` flag in the child parser.
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--test-run",
        type=Path,
        required=True,
        help="Captured test run directory (e.g. test_runs/<run>).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report output directory (default: <test-run>/post_processing).",
    )
    parser.add_argument(
        "--bag",
        type=Path,
        default=None,
        help="Explicit bag path, overriding the run's default bag location.",
    )
    parser.add_argument(
        "--maximum-alignment-gap-s",
        type=float,
        default=DEFAULT_MAXIMUM_ALIGNMENT_GAP_S,
        help=(
            "Largest inter-sample gap, in seconds, still considered one "
            f"segment and eligible for ground-truth alignment (default: "
            f"{DEFAULT_MAXIMUM_ALIGNMENT_GAP_S})."
        ),
    )
    parser.add_argument(
        "--maximum-image-frames",
        type=int,
        default=DEFAULT_MAXIMUM_IMAGE_FRAMES,
        help=(
            "Largest number of image/point-cloud frames retained per "
            f"topic (default: {DEFAULT_MAXIMUM_IMAGE_FRAMES})."
        ),
    )
    return parser


def load_parameter_snapshots(test_run_dir: Path) -> dict[str, ParameterSnapshot]:
    """!
    @brief   Loads every recognized node's snapshotted parameters from a
             run's own `parameters/` copy.

    @param   test_run_dir
             The test run directory.

    @return  Snapshots keyed by registered node name; a file that is
             missing or carries no matching node key is silently omitted
             rather than raised, since not every run exercises every node.
    """
    snapshots: dict[str, ParameterSnapshot] = {}
    for relative_path, node_name in PARAMETER_FILES:
        source_path = test_run_dir / relative_path
        if not source_path.is_file():
            continue
        document = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
        node_section = document.get(node_name)
        if not isinstance(node_section, dict):
            continue
        parameters = node_section.get("ros__parameters")
        if not isinstance(parameters, dict):
            continue
        snapshots[node_name] = ParameterSnapshot(
            source_path=source_path, node_name=node_name, parameters=parameters
        )
    return snapshots


def _load_recording_manifest(bag_path: Path) -> dict:
    """!
    @brief   Loads the recording manifest scripts/launch_simulator.sh's
             start_recorder() writes alongside the bag, if present.

    @param   bag_path
             The bag directory that was read.

    @return  The manifest as a dict, or an empty dict for an old run
             captured before automatic recording existed.
    """
    manifest_path = bag_path.parent / RECORDING_MANIFEST_FILENAME
    if not manifest_path.is_file():
        return {}
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def build_report_context(
    test_run_dir: Path,
    output_dir: Optional[Path],
    bag_path_override: Optional[Path],
    maximum_alignment_gap_s: float,
    maximum_image_frames: int,
) -> ReportContext:
    """!
    @brief   Validates a test run, reads its bag exactly once, parses its
             diagnostic log, loads its parameter snapshots, and assembles
             the resulting `ReportContext`. The one function every entry
             point (aggregate or standalone) calls to build its context.

    @param   test_run_dir
             The test run directory to report on.
    @param   output_dir
             Explicit report output directory, or `None` to default to
             `<test_run_dir>/post_processing`.
    @param   bag_path_override
             Explicit bag path, or `None` to discover the run's default bag.
    @param   maximum_alignment_gap_s
             Largest inter-sample gap still considered one segment.
    @param   maximum_image_frames
             Largest number of image/point-cloud frames retained per topic.

    @return  The assembled `ReportContext`.

    @throws  BagValidationError
             If the test run or bag fails validation, or `output_dir`
             names an existing file rather than a directory.
    """
    validate_test_run_dir(test_run_dir)
    bag_path = discover_bag_path(test_run_dir, bag_path_override)
    bag = read_bag(bag_path, maximum_alignment_gap_s, maximum_image_frames)

    manifest = _load_recording_manifest(bag_path)
    run_metadata = RunMetadata(
        test_run_dir=test_run_dir,
        bag_path=bag_path,
        world=manifest.get("world", "unknown"),
        system=manifest.get("system", "unknown"),
        command_line=manifest.get("command_line"),
        ros_domain_id=manifest.get("ros_domain_id"),
        gz_partition=manifest.get("gz_partition"),
        recording_profile=manifest.get("recording_profile"),
        source_revision=manifest.get("source_revision"),
        dirty_worktree=manifest.get("dirty_worktree"),
        storage_identifier=bag.storage_identifier,
        start_time_ns=bag.start_time_ns,
        end_time_ns=bag.end_time_ns,
    )

    log_path = find_latest_log(test_run_dir / "ros" / "logs", ALPHA_NODE_LOG_PREFIX)
    diagnostics = (
        parse_log_file(log_path)
        if log_path is not None
        else DiagnosticLog(log_path=test_run_dir / "ros" / "logs")
    )

    resolved_output_dir = output_dir if output_dir is not None else test_run_dir / "post_processing"
    if resolved_output_dir.is_file():
        raise BagValidationError(
            f"--output-dir names an existing file, not a directory: {resolved_output_dir}"
        )

    return ReportContext(
        run_metadata=run_metadata,
        bag=bag,
        diagnostics=diagnostics,
        parameters=load_parameter_snapshots(test_run_dir),
        output_dir=resolved_output_dir,
        maximum_alignment_gap_s=maximum_alignment_gap_s,
        maximum_image_frames=maximum_image_frames,
    )
