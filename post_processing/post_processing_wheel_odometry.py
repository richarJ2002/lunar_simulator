"""!
@brief  Generates the wheel-odometry subsystem report page: raw/noisy
        joint states, command chain, derived wheel circumferential speed,
        slip ratios/observations, and wheel odometry's pose/body twist
        compared against ground truth. Standalone entry point and
        importable `generate_wheel_odometry_report()`.
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
from python_tools.data.models import WHEEL_ORDER, JointStateSeries, ReportContext, ReportPage
from python_tools.reporting import figures, html, style

TOPIC_GROUND_TRUTH = "/alpha/localisation/ground_truth/odometry"
TOPIC_ESTIMATE = "/alpha/localisation/wheel/odometry"
TOPIC_RAW_JOINT_STATES = "/alpha/drivers/joint_states"
TOPIC_NOISY_JOINT_STATES = "/alpha/joint_states"
TOPIC_VELOCITY_COMMAND = "/alpha/control/cmd/velocity"
TOPIC_PUBLIC_WHEEL_COMMAND = "/alpha/control/cmd/wheel_joint_states"
TOPIC_DRIVER_WHEEL_COMMAND = "/alpha/drivers/cmd/wheel_joint_states"
TOPIC_SLIP_RATIOS = "/alpha/localisation/wheel/slip_ratios"
TOPIC_SLIP_OBSERVATION = "/alpha/localisation/wheel/slip_observation"
TOPIC_KALMAN_SLIP = "/alpha/localisation/kalman_filter/wheel_slip_ratio"

# System name prefix this project's joint names always carry (findJoint.cc's
# own fixed strings), e.g. "alpha/front_left_drive_joint".
_JOINT_NAME_SYSTEM_PREFIX = "alpha"


def _select_wheel_columns(
    joint_states: Optional[JointStateSeries], suffix: str
) -> Optional[np.ndarray]:
    """!
    @brief   Selects the six drive- or steer-joint columns from a
             JointStateSeries, in WHEEL_ORDER, by resolved joint name.

    @param   joint_states
             The joint-state series to select from, or `None`.
    @param   suffix
             Either "_drive_joint" or "_steer_joint".

    @return  The selected columns, shape (N, 6) for position and (N, 6)
             for velocity stacked as a tuple `(position, velocity)`; `None`
             if `joint_states` is `None` or is missing any required joint.
    """
    if joint_states is None:
        return None
    expected_names = [
        f"{_JOINT_NAME_SYSTEM_PREFIX}/{wheel}{suffix}" for wheel in WHEEL_ORDER
    ]
    name_to_index = {name: index for index, name in enumerate(joint_states.joint_names)}
    if not all(name in name_to_index for name in expected_names):
        return None
    indices = [name_to_index[name] for name in expected_names]
    return joint_states.position_rad[:, indices], joint_states.velocity_radps[:, indices]


def generate_wheel_odometry_report(context: ReportContext) -> ReportPage:
    """!
    @brief   Builds the wheel-odometry report page.

    @param   context
             The report context to build the page from.

    @return  The assembled `ReportPage`.
    """
    warnings: list[str] = []
    sections: list[str] = []
    summary_stats: dict[str, str] = {}

    params = context.parameters.get("wheel_odometry")
    wheel_radius_m = None
    direction_multipliers = None
    estimate_slip = False
    if params is not None:
        p = params.parameters
        wheel_radius_m = float(p.get("wheel_radius_m", 0.0)) or None
        multipliers_list = p.get("drive_direction_multipliers")
        if isinstance(multipliers_list, (list, tuple)) and len(multipliers_list) == 6:
            direction_multipliers = np.array(multipliers_list, dtype=np.float64)
        estimate_slip = p.get("estimate_slip_from_visual", False)
        apply_slip = p.get("apply_slip_feedback", False)
        sections.append(
            "<section class='plot-section'><h2>Configuration</h2><dl>"
            f"<dt>Wheel radius</dt><dd>{wheel_radius_m} m</dd>"
            f"<dt>Drive direction multipliers</dt><dd>{multipliers_list}</dd>"
            f"<dt>estimate_slip_from_visual</dt><dd>{estimate_slip}</dd>"
            f"<dt>apply_slip_feedback</dt><dd>{apply_slip}</dd>"
            "</dl></section>"
        )
        if not estimate_slip:
            sections.append(
                figures.empty_state_card_html(
                    "estimate_slip_from_visual is false: no per-cycle slip "
                    "observations are computed by this node. Absent "
                    f"{TOPIC_SLIP_OBSERVATION} samples reflect this "
                    "configuration, not a failure.",
                    severity="warning",
                )
            )
        if not apply_slip:
            sections.append(
                figures.empty_state_card_html(
                    "apply_slip_feedback is false: the rolling-constraint "
                    "solve uses the statically configured slip default, "
                    "not fused feedback from the Kalman filter.",
                    severity="warning",
                )
            )
    else:
        warnings.append("wheel_odometry parameter snapshot not found")

    raw_joints = context.bag.joint_states.get(TOPIC_RAW_JOINT_STATES)
    noisy_joints = context.bag.joint_states.get(TOPIC_NOISY_JOINT_STATES)

    for label, joints, color in (
        ("Raw (Gazebo)", raw_joints, style.COLOR_RAW_SOURCE),
        ("Noisy (driver)", noisy_joints, style.COLOR_PRIMARY_ESTIMATE),
    ):
        drive_and_steer = _select_wheel_columns(joints, "_drive_joint")
        steer = _select_wheel_columns(joints, "_steer_joint")
        if drive_and_steer is None or steer is None:
            warnings.append(f"{label} joint states missing one or more of the six wheels")
            continue
        _, drive_velocity = drive_and_steer
        steer_position, _ = steer
        drive_figure = figures.six_wheel_small_multiples(
            joints.times_s, drive_velocity, "Drive velocity (rad/s)"
        )
        steer_figure = figures.six_wheel_small_multiples(
            joints.times_s, steer_position, "Steer position (rad)"
        )
        sections.append(
            f"<section class='plot-section'><h2>{html.escape(label)}: drive velocity</h2>"
            f"{html.figure_to_fragment(drive_figure, f'wheel-drive-{label}')}</section>"
            f"<section class='plot-section'><h2>{html.escape(label)}: steer position</h2>"
            f"{html.figure_to_fragment(steer_figure, f'wheel-steer-{label}')}</section>"
        )
        if wheel_radius_m and direction_multipliers is not None:
            speed_mps = metrics.wheel_speed_mps(
                drive_velocity, wheel_radius_m, direction_multipliers
            )
            speed_figure = figures.six_wheel_small_multiples(
                joints.times_s, speed_mps, "Circumferential speed (m/s)"
            )
            sections.append(
                f"<section class='plot-section'><h2>{html.escape(label)}: derived wheel speed</h2>"
                f"{html.figure_to_fragment(speed_figure, f'wheel-speed-{label}')}</section>"
            )

    # Command chain: body velocity -> public per-wheel command -> noisy
    # driver command, so command generation and injected noise are visible.
    velocity_command = context.bag.twist_commands.get(TOPIC_VELOCITY_COMMAND)
    if velocity_command is not None:
        command_figure = figures.three_axis_time_series(
            velocity_command.times_s,
            [
                ("Commanded linear", velocity_command.linear_mps, style.COLOR_PRIMARY_ESTIMATE),
            ],
            ("X", "Y", "Z"),
            "m/s",
        )
        sections.append(
            "<section class='plot-section'><h2>Body velocity command</h2>"
            f"{html.figure_to_fragment(command_figure, 'wheel-velocity-command')}</section>"
        )
    else:
        warnings.append(
            f"{TOPIC_VELOCITY_COMMAND} published no messages: the rover was "
            "never commanded via the higher-level Twist interface this run"
        )
    public_command = context.bag.wheel_actuators.get(TOPIC_PUBLIC_WHEEL_COMMAND)
    driver_command = context.bag.wheel_actuators.get(TOPIC_DRIVER_WHEEL_COMMAND)
    for label, topic, command in (
        ("Public wheel command", TOPIC_PUBLIC_WHEEL_COMMAND, public_command),
        ("Noisy driver command", TOPIC_DRIVER_WHEEL_COMMAND, driver_command),
    ):
        if command is None:
            warnings.append(f"{topic} published no messages: the rover was never commanded this run")
            continue
        figure = figures.six_wheel_small_multiples(
            command.times_s, command.velocity_radps, f"{label} velocity (rad/s)"
        )
        sections.append(
            f"<section class='plot-section'><h2>{html.escape(label)}</h2>"
            f"{html.figure_to_fragment(figure, f'wheel-cmd-{label}')}</section>"
        )

    # Slip: applied ratio, raw observation, Kalman feedback.
    slip_ratios = context.bag.wheel_scalars.get(TOPIC_SLIP_RATIOS)
    slip_observation = context.bag.wheel_scalars.get(TOPIC_SLIP_OBSERVATION)
    kalman_slip = context.bag.wheel_scalars.get(TOPIC_KALMAN_SLIP)
    if slip_ratios is not None:
        figure = figures.six_wheel_small_multiples(
            slip_ratios.times_s, slip_ratios.values, "Applied slip ratio"
        )
        sections.append(
            "<section class='plot-section'><h2>Applied slip ratio (used in rolling-constraint solve)</h2>"
            f"{html.figure_to_fragment(figure, 'wheel-slip-applied')}</section>"
        )
    if slip_observation is not None:
        figure = figures.six_wheel_small_multiples(
            slip_observation.times_s, slip_observation.values, "Observed slip ratio (NaN = not observed)"
        )
        sections.append(
            "<section class='plot-section'><h2>Raw slip observation</h2>"
            f"{html.figure_to_fragment(figure, 'wheel-slip-observed')}</section>"
        )
    elif estimate_slip:
        # Only unexpected (and worth a warning) when the node was actually
        # configured to compute these; the info card above already
        # explains the estimate_slip_from_visual=false case.
        warnings.append(
            f"{TOPIC_SLIP_OBSERVATION} published no messages despite "
            "estimate_slip_from_visual being true"
        )
    if kalman_slip is not None:
        sections.append(
            figures.empty_state_card_html(
                "The Kalman filter's wheel_slip_ratio topic always publishes "
                "a neutral all-zero vector; slip is no longer a fused ESKF "
                "state. This is a compatibility stub, not a measurement.",
                severity="warning",
            )
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
        position_error = metrics.position_error_by_axis(estimate.position_m, position, valid)
        norm_3d_error = np.where(
            valid, metrics.norm_3d(np.nan_to_num(position_error, nan=0.0)), np.nan
        )
        velocity_error = metrics.velocity_error(
            estimate.linear_velocity_mps, linear_velocity, valid
        )
        position_summary = metrics.summarize_errors(norm_3d_error)
        summary_stats["3-D position RMSE"] = f"{position_summary.rmse:.3f} m"
        rate = metrics.sample_rate_hz(estimate.times_s)
        summary_stats["Sample rate"] = f"{rate:.1f} Hz" if rate else "n/a"

        bracket_age = alignment.bracket_width_s(estimate.times_s, truth.times_s)
        summary_stats["Max truth alignment age"] = (
            f"{np.max(bracket_age[np.isfinite(bracket_age)]):.3f} s"
            if np.any(np.isfinite(bracket_age))
            else "n/a"
        )

        position_figure = figures.three_axis_time_series(
            estimate.times_s, [("Position error", position_error, style.COLOR_ERROR)], ("X", "Y", "Z"), "m"
        )
        velocity_figure = figures.three_axis_time_series(
            estimate.times_s, [("Body velocity error", velocity_error, style.COLOR_ERROR)], ("X", "Y", "Z"), "m/s"
        )
        trajectory_figure = figures.trajectory_xy(
            [
                ("Ground truth", truth.position_m, style.COLOR_GROUND_TRUTH),
                ("Wheel estimate", estimate.position_m, style.COLOR_PRIMARY_ESTIMATE),
            ]
        )
        sections.append(
            "<section class='plot-section'><h2>Position error vs ground truth</h2>"
            f"{html.figure_to_fragment(position_figure, 'wheel-position-error')}</section>"
            "<section class='plot-section'><h2>Body velocity error vs ground truth</h2>"
            f"{html.figure_to_fragment(velocity_figure, 'wheel-velocity-error')}</section>"
            "<section class='plot-section'><h2>Trajectory (X-Y)</h2>"
            f"{html.figure_to_fragment(trajectory_figure, 'wheel-trajectory')}</section>"
        )
    else:
        warnings.append(
            f"Cannot compare against ground truth: {TOPIC_GROUND_TRUTH} or "
            f"{TOPIC_ESTIMATE} published no messages"
        )

    body = "".join(sections) if sections else figures.empty_state_card_html(
        "No wheel odometry data available in this run."
    )
    page_html = html.render_page("Wheel Odometry", "wheel_odometry.html", body, warnings)
    return ReportPage(
        filename="wheel_odometry.html",
        title="Wheel Odometry",
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
        description="Generate the standalone wheel-odometry report page.",
        parents=[build_common_parser()],
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: builds the report context for one test run and
             writes just the wheel-odometry page and shared assets.

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
        page = generate_wheel_odometry_report(context)
        html.write_report([page], context.output_dir, context)
    except Exception as error:  # noqa: BLE001 -- top-level CLI error boundary
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(f"wrote {context.output_dir / page.filename}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
