#!/bin/bash
#
# @File:         test_common.sh
#
# @Brief:        Shared harness sourced by every script in scripts/tests/:
#                starts/stops the simulator via scripts/launch_simulator.sh
#                (never reimplements its Gazebo spawn/pause/unpause
#                sequence), drives the rover, and checks the fused
#                estimate stays close to ground truth. Not executable on
#                its own -- `source` it from a test script.
#
# @Date:         18/09/2026
#

# Every test script sources this after its own `set -euo pipefail`, so this
# file intentionally does not set shell options itself.

# Resolve paths relative to this file so tests run from any directory,
# matching launch_simulator.sh's own approach.
TC_TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TC_ROOT="$(dirname "$(dirname "$TC_TESTS_DIR")")"
TC_LAUNCH_SIMULATOR="$TC_ROOT/scripts/launch_simulator.sh"
TC_WORLD_FILE="$TC_ROOT/worlds/lunar_surface.sdf"
TC_LOG_DIR="$(mktemp -d --tmpdir lunar_simulator_test.XXXXXX)"
TC_LAUNCH_LOG="$TC_LOG_DIR/launch.log"

# Populated by tc::start_simulator; empty until then.
TC_SIMULATOR_PID=""

# Source ROS2 Jazzy and the workspace overlay so `ros2` is on PATH even when
# a test script is invoked directly (not just via another sourced shell).
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$TC_ROOT/install/setup.bash" ]; then
  source "$TC_ROOT/install/setup.bash"
fi
set -u

#!
# @brief          Starts the simulator headless via scripts/launch_simulator.sh
#                 and blocks until Alpha's stack is ready to drive.
#
# Never spawns Gazebo, the bridge, or RViz directly -- delegates entirely
# to scripts/launch_simulator.sh (which itself delegates RViz to
# launch/alpha_launch.py, see src/systems/alpha/alpha.rviz alongside
# AlphaNode.h under src/systems/alpha/) so this harness can't drift out of
# sync with its spawn/pause/unpause sequence, camera-noise patching, or
# RViz config.
#
# @param[in]      enable_rviz    "1" to also open RViz (ground truth path:
#                                green, kalman filter path: red); omitted
#                                or any other value keeps this headless.
#                                Requires a display (DISPLAY or
#                                WAYLAND_DISPLAY) when "1" -- scripts/
#                                launch_simulator.sh fails outright rather
#                                than silently continuing without the
#                                visualization the caller explicitly asked
#                                for.
#
tc::start_simulator() {
  local enable_rviz="${1:-0}"
  local -a launch_args=(--headless)
  if [ "$enable_rviz" = "1" ]; then
    if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
      echo "[test] ERROR: --rviz requires a display (DISPLAY or WAYLAND_DISPLAY is unset)" >&2
      exit 1
    fi
    echo "[test] starting simulator with RViz (ground truth path: green, kalman filter path: red) (log: $TC_LAUNCH_LOG)..."
    launch_args+=(--rviz)
  else
    echo "[test] starting simulator (log: $TC_LAUNCH_LOG)..."
  fi
  "$TC_LAUNCH_SIMULATOR" "${launch_args[@]}" > "$TC_LAUNCH_LOG" 2>&1 &
  TC_SIMULATOR_PID=$!

  # inertial_odometry logs this once its startup calibration window
  # completes, which is the last node-side readiness step: by then Alpha's
  # whole stack (all of alpha_node's constructor delay, every subscription,
  # and every publisher) is already up. Bounded so a genuinely broken
  # launch fails the test instead of hanging it.
  local waited=0
  local timeout_s=60
  while ! grep -q "calibration complete" "$TC_LAUNCH_LOG" 2>/dev/null; do
    if ! kill -0 "$TC_SIMULATOR_PID" 2>/dev/null; then
      echo "[test] ERROR: scripts/launch_simulator.sh exited before becoming ready; see $TC_LAUNCH_LOG" >&2
      exit 1
    fi
    if [ "$waited" -ge "$timeout_s" ]; then
      echo "[test] ERROR: simulator did not become ready within ${timeout_s}s; see $TC_LAUNCH_LOG" >&2
      exit 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[test] simulator ready after ${waited}s"

  # A stray publisher on the bare /clock topic from an unrelated ROS 2
  # session on this machine silently corrupts every use_sim_time node's
  # clock without any error -- see CLAUDE.md's Simulator launch mechanics
  # note. Warn rather than fail outright, since this is an environment
  # issue, not something this test suite can fix.
  local clock_publishers
  clock_publishers=$(timeout 5 ros2 topic info /clock --verbose 2>/dev/null \
    | grep -c "Endpoint type: PUBLISHER" || true)
  if [ "$clock_publishers" != "1" ]; then
    echo "[test] WARNING: /clock has $clock_publishers publishers, expected 1." >&2
    echo "[test]          Results may be unreliable -- set a distinct ROS_DOMAIN_ID" >&2
    echo "[test]          and re-run if this test fails unexpectedly." >&2
  fi
}

#!
# @brief          Publishes one body-velocity command.
#
# @param[in]      linear_x_mps   Forward speed in m/s.
# @param[in]      angular_z_radps Yaw rate in rad/s.
#
tc::publish_twist() {
  local linear_x_mps="$1"
  local angular_z_radps="$2"

  ros2 topic pub --once /alpha/control/cmd/velocity geometry_msgs/msg/Twist \
    "{linear: {x: $linear_x_mps, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $angular_z_radps}}" \
    > /dev/null
}

#! @brief Commands zero body velocity; there is no command timeout in this
#         stack (see CLAUDE.md's Wheel command convention), so this is the
#         only way to actually stop the rover.
tc::stop_rover() {
  echo "[test] stopping rover..."
  tc::publish_twist 0.0 0.0
}

#!
# @brief          Drives for a fixed duration, sampling the fused estimate
#                 against ground truth periodically and failing the test if
#                 either ever exceeds its configured bound.
#
# Deliberately loose bounds: this checks for a regression class of bug (the
# EKF diverging or running away, e.g. the wheel-odometry rotational-slip
# issue documented in CLAUDE.md's Key Invariants), not for tight
# accuracy -- ordinary fusion lag/noise during active driving is expected
# and should not fail this check.
#
# @param[in]      duration_s        Total sampling duration in seconds.
# @param[in]      interval_s        Seconds between samples.
# @param[in]      max_horiz_err_m   Maximum acceptable horizontal (x/y)
#                                   position error in metres.
# @param[in]      max_z_err_m       Maximum acceptable |z| position error
#                                   in metres.
# @return         0 if every sample stayed within bounds, 1 otherwise.
#
tc::check_bounded_error() {
  local duration_s="$1"
  local interval_s="$2"
  local max_horiz_err_m="$3"
  local max_z_err_m="$4"
  if ! [[ "$duration_s" =~ ^[1-9][0-9]*$ ]]; then
    echo "[test] ERROR: duration_s must be a positive integer (got '$duration_s')" >&2
    return 1
  fi
  if ! [[ "$interval_s" =~ ^[1-9][0-9]*$ ]]; then
    echo "[test] ERROR: interval_s must be a positive integer (got '$interval_s')" >&2
    return 1
  fi
  local samples=$((duration_s / interval_s))

  echo "[test] checking fused-estimate error for ${duration_s}s (every ${interval_s}s, limits: horiz<=${max_horiz_err_m}m z<=${max_z_err_m}m)..."

  python3 - "$samples" "$interval_s" "$max_horiz_err_m" "$max_z_err_m" <<'PYEOF'
import re
import subprocess
import sys
import time

samples = int(sys.argv[1])
interval_s = float(sys.argv[2])
max_horiz_m = float(sys.argv[3])
max_z_m = float(sys.argv[4])

POSITION_PATTERN = re.compile(
    r"position:\s*\n\s*x:\s*([\-0-9.e]+)\s*\n\s*y:\s*([\-0-9.e]+)\s*\n\s*z:\s*([\-0-9.e]+)"
)


def read_position(topic):
    """Reads one nav_msgs/Odometry message's position via ros2 topic echo.

    Shelling out to the CLI rather than using rclpy keeps this test
    harness dependency-free and consistent with how this harness already
    inspects live topics via the ROS CLI.
    """
    result = subprocess.run(
        ["ros2", "topic", "echo", topic, "--once"],
        capture_output=True,
        text=True,
        timeout=6,
    )
    match = POSITION_PATTERN.search(result.stdout)
    return tuple(float(component) for component in match.groups()) if match else None


has_failed = False

for sample_index in range(samples):
    ground_truth_position = read_position("/alpha/localisation/ground_truth/odometry")
    fused_position = read_position("/alpha/localisation/kalman_filter/odometry")
    elapsed_s = sample_index * interval_s

    if ground_truth_position is None or fused_position is None:
        print(f"t+{elapsed_s:3.0f}s  MISSING DATA  [FAIL]")
        has_failed = True
        time.sleep(interval_s)
        continue

    horizontal_error_m = (
        (fused_position[0] - ground_truth_position[0]) ** 2
        + (fused_position[1] - ground_truth_position[1]) ** 2
    ) ** 0.5
    z_error_m = fused_position[2] - ground_truth_position[2]

    sample_failed = horizontal_error_m > max_horiz_m or abs(z_error_m) > max_z_m
    has_failed = has_failed or sample_failed
    status = "FAIL" if sample_failed else "OK"

    print(
        f"t+{elapsed_s:3.0f}s  horiz_err={horizontal_error_m:8.4f} m  "
        f"z_err={z_error_m:+8.4f} m  [{status}]"
    )

    time.sleep(interval_s)

sys.exit(1 if has_failed else 0)
PYEOF
}

#! @brief Stops the rover and tears down everything tc::start_simulator
#         started. Registered as an EXIT trap by every test script so it
#         still runs on a failed check or an interrupted run.
tc::cleanup() {
  local exit_code=$?

  echo "[test] cleaning up..."
  tc::stop_rover 2>/dev/null || true

  # Ask scripts/launch_simulator.sh to run its own cleanup trap first.
  if [ -n "$TC_SIMULATOR_PID" ] && kill -0 "$TC_SIMULATOR_PID" 2>/dev/null; then
    kill -TERM "$TC_SIMULATOR_PID" 2>/dev/null || true
    sleep 2
  fi

  # Fall back to a direct, narrowly-scoped kill of anything left over --
  # scripts/launch_simulator.sh's own trap does not always fully clean up
  # every descendant process (observed repeatedly during development), so
  # this harness cannot rely on it alone.
  pkill -9 -f "gz sim -s -v4 $TC_WORLD_FILE" 2>/dev/null || true
  pkill -9 -f "install/lunar_simulator/lib/lunar_simulator/alpha_node" 2>/dev/null || true
  # Matches both the short-lived "ros2 run ros_gz_bridge parameter_bridge"
  # wrapper and the long-lived compiled binary it execs
  # (.../lib/ros_gz_bridge/parameter_bridge, a path with no space) -- a
  # pattern requiring the space matched only the former and left the
  # latter running (confirmed during development).
  pkill -9 -f "parameter_bridge" 2>/dev/null || true
  # rviz2 is launched as a Node inside launch/alpha_launch.py now (see
  # src/systems/alpha/alpha.rviz's new home under src/systems/alpha/),
  # not spawned directly by this harness, but gets the same belt-and-
  # suspenders treatment as the two processes above.
  pkill -9 -f "rviz2.*alpha.rviz" 2>/dev/null || true

  rm -rf -- "$TC_LOG_DIR"

  exit "$exit_code"
}
