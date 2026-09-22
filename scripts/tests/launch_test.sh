#!/bin/bash
#
# @File:         launch_test.sh
#
# @Brief:        Checks for the multisystem launcher and run artifact layout.
#                Never starts Gazebo, never requires a build to exist.
#
# @Date:         19/09/2026
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
LAUNCH_SH="$ROOT/scripts/launch_simulator.sh"
LAUNCH_PY="$ROOT/launch/alpha_launch.py"
TEST_COMMON="$ROOT/scripts/tests/test_common.sh"

fail() {
  echo "[launch_test] FAIL: $1" >&2
  exit 1
}

pass() {
  echo "[launch_test] OK: $1"
}

# ---------------------------------------------------------------------------- #
# 1. Syntax: bash -n, py_compile, shellcheck
# ---------------------------------------------------------------------------- #

bash -n "$LAUNCH_SH" || fail "bash -n launch_simulator.sh"
bash -n "$TEST_COMMON" || fail "bash -n test_common.sh"
bash -n "$0" || fail "bash -n self"
pass "bash -n"

python3 -m py_compile "$LAUNCH_PY" || fail "py_compile alpha_launch.py"
pass "py_compile"

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck -S warning "$LAUNCH_SH" "$TEST_COMMON" "$0" || fail "shellcheck"
  pass "shellcheck"
else
  echo "[launch_test] SKIP: shellcheck not on PATH"
fi

# ---------------------------------------------------------------------------- #
# 2. Explicit file-exists list
# ---------------------------------------------------------------------------- #

required=(
  "worlds/lunar_surface.sdf"
  "src/systems/alpha/alpha_model/model.sdf"
  "src/systems/alpha/alpha_model/model.config"
  "launch/alpha_launch.py"
  "config/alpha_ros_gz_bridge.yaml"
  "parameters/systems/alpha/alpha_drivers/alpha_drivers.yaml"
  "parameters/systems/alpha/alpha_localisation/alpha_kalman_filter.yaml"
  "parameters/systems/alpha/alpha_localisation/ground_truth.yaml"
  "parameters/systems/alpha/alpha_localisation/inertial_odometry.yaml"
  "parameters/systems/alpha/alpha_localisation/visual_odometry.yaml"
  "parameters/systems/alpha/alpha_localisation/wheel_odometry.yaml"
  "parameters/systems/alpha/alpha_control/ackermann_controller.yaml"
  "src/systems/alpha/alpha.rviz"
)
for rel in "${required[@]}"; do
  [ -f "$ROOT/$rel" ] || fail "missing $rel"
done
pass "file-exists list (${#required[@]} files)"

# ---------------------------------------------------------------------------- #
# 3. YAML: bridge exact-set + per-file node-key asserts
# ---------------------------------------------------------------------------- #

python3 - "$ROOT" <<'PYEOF' || exit 1
import sys
import yaml
from pathlib import Path

root = Path(sys.argv[1])

bridge = yaml.safe_load(open(root / "config/alpha_ros_gz_bridge.yaml"))
names = sorted(e["ros_topic_name"] for e in bridge)
expected = sorted([
    "/clock",
    "/alpha/drivers/cmd/wheel_joint_states",
    "/alpha/drivers/imu",
    "/alpha/drivers/joint_states",
    "/alpha/drivers/ground_truth/odometry",
    "/alpha/drivers/loccam/left",
    "/alpha/drivers/loccam/right",
    "/alpha/drivers/navcam/left",
    "/alpha/drivers/navcam/right",
])
assert names == expected, f"bridge set mismatch: {names}"
for entry in bridge:
    if entry["ros_topic_name"] == "/clock":
        continue
    assert entry["ros_topic_name"].startswith("/alpha/drivers/"), entry
    assert entry["gz_topic_name"].startswith("/alpha/drivers/"), entry
print("[launch_test] OK: bridge exact 9-name set")

checks = [
    ("parameters/systems/alpha/alpha_drivers/alpha_drivers.yaml", "alpha_driver_node"),
    ("parameters/systems/alpha/alpha_localisation/alpha_kalman_filter.yaml", "continuous_ekf"),
    ("parameters/systems/alpha/alpha_localisation/ground_truth.yaml", "ground_truth"),
    ("parameters/systems/alpha/alpha_localisation/inertial_odometry.yaml", "inertial_odometry"),
    ("parameters/systems/alpha/alpha_localisation/visual_odometry.yaml", "visual_odometry"),
    ("parameters/systems/alpha/alpha_localisation/wheel_odometry.yaml", "wheel_odometry"),
    ("parameters/systems/alpha/alpha_control/ackermann_controller.yaml", "ackermann_controller"),
]
for rel, key in checks:
    data = yaml.safe_load(open(root / rel))
    assert isinstance(data, dict) and key in data, f"{rel} missing {key}"
    assert isinstance(data[key].get("ros__parameters"), dict), f"{rel} empty params"
print("[launch_test] OK: 7 param node keys")
PYEOF

# ---------------------------------------------------------------------------- #
# 4. --help exits 0 WITHOUT colcon (stub colcon on PATH to prove)
# ---------------------------------------------------------------------------- #

stub_dir="$(mktemp -d --tmpdir launch_test_stub.XXXXXX)"
trap 'rm -rf -- "$stub_dir"' EXIT
printf '#!/bin/bash\necho "stub colcon should never run for --help" >&2\nexit 99\n' > "$stub_dir/colcon"
chmod +x "$stub_dir/colcon"
PATH="$stub_dir:$PATH" "$LAUNCH_SH" --help >/dev/null || fail "--help non-zero with stub colcon"
pass "--help exits 0 without colcon"

# ---------------------------------------------------------------------------- #
# 5. Run artifact layout and parameter snapshot
# ---------------------------------------------------------------------------- #

run_root="$stub_dir/run-root"
mkdir -p "$run_root/parameters/example"
printf 'snapshot-marker\n' > "$run_root/parameters/example/marker.txt"
bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  create_test_run
  [[ "$(basename "$TEST_RUN_DIR")" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}$ ]]
  [ -f "$TEST_RUN_DIR/parameters/example/marker.txt" ]
  [ -d "$TEST_RUN_DIR/ros/build_logs" ]
  [ -d "$TEST_RUN_DIR/ros/logs" ]
  [ -d "$TEST_RUN_DIR/ros/bags" ]
  [ -d "$TEST_RUN_DIR/logs" ]
  [ -d "$TEST_RUN_DIR/post_processing" ]
  [ "$COLCON_LOG_PATH" = "$TEST_RUN_DIR/ros/build_logs" ]
  [ "$ROS_LOG_DIR" = "$TEST_RUN_DIR/ros/logs" ]
  [ "$LUNAR_SIMULATOR_ROSBAG_DIR" = "$TEST_RUN_DIR/ros/bags" ]
  start_terminal_capture
  [ "$INTERACTIVE_RUN" -eq 0 ]
  printf "terminal-capture-marker\n"
  timestamp_test_run_dir="$TEST_RUN_DIR"
  rename_test_run ""
  [ "$TEST_RUN_DIR" = "$timestamp_test_run_dir" ]
  if rename_test_run "invalid/name"; then
    exit 1
  fi
  [ "$TEST_RUN_DIR" = "$timestamp_test_run_dir" ]
  rename_test_run "straight-drive"
  [[ "$(basename "$TEST_RUN_DIR")" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{2}-straight-drive$ ]]
  [ "$COLCON_LOG_PATH" = "$TEST_RUN_DIR/ros/build_logs" ]
  [ "$ROS_LOG_DIR" = "$TEST_RUN_DIR/ros/logs" ]
  [ "$LUNAR_SIMULATOR_ROSBAG_DIR" = "$TEST_RUN_DIR/ros/bags" ]
  printf "terminal-capture-after-rename\n"
' _ "$LAUNCH_SH" "$run_root" >/dev/null || fail "test-run artifact layout"
run_directories=("$run_root"/test_runs/*)
[ "${#run_directories[@]}" -eq 1 ] || fail "test-run directory count"
terminal_log="${run_directories[0]}/logs/terminal.txt"
for _ in {1..20}; do
  grep -q "terminal-capture-marker" "$terminal_log" 2>/dev/null && break
  sleep 0.1
done
grep -q "terminal-capture-marker" "$terminal_log" || fail "terminal capture content"
grep -q "terminal-capture-after-rename" "$terminal_log" || fail "renamed terminal capture content"
pass "test-run artifact layout"

# ---------------------------------------------------------------------------- #
# 6. Three negative cases, all non-zero without Gazebo
# ---------------------------------------------------------------------------- #

"$LAUNCH_SH" --no-such-flag >/dev/null 2>&1 && fail "unknown flag accepted" || pass "unknown flag non-zero"
"$LAUNCH_SH" a b c >/dev/null 2>&1 && fail "3 positionals accepted" || pass ">2 positionals non-zero"
"$LAUNCH_SH" lunar_surface no_such_system_xyz >/dev/null 2>&1 && fail "bad system accepted" || pass "bad system non-zero"

# ---------------------------------------------------------------------------- #
# 7. Rename gates over working-tree text.
#    rg excludes build/install/log/.git/__pycache__/*.pyc; *.bak included.
#    This file itself is excluded from every gate (it must name each pattern
#    to check it, so an unexcluded self-search would always self-trigger).
# ---------------------------------------------------------------------------- #

# Build patterns without writing any forbidden literal verbatim in this file:
# each assignment below splits the literal so this file's own text never
# contains the searched string as a contiguous substring.
OLD_NS="alpha""_system"
OLD_LAUNCH="${OLD_NS}""_launch.py"
OLD_RVIZ="lunar_simulator_test"".rviz"
BARE_LAUNCH="scripts/launch_simulator""[^.]"
OLD_BRIDGE="ros_gz_bridge"".yaml"
OLD_COMMON="common"".yaml"

gate_zero() {
  local desc="$1"
  local pattern="$2"
  local matches
  matches="$(rg --no-messages -N --glob '!build/**' --glob '!install/**' \
    --glob '!log/**' --glob '!.git/**' --glob '!__pycache__/**' \
    --glob '!*.pyc' --glob '!**/launch_test.sh' \
    -e "$pattern" "$ROOT" || true)"
  if [ -n "$matches" ]; then
    echo "$matches" >&2
    fail "$desc (non-zero matches)"
  fi
  pass "$desc"
}

gate_zero "no old namespace" "$OLD_NS"
gate_zero "no old launch filename" "$OLD_LAUNCH"
gate_zero "no old rviz filename" "$OLD_RVIZ"
gate_zero "no bare launcher ref" "$BARE_LAUNCH"
gate_zero "no old shared params" "$OLD_COMMON"

# Bridge filename: every mention must be the new config (or its generic
# SYSTEM-templated form) or live inside a .bak backup.
# Exclude launch_simulator.sh since it references the bridge config variable.
bridge_hits="$(rg --no-messages --glob '!build/**' --glob '!install/**' \
  --glob '!log/**' --glob '!.git/**' --glob '!__pycache__/**' \
  --glob '!*.pyc' --glob '!**/launch_test.sh' --glob '!**/launch_simulator.sh' \
  -e "$OLD_BRIDGE" "$ROOT" || true)"
if [ -z "$bridge_hits" ]; then
  fail "bridge filename unreferenced"
fi
bad_bridge="$(printf '%s\n' "$bridge_hits" | grep -v '\.bak:' | grep -v 'alpha_ros_gz_bridge' | grep -v 'SYSTEM}_ros_gz_bridge' || true)"
if [ -n "$bad_bridge" ]; then
  echo "$bad_bridge" >&2
  fail "bridge filename only new+.bak"
fi
pass "bridge filename only new+.bak"

echo "[launch_test] PASS"
