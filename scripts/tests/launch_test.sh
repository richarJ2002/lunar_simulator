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

# ---------------------------------------------------------------------------- #
# 8. Recording: argument parsing, topic-profile assembly, registry sync
# ---------------------------------------------------------------------------- #

bash -c '
  set -euo pipefail
  source "$1"
  parse_arguments --record-images
  [ "$RECORD_IMAGES" -eq 1 ]
  assemble_record_topics
  [ "${#RECORD_TOPICS[@]}" -eq 23 ]
  RECORD_IMAGES=0
  assemble_record_topics
  [ "${#RECORD_TOPICS[@]}" -eq 20 ]
' _ "$LAUNCH_SH" >/dev/null || fail "recording argument parsing / topic assembly"
pass "recording argument parsing / topic assembly"

# scripts/launch_simulator.sh's CORE_RECORD_TOPICS/IMAGE_RECORD_TOPICS and
# post_processing/python_tools/bag/topic_registry.py's CORE_TOPICS/
# IMAGE_TOPICS are two independent, by-hand-kept-in-sync lists (see both
# files' own comments on why they are not shared at runtime). Prove they
# have not drifted rather than trusting the comment alone.
core_from_python="$(PYTHONPATH="$ROOT/post_processing" python3 -m python_tools.bag.topic_registry --profile core | sort)"
core_from_bash="$(bash -c 'source "$1"; assemble_record_topics; printf "%s\n" "${RECORD_TOPICS[@]}"' _ "$LAUNCH_SH" | sort)"
[ "$core_from_python" = "$core_from_bash" ] || fail "core topic list matches python_tools registry"
pass "core topic list matches python_tools registry"

images_from_python="$(PYTHONPATH="$ROOT/post_processing" python3 -m python_tools.bag.topic_registry --profile core+images | sort)"
images_from_bash="$(bash -c 'source "$1"; RECORD_IMAGES=1; assemble_record_topics; printf "%s\n" "${RECORD_TOPICS[@]}"' _ "$LAUNCH_SH" | sort)"
[ "$images_from_python" = "$images_from_bash" ] || fail "core+images topic list matches python_tools registry"
pass "core+images topic list matches python_tools registry"

# ---------------------------------------------------------------------------- #
# 9. Recorder lifecycle: bag destination, manifest, PID-scoped SIGTERM+wait,
#    bounded retry/failure, and the launcher's own INT/TERM trap handling.
# ---------------------------------------------------------------------------- #
# 2026-09-23 fix: a real manual `Ctrl+C` acceptance test found the launcher
# had no trap for SIGINT/SIGTERM (only EXIT), and that the recorder's own
# shutdown signal (SIGINT) is provably ignored by the real `ros2 bag
# record` process (a background job in a job-control-disabled script has
# SIGINT set to SIG_IGN by bash before exec, and rclpy/CPython never
# re-arms an inherited SIG_IGN) -- confirmed directly against two real
# orphaned recorders left over from that failure (`/proc/<pid>/status`
# showed SIGINT in SigIgn, SIGTERM in SigCgt; SIGTERM produced a fully
# finalized, `ros2 bag info`-readable bag). The previous dynamic test here
# was SKIPped in this sandbox because it tried to deliver SIGINT to a
# background child of a nested `bash -c`, which this sandbox cannot do --
# but that was never only a sandbox artifact: it was quietly exercising
# (and hiding) the exact same real-world "background jobs ignore SIGINT"
# behavior that caused the production bug, just misattributed entirely to
# the sandbox. This section replaces SIGINT with SIGTERM everywhere a
# signal actually needs to reach a background process, which this sandbox
# (confirmed directly, five repeated trials, both nested and top-level)
# delivers reliably, so nothing here is capability-skipped any more except
# the one genuinely sandbox-specific gap noted below.

cat > "$stub_dir/ros2" <<'STUBEOF'
#!/bin/bash
# Stub `ros2`: only understands `ros2 bag record ...`, just enough to
# exercise start_recorder()/stop_recorder() without a real ROS graph.
# Deliberately installs NO trap for SIGINT at all -- unlike the previous
# version of this stub, which trapped both INT and TERM and so could never
# have caught the real "SIGINT is ignored" bug even with perfect signal
# delivery. Only SIGTERM is ever handled, faithfully matching the real
# `ros2 bag record` process this stub stands in for.
if [ "$1" = "bag" ] && [ "$2" = "record" ]; then
  shift 2
  output_dir=""
  while [ $# -gt 0 ]; do
    case "$1" in
      -o) output_dir="$2"; shift 2 ;;
      *) shift ;;
    esac
  done
  # A destination containing one of these sentinels simulates a recorder
  # that fails to start, ignores its first shutdown signal, or never stops
  # at all, without needing separate stub scripts.
  if [[ "$output_dir" == *STUB_RECORDER_FAIL* ]]; then
    exit 7
  fi
  mkdir -p "$output_dir"
  if [[ "$output_dir" == *STUB_RECORDER_NEVER_EXITS* ]]; then
    trap '' TERM
  elif [[ "$output_dir" == *STUB_RECORDER_IGNORE_FIRST_TERM* ]]; then
    term_count=0
    on_term() {
      term_count=$((term_count + 1))
      [ "$term_count" -ge 2 ] && { touch "$output_dir/metadata.yaml"; exit 0; }
    }
    trap on_term TERM
  else
    # Only write metadata.yaml once actually asked to shut down, so a test
    # can tell a genuinely awaited shutdown from a fire-and-forget one.
    trap 'touch "$output_dir/metadata.yaml"; exit 0' TERM
  fi
  while true; do sleep 0.1; done
fi
echo "stub ros2: unhandled invocation: $*" >&2
exit 99
STUBEOF
chmod +x "$stub_dir/ros2"

# Static proof this always holds: stop_recorder() shall target only the
# tracked PID with SIGTERM (SIGINT is provably ignored by the real
# recorder, per the evidence above -- sending it would be a silent no-op
# forever, not a stricter or more polite request), never a broad pkill
# sweep (this repo's own long regression scripts use
# `pkill -9 -f parameter_bridge` for cleanup, which would risk killing an
# unrelated recording on this machine).
grep -qF 'kill -TERM "$recorder_pid"' "$LAUNCH_SH" \
  || fail "stop_recorder must SIGTERM the tracked recorder PID"
if grep -Eq 'kill -(SIGINT|INT)[^|]*RECORDER_PID' "$LAUNCH_SH"; then
  fail "stop_recorder must not send SIGINT to the recorder (confirmed permanently ignored by the real process)"
fi
if grep -Eq 'pkill[^|;]*record' "$LAUNCH_SH"; then
  fail "recorder cleanup must not use a broad pkill"
fi
pass "stop_recorder signals only the tracked recorder PID via SIGTERM (no SIGINT, no broad pkill)"

recording_root="$(mktemp -d --tmpdir launch_test_recording.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 20 bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags"
  RECORDER_SHUTDOWN_TIMEOUT_S=3
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"

  start_recorder
  [ "$BAG_DESTINATION" = "$LUNAR_SIMULATOR_ROSBAG_DIR/localisation" ]
  [ -d "$BAG_DESTINATION" ]
  [ -f "$LUNAR_SIMULATOR_ROSBAG_DIR/manifest.json" ]
  python3 -c "
import json, sys
manifest = json.load(open(sys.argv[1]))
assert manifest[\"world\"] == \"lunar_surface\", manifest
assert manifest[\"system\"] == \"alpha\", manifest
assert manifest[\"recording_profile\"] == \"core\", manifest
assert manifest[\"storage_identifier\"] == \"mcap\", manifest
assert len(manifest[\"topics\"]) == 20, manifest
assert manifest[\"bag_destination\"].endswith(\"/localisation\"), manifest
" "$LUNAR_SIMULATOR_ROSBAG_DIR/manifest.json"
  # The recorder is still alive and has not yet finalized its metadata.
  kill -0 "$RECORDER_PID"
  [ ! -f "$BAG_DESTINATION/metadata.yaml" ]

  # stop_recorder() shall block until the recorder has actually exited and
  # confirm metadata.yaml, not just fire the signal and move on.
  stop_recorder
  [ -f "$BAG_DESTINATION/metadata.yaml" ]
  [ -z "$RECORDER_PID" ]
  # Calling it again with no recorder running is a safe no-op.
  stop_recorder
' _ "$LAUNCH_SH" "$ROOT" "$recording_root" >/dev/null || fail "recorder start/stop lifecycle"
pass "recorder start/stop lifecycle (successful SIGTERM finalization)"
rm -rf -- "$recording_root"

# "recorder already exited" -- stop_recorder() must not error, and must
# still confirm metadata.yaml, when the process is already gone before it
# is ever called.
already_exited_root="$(mktemp -d --tmpdir launch_test_recording_exited.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 10 bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags"
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  start_recorder
  kill -TERM "$RECORDER_PID"
  for _ in $(seq 1 50); do kill -0 "$RECORDER_PID" 2>/dev/null || break; sleep 0.1; done
  ! kill -0 "$RECORDER_PID" 2>/dev/null
  stop_recorder
  [ -z "$RECORDER_PID" ]
' _ "$LAUNCH_SH" "$ROOT" "$already_exited_root" >/dev/null || fail "recorder already exited"
pass "stop_recorder handles a recorder that already exited before it was called"
rm -rf -- "$already_exited_root"

# "recorder ignoring the first attempted signal" -- the retry must succeed.
retry_root="$(mktemp -d --tmpdir launch_test_recording_retry.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 15 bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags-STUB_RECORDER_IGNORE_FIRST_TERM"
  RECORDER_SHUTDOWN_TIMEOUT_S=3
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  start_recorder
  stop_recorder
  [ -f "$BAG_DESTINATION/metadata.yaml" ]
' _ "$LAUNCH_SH" "$ROOT" "$retry_root" >/dev/null || fail "recorder retry after ignored first signal"
pass "stop_recorder retries and succeeds when the recorder ignores the first SIGTERM"
rm -rf -- "$retry_root"

# Bounded failure: a recorder that never exits must not hang the launcher
# indefinitely, must not be SIGKILLed as an automatic "recovery", and must
# be reported as a clear, non-zero failure identifying the unfinalized bag.
never_exits_root="$(mktemp -d --tmpdir launch_test_recording_never.XXXXXX)"
start_time=$(date +%s)
PATH="$stub_dir:$PATH" timeout -k 5 15 bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags-STUB_RECORDER_NEVER_EXITS"
  RECORDER_SHUTDOWN_TIMEOUT_S=2
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  start_recorder
  survivor_pid="$RECORDER_PID"
  if stop_recorder; then
    echo "BUG: stop_recorder reported success for a recorder that never exited" >&2
    exit 1
  fi
  [ -n "$RECORDER_PID" ]
  [ "$RECORDER_PID" = "$survivor_pid" ]
  kill -0 "$survivor_pid"
  [ ! -f "$BAG_DESTINATION/metadata.yaml" ]
  kill -9 "$survivor_pid" 2>/dev/null || true
' _ "$LAUNCH_SH" "$ROOT" "$never_exits_root" >/dev/null || fail "recorder failure-to-stop reporting"
elapsed=$(( $(date +%s) - start_time ))
[ "$elapsed" -lt 15 ] || fail "recorder failure path did not stay bounded (${elapsed}s)"
pass "stop_recorder reports a bounded, non-zero failure without SIGKILL when the recorder never exits (${elapsed}s)"
rm -rf -- "$never_exits_root"

# ---------------------------------------------------------------------------- #
# 9b. The launcher's own SIGINT/SIGTERM trap handling (not the recorder's).
# ---------------------------------------------------------------------------- #

# Logic check via direct invocation for both signals: handle_termination_
# signal() is a plain function, so calling it directly still exercises its
# real logic (128+signum exit-code computation, logging, and triggering
# finalize_test_run()/stop_recorder() via `exit`) even where live kernel
# signal delivery cannot be used in this sandbox (SIGINT -- see below).
for signal_name in INT TERM; do
  handler_root="$(mktemp -d --tmpdir launch_test_handler.XXXXXX)"
  expected_code=$((128 + $(kill -l "$signal_name")))
  PATH="$stub_dir:$PATH" timeout -k 5 10 bash -c '
    set -uo pipefail
    source "$1"
    ROOT="$2"
    RUN_LOGS_DIR="$3/logs"
    LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags"
    RECORDER_SHUTDOWN_TIMEOUT_S=3
    mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
    QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
    trap finalize_test_run EXIT
    start_recorder
    handle_termination_signal "$4"
  ' _ "$LAUNCH_SH" "$ROOT" "$handler_root" "$signal_name" \
    >/dev/null 2>"$handler_root/stderr.log" && actual_code=0 || actual_code=$?
  [ "$actual_code" -eq "$expected_code" ] \
    || fail "handle_termination_signal $signal_name exit code ($actual_code != $expected_code)"
  [ -f "$handler_root/bags/localisation/metadata.yaml" ] \
    || fail "handle_termination_signal $signal_name did not finalize the recorder"
  grep -q "Received SIG${signal_name}" "$handler_root/stderr.log" \
    || fail "handle_termination_signal $signal_name did not log its own shutdown message"
  pass "handle_termination_signal $signal_name computes exit $expected_code and finalizes the recorder"
  rm -rf -- "$handler_root"
done

# Real, live SIGTERM delivery to a backgrounded process blocked in a
# foreground `wait`, mirroring main()'s actual trap-then-wait structure
# end to end -- not a direct function call. Confirmed reliable in this
# sandbox across five repeated trials, both nested-`bash -c` and top-level.
live_root="$(mktemp -d --tmpdir launch_test_live_signal.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 15 bash -c '
  set -uo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags"
  RECORDER_SHUTDOWN_TIMEOUT_S=3
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  trap "handle_termination_signal INT" INT
  trap "handle_termination_signal TERM" TERM
  trap finalize_test_run EXIT
  start_recorder
  sleep 100 &
  GAZEBO_PID=$!
  wait "$GAZEBO_PID"
' _ "$LAUNCH_SH" "$ROOT" "$live_root" >"$live_root/stdout.log" 2>&1 &
outer_pid=$!
for _ in $(seq 1 50); do
  [ -f "$live_root/bags/localisation/manifest.json" ] && break
  sleep 0.1
done
kill -TERM "$outer_pid"
wait "$outer_pid" && actual_code=0 || actual_code=$?
[ "$actual_code" -eq 143 ] || fail "live SIGTERM to launcher: exit code ($actual_code != 143)"
[ -f "$live_root/bags/localisation/metadata.yaml" ] || fail "live SIGTERM to launcher: recorder not finalized"
grep -q "Received SIGTERM" "$live_root/stdout.log" || fail "live SIGTERM to launcher: shutdown message missing"
pass "a real SIGTERM to the launcher, blocked in its foreground wait, cleanly finalizes the recorder (exit 143)"
rm -rf -- "$live_root"

# ---------------------------------------------------------------------------- #
# 9c. A real terminal Ctrl+C: SIGINT delivered to the WHOLE foreground
#     process group, while blocked in a genuine foreground wait, through the
#     launcher's actual start_terminal_capture() tee/process-substitution
#     logging topology.
# ---------------------------------------------------------------------------- #
# 2026-09-23 fix (second pass). A real manual Ctrl+C during normal operation
# produced exit status 141 (128+SIGPIPE): no "Received SIGINT" message, no
# recorder-shutdown message, no metadata.yaml, and a surviving orphaned
# recorder. This proved the first pass's fix (9b above) was necessary but
# incomplete -- and proved the tests that "passed" before that manual run
# were not exercising the real bug: neither test above calls
# start_terminal_capture() at all, and the live-signal test signals only the
# outer bash -c's own PID, never its process group, so neither could ever
# have caught a bug that depends on both.
#
# Root cause, confirmed here with real process-group SIGINT delivery (not a
# direct function call, not SIGTERM substituted for SIGINT): `tee` in
# start_terminal_capture() is started via process substitution (`>(...)`),
# not `cmd &`, so unlike the backgrounded recorder it is NOT immune to
# SIGINT -- confirmed via /proc/<tee-pid>/status showing SIGINT at its
# default (non-ignored) disposition before any signal is sent. A terminal
# Ctrl+C sends SIGINT to the entire foreground process group at once (no job
# control is enabled anywhere in this script, so every child -- gazebo,
# recorder, ROS nodes, and tee -- shares one process group with the script
# itself), which kills `tee` almost immediately. Every later write this
# script makes to its own, now pipe-broken, stdout/stderr -- including the
# caught-signal trap's own "Received SIG..." line -- then raises SIGPIPE,
# which is fatal to bash and aborts execution before cleanup runs. Fixed by
# making the tee subshell ignore INT/TERM before exec-ing into `tee` (SIG_IGN
# survives exec, so it stays immune and now exits only on EOF), plus
# `trap '' PIPE` in main() as defense in depth.
#
# This also corrects an earlier NOTE in this file claiming live SIGINT
# delivery to a background process is unavailable in this sandbox. The full
# picture, empirically pinned down while building this test: a `kill -INT`
# aimed at a process group, issued by a sender that is itself already
# nested one process level below the agent shell's own directly-spawned
# command (which describes literally any command running inside this
# script, since `launch_test.sh` is always invoked as its own process), is
# accepted by the kernel (the syscall returns success) but is silently
# never delivered -- confirmed with a controlled pair of trials using an
# identical `setsid`-backed target: sent directly at the top level it
# worked immediately; sent from one script-nesting level deeper it produced
# zero observable effect on any process in the target group, indefinitely.
# A genuine PTY sidesteps this: writing the raw INTR byte (0x03) into a
# pty's master fd makes the *kernel's tty line discipline itself* raise
# SIGINT against the pty's foreground process group -- not a `kill()`
# syscall issued by this script at all, which is exactly what a real
# terminal does on a physical Ctrl+C. Confirmed reliable across repeated
# nested trials (a python3 helper opening the pty, invoked from a bash
# script, invoked from this file), so this test drives the harness through
# a real pty rather than `setsid` + `kill -INT -PGID`.

sigint_root="$(mktemp -d --tmpdir launch_test_sigint.XXXXXX)"
harness_script="$sigint_root/harness.sh"
cat > "$harness_script" <<'HARNESSEOF'
#!/bin/bash
set -uo pipefail
source "$1"
ROOT="$2"
RUN_DIR="$3"
RUN_LOGS_DIR="$RUN_DIR/logs"
LUNAR_SIMULATOR_ROSBAG_DIR="$RUN_DIR/bags"
TEST_RUN_DIR="$RUN_DIR"
RECORDER_SHUTDOWN_TIMEOUT_S=3
mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"

# Mirrors main()'s actual trap/redirection setup and ordering, including the
# tee logging topology the 9b tests above never exercise.
start_terminal_capture
trap finalize_test_run EXIT
trap 'handle_termination_signal INT' INT
trap 'handle_termination_signal TERM' TERM
trap '' PIPE

start_recorder
echo "$RECORDER_PID" > "$RUN_DIR/recorder.pid"
echo "$$" > "$RUN_DIR/main.pid"

# Foreground blocking call, standing in for run_bridge()'s synchronous
# `ros2 run ros_gz_bridge parameter_bridge ...` -- this is where a real
# terminal Ctrl+C actually lands during normal operation, not in
# `wait "$GAZEBO_PID"` (only reached after the bridge itself exits).
sleep 100
HARNESSEOF
chmod +x "$harness_script"

# A real PTY, given a real terminal Ctrl+C's raw INTR byte -- not a `kill`
# syscall (see above for why a direct `kill -INT -PGID` cannot be trusted
# from within this file's own process nesting). This helper opens the pty,
# forks a session leader that makes the pty slave its controlling terminal
# (exactly how a real terminal starts a foreground job), execs the harness
# under it with PATH set for the stub `ros2`, waits for it to report ready,
# writes 0x03, then waits for the harness to exit.
pty_driver="$sigint_root/pty_ctrl_c_driver.py"
cat > "$pty_driver" <<'PYDRIVEREOF'
import os
import sys
import time
import pty
import fcntl
import termios
import signal

harness, run_dir, launch_sh, root, path_env = sys.argv[1:6]

master_fd, slave_fd = pty.openpty()

pid = os.fork()
if pid == 0:
    os.close(master_fd)
    os.setsid()
    fcntl.ioctl(slave_fd, termios.TIOCSCTTY, 0)
    os.dup2(slave_fd, 0)
    os.dup2(slave_fd, 1)
    os.dup2(slave_fd, 2)
    if slave_fd > 2:
        os.close(slave_fd)
    os.environ["PATH"] = path_env
    exit_code_file = os.path.join(run_dir, "exit_code.txt")
    child_cmd = f"'{harness}' '{launch_sh}' '{root}' '{run_dir}'; echo $? > '{exit_code_file}'"
    os.execv("/bin/bash", ["/bin/bash", "-c", child_cmd])
    os._exit(127)

os.close(slave_fd)

main_pid_file = os.path.join(run_dir, "main.pid")
deadline = time.time() + 5
while time.time() < deadline and not os.path.exists(main_pid_file):
    time.sleep(0.1)
if not os.path.exists(main_pid_file):
    print("harness never reached its foreground wait", file=sys.stderr)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    sys.exit(1)

# Only the trivial remainder of start_recorder() runs after main.pid is
# written, before the harness blocks in its foreground sleep.
time.sleep(0.3)

# The kernel's tty line discipline (ISIG), not a kill() syscall from this
# process, raises SIGINT against the pty's foreground process group here --
# exactly what a real terminal does on a physical Ctrl+C.
os.write(master_fd, b"\x03")

exit_code_file = os.path.join(run_dir, "exit_code.txt")
deadline = time.time() + 10
while time.time() < deadline and not os.path.exists(exit_code_file):
    time.sleep(0.1)
if not os.path.exists(exit_code_file):
    print("harness did not exit after a PTY-driven Ctrl+C (hang)", file=sys.stderr)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    sys.exit(1)

try:
    os.waitpid(pid, 0)
except ChildProcessError:
    pass
sys.exit(0)
PYDRIVEREOF

python3 "$pty_driver" "$harness_script" "$sigint_root" "$LAUNCH_SH" "$ROOT" "$stub_dir:$PATH" \
  2>"$sigint_root/driver_stderr.log" \
  || fail "SIGINT process-group test: $(cat "$sigint_root/driver_stderr.log" 2>/dev/null || echo "PTY driver reported failure")"

exit_code=""
[ -f "$sigint_root/exit_code.txt" ] && exit_code="$(cat "$sigint_root/exit_code.txt")"
[ -n "$exit_code" ] || fail "SIGINT process-group test: exit_code.txt missing after PTY driver reported success"
[ "$exit_code" -eq 130 ] || fail "SIGINT process-group test: exit code ($exit_code != 130 -- must not be 141/SIGPIPE)"

terminal_log="$sigint_root/logs/terminal.txt"
grep -q "Received SIGINT" "$terminal_log" 2>/dev/null \
  || fail "SIGINT process-group test: shutdown message missing from terminal log"
grep -q "Stopping rosbag recorder" "$terminal_log" 2>/dev/null \
  || fail "SIGINT process-group test: recorder-shutdown message missing from terminal log"
[ -f "$sigint_root/bags/localisation/metadata.yaml" ] \
  || fail "SIGINT process-group test: recorder bag not finalized"

recorder_pid="$(cat "$sigint_root/recorder.pid" 2>/dev/null || true)"
[ -n "$recorder_pid" ] || fail "SIGINT process-group test: recorder PID never captured"
if kill -0 "$recorder_pid" 2>/dev/null; then
  kill -9 "$recorder_pid" 2>/dev/null || true
  fail "SIGINT process-group test: recorder pid $recorder_pid still alive after shutdown"
fi

pass "a real process-group SIGINT (real terminal Ctrl+C), delivered while blocked in a foreground wait through the actual tee logging topology, logs the shutdown message, finalizes the recorder, and exits 130 (not 141)"
rm -rf -- "$sigint_root"

# "cleanup invoked twice" -- FINALIZE_RAN must make a second invocation a
# safe, side-effect-free no-op (defense in depth: the current trap wiring
# only reaches finalize_test_run via one path at a time, but the contract
# itself must hold regardless).
idempotency_root="$(mktemp -d --tmpdir launch_test_idempotent.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 10 bash -c '
  set -uo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags"
  RECORDER_SHUTDOWN_TIMEOUT_S=3
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  start_recorder
  # First invocation performs real cleanup, in a subshell so its `exit`
  # only ends the subshell, letting this test continue.
  ( finalize_test_run ) >"$3/first.log" 2>&1
  [ -f "$BAG_DESTINATION/metadata.yaml" ]
  # A second invocation, with FINALIZE_RAN pre-set the way a real second
  # trap firing would leave it, must be a silent no-op: no second
  # "Stopping rosbag recorder" message, clean exit status.
  FINALIZE_RAN=1
  ( finalize_test_run ) >"$3/second.log" 2>&1
  second_exit=$?
  [ "$second_exit" -eq 0 ]
  ! grep -q "Stopping rosbag recorder" "$3/second.log"
' _ "$LAUNCH_SH" "$ROOT" "$idempotency_root" >/dev/null || fail "finalize_test_run idempotency"
pass "finalize_test_run runs recorder cleanup exactly once even if invoked twice"
rm -rf -- "$idempotency_root"

failure_root="$(mktemp -d --tmpdir launch_test_recording_fail.XXXXXX)"
PATH="$stub_dir:$PATH" timeout -k 5 10 bash -c '
  set -euo pipefail
  source "$1"
  ROOT="$2"
  RUN_LOGS_DIR="$3/logs"
  LUNAR_SIMULATOR_ROSBAG_DIR="$3/bags-STUB_RECORDER_FAIL"
  mkdir -p "$RUN_LOGS_DIR" "$LUNAR_SIMULATOR_ROSBAG_DIR"
  QOS_OVERRIDES_FILE="$ROOT/config/alpha_rosbag_qos.yaml"
  start_recorder
' _ "$LAUNCH_SH" "$ROOT" "$failure_root" >/dev/null 2>&1 \
  && fail "recorder immediate-exit accepted" || pass "recorder immediate-exit is non-zero"
rm -rf -- "$failure_root"

echo "[launch_test] PASS"
