#!/bin/bash
#
# @File:         launch_simulator.sh
#
# @Brief:        Launch Gazebo, spawn the rover model, and start the ROS bridge.
#
# @Date:         20/09/2026
#

set -euo pipefail

# ---------------------------------------------------------------------------- #
# HELPER FUNCTIONS
# ---------------------------------------------------------------------------- #

msg () {
  echo "[MSG] ${1}"
}

wrn () {
  echo "[WRN] ${1}"
}

err () {
  echo "[ERR] ${1}" >&2
}

#!
# @brief          Prints a path relative to the repository root when it lies
#                 inside it, so terminal lines stay short; other paths are
#                 printed unchanged.
#
# @param  $1      Absolute path.
rel_path() {
  local path="$1"
  if [ -n "$ROOT" ] && [[ "$path" == "$ROOT"/* ]]; then
    printf '%s' "${path#"$ROOT"/}"
  else
    printf '%s' "$path"
  fi
}

#!
# @brief          Resolves and exports the run's rosbag directory under
#                 the canonical SRS_ROSBAG_DIR name (WP-05 Step 6 rename
#                 of LUNAR_SIMULATOR_ROSBAG_DIR). One-release shim: an
#                 SRS_ROSBAG_DIR export wins; a legacy
#                 LUNAR_SIMULATOR_ROSBAG_DIR export is honoured once with
#                 a warning; otherwise the default under $RUN_ROS_DIR is
#                 used. Both names are exported so children started during
#                 the transition see the same path either way.
resolve_rosbag_dir() {
  if [ -z "${SRS_ROSBAG_DIR:-}" ]; then
    if [ -n "${LUNAR_SIMULATOR_ROSBAG_DIR:-}" ]; then
      wrn "LUNAR_SIMULATOR_ROSBAG_DIR is deprecated; use SRS_ROSBAG_DIR"
      SRS_ROSBAG_DIR="$LUNAR_SIMULATOR_ROSBAG_DIR"
    else
      SRS_ROSBAG_DIR="$RUN_ROS_DIR/bags"
    fi
  fi
  export SRS_ROSBAG_DIR
  export LUNAR_SIMULATOR_ROSBAG_DIR="$SRS_ROSBAG_DIR"
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [environment] [system] [--headless] [--rviz] [--record-images] [-h|--help]

  environment       World SDF to simulate (default: crater_field).
                    3-step resolve: absolute path (must exist), else
                    CWD-relative path (if exists), else environment/-
                    relative: a bare name picks the variant dir's SDF
                    (e.g. crater_field ->
                    environment/lunar/crater_field/crater_field.sdf),
                    or a planet/variant[/file] path under environment/.
  system            Rover system under src/systems/ (default: alpha).
                    3-step resolve: absolute dir (must exist), else
                    CWD-relative dir (if exists), else a bare name under
                    src/systems/.
  --headless        Run Gazebo server without GUI
  --rviz            Open RViz with the system's .rviz config
  --record-images   Also record the LocCam stereo and annotated feature
                     image topics into this run's rosbag. Core telemetry
                     (odometry, IMU, joint states, diagnostics topics) is
                     always recorded regardless of this flag; images are
                     opt-in because at 1024x1024/10 Hz they can add multiple
                     gigabytes to a normal system test.
  -h, --help        Show this help

ROS_DOMAIN_ID defaults to 73 so unrelated ROS sessions cannot publish a
second /clock into this simulator. GZ_PARTITION defaults to
space_robotics_simulator_<ROS_DOMAIN_ID> so unrelated Gazebo sessions cannot feed the
bridge. Export either value before launching to override it.

Every run's core+optional telemetry lands in a rosbag under
test_runs/<run>/ros/bags/localisation, alongside a manifest.json recording
the environment/system/domain/partition/profile/topics/source revision this run used. See
post_processing/post_processing.py --test-run test_runs/<run> to turn a
captured run into an interactive HTML report.
EOF
}

# ---------------------------------------------------------------------------- #
# VARIABLES
# ---------------------------------------------------------------------------- #

# Run Gazebo in headless mode (no GUI window). Set via --headless flag or
# RUN_SIMULATION_HEADLESS=1 environment variable.
RUN_SIMULATION_HEADLESS="${RUN_SIMULATION_HEADLESS:-0}"

# Open RViz with the system's RViz config file (src/systems/<system>/<system>.rviz).
# Set via --rviz flag.
OPEN_RVIZ=0

# Also record the LocCam stereo and annotated feature image topics into this
# run's rosbag, on top of the always-on core telemetry set. Set via
# --record-images flag.
RECORD_IMAGES=0

# Keep this simulator's global /clock topic isolated from unrelated ROS work.
# Respect an explicit caller override for multi-system or CI environments.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_DOMAIN_ID

# Gazebo Transport discovery is independent of DDS discovery. Isolate it as
# well so this bridge receives only this simulation's Gazebo /clock stream.
GZ_PARTITION="${GZ_PARTITION:-space_robotics_simulator_${ROS_DOMAIN_ID}}"
export GZ_PARTITION

# Environment selector (bare variant name, environment/-relative path,
# CWD-relative path, or absolute path). Default: crater_field.
ENVIRONMENT="crater_field"

# Rover system name. Must match a directory under src/systems/.
# Default: alpha.
SYSTEM="alpha"

# Absolute paths (populated by resolve_paths)
SCRIPT_DIR=""
ROOT=""

WORLD_FILE=""          # Resolved world SDF file
WORLD_NAME=""           # Gazebo world name read from WORLD_FILE's <world name>
MODEL_DIR=""           # Directory containing model.sdf and model.config
MODEL_FILE=""          # Path to model.sdf
MODEL_CONFIG=""        # Path to model.config
LAUNCH_FILE=""         # Path to launch/<system>_launch.py
BRIDGE_CONFIG=""       # Path to config/<system>/ros_gz_bridge.yaml
RVIZ_SOURCE=""         # Path to src/systems/<system>/<system>.rviz
DRIVERS_YAML=""       # Path to parameters/systems/<system>/<system>_drivers/<system>_drivers.yaml
QOS_OVERRIDES_FILE=""  # Path to config/alpha/rosbag_qos.yaml

# Runtime values
MODEL_NAME=""          # Model name extracted from model.config
GENERATED_MODEL_FILE="" # Temporary SDF with camera noise patched
GAZEBO_PID=""          # Gazebo process ID
RECORDER_PID=""        # Background `ros2 bag record` process ID, once started
RVIZ_PID=""            # RViz process ID when --rviz started it
READY_WATCHER_PID=""   # Background wait_for_system_ready.py process ID
# Wall-clock seconds the launcher waits for the start-up supervisor's READY
# before warning (commands stay blocked by the driver's gate regardless).
READY_TIMEOUT_S="${READY_TIMEOUT_S:-120}"
BAG_DESTINATION=""     # Path passed to `ros2 bag record -o`
RECORD_TOPICS=()       # Assembled by assemble_record_topics() from the profile
ORIGINAL_ARGS=()       # This invocation's argv, captured before parsing/shifting
TEST_RUN_DIR=""        # Timestamped root for this simulation's artifacts
RUN_ROS_DIR=""         # Colcon, ROS, and rosbag artifacts
RUN_LOGS_DIR=""        # Terminal and simulator logs
SIMULATION_STARTED=0    # Whether this invocation reached Gazebo startup
INTERACTIVE_RUN=0       # Whether stdin/stdout began attached to a terminal

# Set by handle_termination_signal() so finalize_test_run() reports the
# conventional 128+signum exit code for a signal-caused shutdown, rather
# than whatever $? happened to be for the foreground command that was
# running when the signal arrived (2026-09-23 fix: see finalize_test_run).
SIGNAL_EXIT_CODE=""
# Guards finalize_test_run() so recorder shutdown and cleanup run exactly
# once, even though it can now be reached both via a caught signal (which
# itself calls `exit`, re-entering the EXIT trap) and via the EXIT trap
# firing on its own for a normal or `set -e` error exit.
FINALIZE_RAN=0
# Longest time, in seconds, stop_recorder() waits for one SIGTERM attempt
# to finalize the recorder before retrying or giving up. Overridable via
# environment so the test suite can use a short bound; production keeps
# the default. Fifteen seconds is generous relative to the few seconds a
# real ~60 MB bag took to flush and finalize when SIGTERM was sent
# directly to two orphaned recorders while diagnosing this fix.
RECORDER_SHUTDOWN_TIMEOUT_S="${RECORDER_SHUTDOWN_TIMEOUT_S:-15}"

# ---------------------------------------------------------------------------- #
# ARGUMENT PARSING
# ---------------------------------------------------------------------------- #

parse_arguments() {
  # Positional arguments: environment, system
  if [ $# -ge 1 ] && [[ "$1" != -* ]]; then
    ENVIRONMENT="$1"
    shift
  fi
  if [ $# -ge 1 ] && [[ "$1" != -* ]]; then
    SYSTEM="$1"
    shift
  fi

  # Flags
  while [ $# -gt 0 ]; do
    case "$1" in
      --headless)       RUN_SIMULATION_HEADLESS=1; shift ;;
      --rviz)           OPEN_RVIZ=1; shift ;;
      --record-images)  RECORD_IMAGES=1; shift ;;
      --help|-h)  usage; exit 0 ;;
      --)         shift; break ;;
      -*)         err "Unknown option: $1"; usage >&2; exit 1 ;;
      *)          break ;;
    esac
  done

  if [ $# -gt 0 ]; then
    err "Too many positional arguments: $*"
    usage >&2
    exit 1
  fi
}

# ---------------------------------------------------------------------------- #
# VALIDATION
# ---------------------------------------------------------------------------- #

validate_names() {
  # Isolation values only; the environment/system positionals are checked
  # by their own resolvers (regex for bare names, existence for paths).
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || { err "Invalid ROS_DOMAIN_ID '$ROS_DOMAIN_ID'"; exit 1; }
  [ "$ROS_DOMAIN_ID" -le 232 ] || { err "ROS_DOMAIN_ID must be in [0, 232]"; exit 1; }
  [[ "$GZ_PARTITION" =~ ^[A-Za-z0-9_-]+$ ]] || { err "Invalid GZ_PARTITION '$GZ_PARTITION'"; exit 1; }
}

#!
# @brief          Sets WORLD_FILE to the single world SDF inside a variant
#                 directory: <dir>/<variant>.sdf when present, else the
#                 lone *.sdf. Anything else is ambiguous, so it fails.
#
# @param  $1      Variant directory.
pick_variant_sdf() {
  local dir="$1"
  local base
  base="$(basename "$dir")"
  if [ -f "$dir/$base.sdf" ]; then
    WORLD_FILE="$dir/$base.sdf"
    return 0
  fi
  local matches=()
  local candidate
  for candidate in "$dir"/*.sdf; do
    [ -f "$candidate" ] && matches+=("$candidate")
  done
  if [ "${#matches[@]}" -eq 1 ]; then
    WORLD_FILE="${matches[0]}"
    return 0
  fi
  err "No single world SDF in $(rel_path "$dir")"
  exit 1
}

#!
# @brief          Sets WORLD_FILE from an already-located path: a file is
#                 used directly, a directory resolves to its variant SDF.
#
# @param  $1      Existing file or directory path.
set_world_file_from_path() {
  local path="$1"
  if [ -f "$path" ]; then
    WORLD_FILE="$path"
  elif [ -d "$path" ]; then
    pick_variant_sdf "$path"
  else
    err "Environment not found: $path"
    exit 1
  fi
}

#!
# @brief          3-step resolve for the environment positional into
#                 WORLD_FILE: (a) absolute path (must exist, else fail);
#                 (b) CWD-relative path (when it exists); (c)
#                 environment/-relative: a planet/variant[/file] path, or a
#                 bare name (^[a-z0-9_]+$) matching a variant directory
#                 whose SDF is picked via pick_variant_sdf(). A trailing
#                 .sdf on an otherwise-bare name is stripped so the old
#                 `launch_simulator.sh lunar_surface.sdf` form keeps working.
resolve_environment() {
  local input="$1"
  if [[ "$input" = /* ]]; then
    [ -e "$input" ] || { err "Environment not found: $input"; exit 1; }
    set_world_file_from_path "$input"
    return 0
  fi
  if [ -e "$input" ]; then
    set_world_file_from_path "$(realpath -m "$input")"
    return 0
  fi
  if [[ "$input" == */* ]]; then
    [ -e "$ROOT/environment/$input" ] || { err "Environment not found: $input"; exit 1; }
    set_world_file_from_path "$ROOT/environment/$input"
    return 0
  fi
  local query="${input%.sdf}"
  [[ "$query" =~ ^[a-z0-9_]+$ ]] || { err "Invalid environment '$input'"; exit 1; }
  local variant
  for variant in "$ROOT"/environment/*/"$query"; do
    if [ -d "$variant" ]; then
      pick_variant_sdf "$variant"
      return 0
    fi
  done
  local sdf
  for sdf in "$ROOT"/environment/*/"$query.sdf"; do
    if [ -f "$sdf" ]; then
      WORLD_FILE="$sdf"
      return 0
    fi
  done
  err "Unknown environment '$input'"
  exit 1
}

#!
# @brief          3-step resolve for the system positional into SYSTEM (the
#                 directory basename every derived path is built from):
#                 (a) absolute directory (must exist, else fail); (b)
#                 CWD-relative directory (when it exists); (c) a bare name
#                 (^[a-z0-9_]+$) or src/systems/-relative path resolving
#                 under src/systems/.
resolve_system() {
  local input="$1"
  if [[ "$input" = /* ]]; then
    [ -d "$input" ] || { err "System not found: $input"; exit 1; }
    SYSTEM="$(basename "$input")"
    return 0
  fi
  if [ -d "$input" ]; then
    SYSTEM="$(basename "$input")"
    return 0
  fi
  if [[ "$input" == */* ]]; then
    [ -d "$ROOT/src/systems/$input" ] || { err "System not found: $input"; exit 1; }
    SYSTEM="$(basename "$input")"
    return 0
  fi
  [[ "$input" =~ ^[a-z0-9_]+$ ]] || { err "Invalid system '$input'"; exit 1; }
  [ -d "$ROOT/src/systems/$input" ] || { err "Unknown system '$input'"; exit 1; }
  SYSTEM="$input"
}

#!
# @brief          Reads the Gazebo world name from the resolved SDF's
#                 <world name="...">. Service calls use this, never the
#                 file name, so renaming a file without its <world name>
#                 cannot silently address the wrong /world/* services.
derive_world_name() {
  WORLD_NAME="$(python3 -c "
import xml.etree.ElementTree as ET, sys
tree = ET.parse(sys.argv[1])
world = tree.getroot().find('world')
print(((world.get('name') if world is not None else '') or '').strip())
" "$WORLD_FILE")"
  [ -n "$WORLD_NAME" ] || { err "No <world name> in $(rel_path "$WORLD_FILE")"; exit 1; }
}

resolve_paths() {
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  ROOT="$(dirname "$SCRIPT_DIR")"

  resolve_environment "$ENVIRONMENT"
  resolve_system "$SYSTEM"
  derive_world_name
  MODEL_DIR="$ROOT/src/systems/${SYSTEM}/${SYSTEM}_model"
  MODEL_FILE="$MODEL_DIR/model.sdf"
  MODEL_CONFIG="$MODEL_DIR/model.config"
  LAUNCH_FILE="$ROOT/launch/${SYSTEM}_launch.py"
  BRIDGE_CONFIG="$ROOT/config/${SYSTEM}/ros_gz_bridge.yaml"
  RVIZ_SOURCE="$ROOT/src/systems/${SYSTEM}/${SYSTEM}.rviz"
  DRIVERS_YAML="$ROOT/parameters/systems/${SYSTEM}/${SYSTEM}_drivers/${SYSTEM}_drivers.yaml"
  # Per-system rosbag QoS overrides when the system ships its own file;
  # otherwise the Alpha default. start_recorder() rewrites that default's
  # /alpha/... keys for a non-alpha SYSTEM, the same way a new system
  # needs its own bridge config and launch file.
  if [ -f "$ROOT/config/${SYSTEM}/rosbag_qos.yaml" ]; then
    QOS_OVERRIDES_FILE="$ROOT/config/${SYSTEM}/rosbag_qos.yaml"
  else
    QOS_OVERRIDES_FILE="$ROOT/config/alpha/rosbag_qos.yaml"
  fi
}

validate_source_files() {
  local required_files=(
    "$WORLD_FILE"
    "$MODEL_FILE"
    "$MODEL_CONFIG"
    "$LAUNCH_FILE"
    "$BRIDGE_CONFIG"
    "$DRIVERS_YAML"
    "$QOS_OVERRIDES_FILE"
  )

  for f in "${required_files[@]}"; do
    [ -f "$f" ] || { err "Missing file: $(rel_path "$f")"; exit 1; }
  done

  # RViz config is optional; if missing, disable RViz
  [ -f "$RVIZ_SOURCE" ] || OPEN_RVIZ=0
}

# ---------------------------------------------------------------------------- #
# RUN ARTIFACTS
# ---------------------------------------------------------------------------- #

create_test_run() {
  local test_runs_dir="$ROOT/test_runs"
  local timestamp

  mkdir -p "$test_runs_dir"
  while true; do
    timestamp="$(date +%Y-%m-%d-%H-%M-%S)"
    TEST_RUN_DIR="$test_runs_dir/$timestamp"
    if mkdir "$TEST_RUN_DIR" 2>/dev/null; then
      break
    fi
    if [ ! -d "$TEST_RUN_DIR" ]; then
      err "Cannot create $(rel_path "$TEST_RUN_DIR")"
      return 1
    fi
    sleep 1
  done

  RUN_ROS_DIR="$TEST_RUN_DIR/ros"
  RUN_LOGS_DIR="$TEST_RUN_DIR/logs"
  mkdir -p "$TEST_RUN_DIR/parameters" \
    "$RUN_ROS_DIR/build_logs" \
    "$RUN_ROS_DIR/logs" \
    "$RUN_ROS_DIR/bags" \
    "$RUN_LOGS_DIR" \
    "$TEST_RUN_DIR/post_processing"
  cp -a "$ROOT/parameters/." "$TEST_RUN_DIR/parameters/"

  export TEST_RUN_DIR
  export COLCON_LOG_PATH="$RUN_ROS_DIR/build_logs"
  export ROS_LOG_DIR="$RUN_ROS_DIR/logs"
  resolve_rosbag_dir
}

#!
# @brief          Redirects this script's own stdout/stderr through `tee`
#                 so the terminal session is captured to
#                 $RUN_LOGS_DIR/terminal.txt.
#
#                 2026-09-23 fix: a real manual `Ctrl+C` acceptance test
#                 found this produced exit status 141 (128+SIGPIPE), no
#                 "Received SIGINT" message, no recorder-shutdown message,
#                 and an orphaned recorder -- a *different, more fundamental*
#                 failure than the earlier SIGINT-to-recorder bug this same
#                 date's other fix addressed. Root cause, confirmed with a
#                 real process-group SIGINT (not a direct function call or
#                 SIGTERM substitute; see the plan for the full trail): the
#                 `tee` below is started via process substitution (`>(...)`),
#                 not `cmd &`, so it does NOT get bash's "asynchronous
#                 commands are immune to SIGINT/SIGQUIT" treatment the way
#                 the backgrounded recorder does -- confirmed directly via
#                 /proc/<tee-pid>/status showing SIGINT at its default,
#                 non-ignored disposition. A terminal Ctrl+C sends SIGINT to
#                 this entire foreground process group at once (no job
#                 control is enabled here, so every child, including this
#                 tee, shares one process group), which kills `tee` almost
#                 immediately. Every later write this script makes to its
#                 own (now pipe-broken) stdout/stderr -- including the
#                 caught-signal trap's own "Received SIG..." line -- then
#                 raises SIGPIPE, which is fatal to bash itself and aborts
#                 execution before any real cleanup (recorder SIGTERM,
#                 metadata.yaml check) can run. Fixed two ways: (1) this
#                 subshell ignores INT/TERM before `exec`-ing into `tee`;
#                 `trap '' SIG` sets SIG_IGN, which (unlike a caught-with-
#                 handler trap) survives `exec`, so the running `tee`
#                 process stays permanently immune to both signals and now
#                 exits only on EOF (i.e. once this script closes its own
#                 fd, normally at process exit) -- the terminal log stays
#                 complete through the whole shutdown sequence. (2) `main()`
#                 separately sets `trap '' PIPE` as defense in depth, so a
#                 broken-pipe write anywhere in this script's own direct
#                 output can no longer kill it outright even if `tee` were
#                 to die for some other reason. Confirmed this does not mask
#                 genuine pipe failures in this script's own explicit
#                 pipelines (e.g. pause_simulation's `gz service | grep -q`):
#                 bash resets SIGPIPE to its default, fatal disposition for
#                 every non-final pipeline component regardless of a
#                 shell-level trap, so `trap '' PIPE` only protects this
#                 script's own direct builtin writes.
start_terminal_capture() {
  if [ -t 0 ] && [ -t 1 ]; then
    INTERACTIVE_RUN=1
  fi
  exec {TERMINAL_LOG_FD}>>"$RUN_LOGS_DIR/terminal.txt"
  exec > >(trap '' INT TERM; exec tee -a "/dev/fd/$TERMINAL_LOG_FD") 2>&1
  msg "Run: $(rel_path "$TEST_RUN_DIR")"
}

rename_test_run() {
  local run_name="$1"
  local renamed_test_run_dir

  [ -n "$run_name" ] || return 0
  if [[ ! "$run_name" =~ ^[A-Za-z0-9._-]+$ ]] ||
    [ "${#run_name}" -gt 100 ]; then
    err "Name: 1-100 of A-Z a-z 0-9 . _ -"
    return 1
  fi

  renamed_test_run_dir="${TEST_RUN_DIR}-${run_name}"
  if [ -e "$renamed_test_run_dir" ]; then
    err "Already exists: $(rel_path "$renamed_test_run_dir")"
    return 1
  fi
  if ! mv -- "$TEST_RUN_DIR" "$renamed_test_run_dir"; then
    err "Cannot rename to $(rel_path "$renamed_test_run_dir")"
    return 1
  fi

  local previous_test_run_dir="$TEST_RUN_DIR"
  TEST_RUN_DIR="$renamed_test_run_dir"
  RUN_ROS_DIR="$TEST_RUN_DIR/ros"
  RUN_LOGS_DIR="$TEST_RUN_DIR/logs"
  export TEST_RUN_DIR
  export COLCON_LOG_PATH="$RUN_ROS_DIR/build_logs"
  export ROS_LOG_DIR="$RUN_ROS_DIR/logs"
  SRS_ROSBAG_DIR="$RUN_ROS_DIR/bags"
  resolve_rosbag_dir
  # The recorder (already stopped by finalize_test_run) wrote manifest.json
  # naming the bag under the timestamp-only directory; carry that path along
  # so the manifest never points at a directory that no longer exists.
  if [[ -n "$BAG_DESTINATION" && "$BAG_DESTINATION" == "$previous_test_run_dir"/* ]]; then
    BAG_DESTINATION="$TEST_RUN_DIR${BAG_DESTINATION#"$previous_test_run_dir"}"
  fi
  relocate_recording_manifest "$previous_test_run_dir" "$TEST_RUN_DIR" ||
    wrn "manifest.json bag path not updated"
  msg "Named: $(rel_path "$TEST_RUN_DIR")"
}

#!
# @brief          Rewrites manifest.json's bag_destination after the run
#                 directory has been renamed, preserving every other field.
#                 A missing manifest (no recording) is not an error.
#
# @param  $1      The run directory before the rename.
# @param  $2      The run directory after the rename.
relocate_recording_manifest() {
  local manifest_path="$SRS_ROSBAG_DIR/manifest.json"
  [ -f "$manifest_path" ] || return 0
  python3 - "$manifest_path" "$1" "$2" <<'PYEOF'
import json
import sys

manifest_path, previous_dir, renamed_dir = sys.argv[1:4]
with open(manifest_path, encoding="utf-8") as manifest_file:
    manifest = json.load(manifest_file)
destination = manifest.get("bag_destination")
if isinstance(destination, str) and destination.startswith(previous_dir + "/"):
    manifest["bag_destination"] = renamed_dir + destination[len(previous_dir):]
    with open(manifest_path, "w", encoding="utf-8") as manifest_file:
        json.dump(manifest, manifest_file, indent=2, sort_keys=True)
        manifest_file.write("\n")
PYEOF
}

prompt_for_test_run_name() {
  local run_name

  [ "$INTERACTIVE_RUN" -eq 1 ] || return 0
  while true; do
    printf "Run name (Enter to skip): "
    if ! IFS= read -r run_name; then
      return 0
    fi
    [ -n "$run_name" ] || return 0
    if rename_test_run "$run_name"; then
      return 0
    fi
  done
}

#!
# @brief          Installed for SIGINT and SIGTERM (2026-09-23 fix: a real
#                 manual `Ctrl+C` acceptance test found the launcher had no
#                 trap for either -- only EXIT -- so a terminal interrupt
#                 returned directly to the shell without ever running
#                 finalize_test_run(), leaving the recorder orphaned and
#                 metadata.yaml unfinalized). Records the conventional
#                 128+signum exit code for this signal, then calls `exit`,
#                 which itself invokes finalize_test_run() via the EXIT
#                 trap -- there is exactly one place cleanup actually
#                 happens, regardless of which of these three traps fired
#                 it. Clears its own traps first so a determined second
#                 Ctrl+C/kill can still force an immediate exit rather than
#                 being silently absorbed if cleanup is itself stuck.
#
# @param  $1      The signal name as bash's trap syntax names it (e.g.
#                 "INT" or "TERM"), passed explicitly by the `trap '...'
#                 SIG` command line rather than inferred, since a trap
#                 action has no other reliable way to know which signal
#                 invoked it.
handle_termination_signal() {
  local signal_name="$1"
  trap - INT TERM
  err "Received SIG${signal_name}; shutting down"
  SIGNAL_EXIT_CODE=$((128 + $(kill -l "$signal_name")))
  exit "$SIGNAL_EXIT_CODE"
}

#!
# @brief          The single place cleanup actually happens, reached via
#                 the EXIT trap for every termination path: normal
#                 completion, a `set -e` error, or a caught SIGINT/SIGTERM
#                 (which reach here by calling `exit` from
#                 handle_termination_signal(), re-entering this same EXIT
#                 trap). Idempotent: FINALIZE_RAN guards against running
#                 the body twice even though only one of these paths can
#                 actually reach it in practice.
finalize_test_run() {
  local exit_status=$?

  [ "$FINALIZE_RAN" -eq 1 ] && return 0
  FINALIZE_RAN=1
  trap - EXIT INT TERM
  if [ -n "$SIGNAL_EXIT_CODE" ]; then
    exit_status="$SIGNAL_EXIT_CODE"
  fi
  # Stop the recorder (if any) before the rename prompt so metadata.yaml is
  # finalized before the run directory -- and the bag inside it -- can
  # move. A recorder that could not be gracefully finalized overrides an
  # otherwise-successful exit status: a run advertised as automatically
  # recorded must not report success while leaving an unfinalized bag.
  if ! stop_recorder; then
    exit_status=1
  fi
  if [ -n "$GENERATED_MODEL_FILE" ]; then
    rm -f -- "$GENERATED_MODEL_FILE"
  fi
  # RViz and the READY watcher belong to this launcher; close them on
  # every exit path.
  local owned_pid
  for owned_pid in "$RVIZ_PID" "$READY_WATCHER_PID"; do
    if [ -n "$owned_pid" ] && kill -0 "$owned_pid" 2>/dev/null; then
      kill -TERM "$owned_pid" 2>/dev/null || true
    fi
  done
  if [ "$SIMULATION_STARTED" -eq 1 ]; then
    prompt_for_test_run_name || true
  fi
  exit "$exit_status"
}

# ---------------------------------------------------------------------------- #
# BUILD
# ---------------------------------------------------------------------------- #

build_workspace() {
  msg "Building workspace (logs/build.txt)"
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  # Compiler output (including the documented -Wconversion baseline) stays
  # in the run's log; only a failure is surfaced on the terminal.
  if ! colcon build --symlink-install --base-paths "$ROOT" \
      >"$RUN_LOGS_DIR/build.txt" 2>&1; then
    err "Build failed; see logs/build.txt"
    return 1
  fi
  set +u
  source "$ROOT/install/setup.bash"
  set -u
}

# ---------------------------------------------------------------------------- #
# MODEL PREPARATION
# ---------------------------------------------------------------------------- #

extract_model_name() {
  msg "Reading model name"
  MODEL_NAME="$(python3 -c "
import xml.etree.ElementTree as ET, sys
tree = ET.parse(sys.argv[1])
print((tree.getroot().findtext('name') or '').strip())
" "$MODEL_CONFIG")"

  [[ -n "$MODEL_NAME" && "$MODEL_NAME" =~ ^[A-Za-z0-9_-]+$ ]] || {
    err "Bad model name in $(rel_path "$MODEL_CONFIG")"
    exit 1
  }
}

patch_camera_noise() {
  # Gazebo applies camera noise natively via the SDF. The drivers YAML exposes
  # camera_pixel_stddev in 8-bit intensity units (0-255). We convert to Gazebo's
  # normalized intensity (0.0-1.0) and patch the SDF before spawn so noise is
  # applied at the sensor level without relaying images through ROS.
  local noise_enabled
  local camera_pixel_stddev
  local normalized_stddev

  noise_enabled="$(awk '/^noise_enabled:/ {print $2; exit}' "$DRIVERS_YAML")"
  camera_pixel_stddev="$(awk '/^camera_pixel_stddev:/ {print $2; exit}' "$DRIVERS_YAML")"

  [ "$noise_enabled" = "false" ] && camera_pixel_stddev="0.0"

  normalized_stddev="$(awk -v v="$camera_pixel_stddev" 'BEGIN {printf "%.10f", v/255.0}')"

  GENERATED_MODEL_FILE="$(mktemp --tmpdir -- "${MODEL_NAME}.XXXXXX.sdf")"
  sed "s|<stddev>0.0235294118</stddev><!-- ALPHA_CAMERA_NOISE_STDDEV -->|<stddev>$normalized_stddev</stddev><!-- ALPHA_CAMERA_NOISE_STDDEV -->|g" \
    "$MODEL_FILE" > "$GENERATED_MODEL_FILE"

}

# ---------------------------------------------------------------------------- #
# GAZEBO OPERATIONS
# ---------------------------------------------------------------------------- #

start_gazebo() {
  msg "Starting Gazebo (logs/gazebo.txt)"
  GZ_SIM_RESOURCE_PATH="$(dirname "$WORLD_FILE"):$ROOT/environment/lunar/crater_field:$ROOT/environment/martian/plain_stub:$MODEL_DIR"
  export GZ_SIM_RESOURCE_PATH

  # Full -v4 output goes to the run's log, not the operator's terminal;
  # readiness is detected through `gz service -l`, never by reading it.
  if [ "$RUN_SIMULATION_HEADLESS" = "1" ]; then
    gz sim -s -v4 "$WORLD_FILE" >>"$RUN_LOGS_DIR/gazebo.txt" 2>&1 &
  else
    gz sim -v4 "$WORLD_FILE" >>"$RUN_LOGS_DIR/gazebo.txt" 2>&1 &
  fi
  GAZEBO_PID=$!
  SIMULATION_STARTED=1
}

wait_for_world() {
  msg "Waiting for world '$WORLD_NAME'"
  local ready=0
  for _ in {1..30}; do
    if gz service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/create"; then
      ready=1
      break
    fi
    sleep 1
  done
  [ "$ready" = "1" ] || { err "World not ready; see logs/gazebo.txt"; exit 1; }
}

pause_simulation() {
  msg "Pausing simulation"
  gz service -s "/world/$WORLD_NAME/control" \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
    -r "pause: true" | grep -q "data: true"
}

spawn_model() {
  msg "Spawning '$MODEL_NAME'"
  # The reply is checked rather than printed, like pause_simulation's.
  if ! gz service -s "/world/$WORLD_NAME/create" \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
      -r "sdf_filename: \"$GENERATED_MODEL_FILE\", name: \"$MODEL_NAME\", allow_renaming: false, pose: {position: {x: 0.0, y: 0.0, z: 0.02}}" \
      | grep -q "data: true"; then
    err "Spawn request was rejected"
    exit 1
  fi
}

wait_for_spawn() {
  msg "Waiting for spawn"
  local spawned=0
  for _ in {1..15}; do
    if gz model --list 2>/dev/null | sed -n 's/^[[:space:]]*-[[:space:]]*//p' | grep -Fxq "$MODEL_NAME"; then
      spawned=1
      break
    fi
    sleep 1
  done
  [ "$spawned" = "1" ] || { err "Model missing after spawn"; exit 1; }
}

unpause_simulation() {
  msg "Starting simulation"
  gz service -s "/world/$WORLD_NAME/control" \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
    -r "pause: false" >/dev/null
}

# ---------------------------------------------------------------------------- #
# ROSBAG RECORDING
# ---------------------------------------------------------------------------- #

# Core telemetry, always recorded: ground truth, every subsystem's raw
# sensor input and odometry output, commands, the slip topics, and the
# shared diagnostics topic that carries every node's periodic machine
# records in simulation time (moved off the console). Excludes
# retained `Path` topics (nav_msgs/Path duplicates odometry history and
# makes later bag messages progressively larger for no post-processing
# benefit -- report trajectories are drawn from the odometry topics
# instead).
# Topic suffixes recorded under /${SYSTEM}/ (`/clock` is global and keeps
# no prefix). assemble_record_topics() prefixes them with the live SYSTEM
# value, so /alpha stays the default-system example without hardcoding it
# in every entry.
CORE_RECORD_SUFFIXES=(
  diagnostics
  system/state
  drivers/ground_truth/odometry
  localisation/ground_truth/odometry
  drivers/imu
  imu
  localisation/inertial/filtered_imu
  localisation/inertial/odometry
  drivers/joint_states
  joint_states
  control/cmd/velocity
  control/cmd/wheel_joint_states
  drivers/cmd/wheel_joint_states
  localisation/wheel/odometry
  localisation/wheel/slip_ratios
  localisation/wheel/slip_observation
  localisation/visual/odometry
  localisation/visual/point_cloud
  localisation/visual/reset
  localisation/kalman_filter/odometry
  localisation/kalman_filter/wheel_slip_ratio
  control/filtered_odometry
)

# Added on top of the core set only when --record-images is given.
IMAGE_RECORD_SUFFIXES=(
  drivers/loccam/left
  drivers/loccam/right
  localisation/visual/features
)

#!
# @brief          Populates RECORD_TOPICS from the core set plus, when
#                 --record-images was given, the image set, each under
#                 /${SYSTEM}/ (/clock stays global).
assemble_record_topics() {
  RECORD_TOPICS=(/clock)
  local suffix
  for suffix in "${CORE_RECORD_SUFFIXES[@]}"; do
    RECORD_TOPICS+=("/${SYSTEM}/${suffix}")
  done
  if [ "$RECORD_IMAGES" = "1" ]; then
    for suffix in "${IMAGE_RECORD_SUFFIXES[@]}"; do
      RECORD_TOPICS+=("/${SYSTEM}/${suffix}")
    done
  fi
}

#!
# @brief          Writes ros/bags/manifest.json describing this run's
#                 recording: environment/system/domain/partition, the exact
#                 command line, the recording profile and topic list, the
#                 storage identifier, and source revision/dirty-worktree
#                 when this checkout is a git repository. Deliberately does
#                 not copy the working diff itself -- the parameter
#                 snapshot under TEST_RUN_DIR/parameters/ remains the
#                 tuning source of truth, and a diff is not needed to tell
#                 a later reader which commit a run was captured against.
write_recording_manifest() {
  local recording_profile="core"
  [ "$RECORD_IMAGES" = "1" ] && recording_profile="core+images"

  # Resolve the source revision only when this checkout is a git repository;
  # a source tarball or export has neither, and that is not an error.
  local git_available="false"
  local git_revision="unknown"
  local git_dirty="false"
  if git -C "$ROOT" rev-parse --git-dir >/dev/null 2>&1; then
    git_available="true"
    git_revision="$(git -C "$ROOT" rev-parse HEAD)"
    # A non-empty porcelain status means at least one tracked or untracked
    # change is present relative to HEAD.
    if [ -n "$(git -C "$ROOT" status --porcelain)" ]; then
      git_dirty="true"
    fi
  fi

  # Pass every field through the environment rather than argv: topic names
  # and args are shell-safe individually, but building one JSON-correct
  # argv/heredoc split in bash is more error-prone than letting Python do
  # its own whitespace-split and JSON escaping.
  MANIFEST_WORLD="$ENVIRONMENT" \
  MANIFEST_SYSTEM="$SYSTEM" \
  MANIFEST_ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  MANIFEST_GZ_PARTITION="$GZ_PARTITION" \
  MANIFEST_COMMAND_LINE="$(basename "$0") ${ORIGINAL_ARGS[*]-}" \
  MANIFEST_RECORDING_PROFILE="$recording_profile" \
  MANIFEST_TOPICS="${RECORD_TOPICS[*]}" \
  MANIFEST_STORAGE_IDENTIFIER="mcap" \
  MANIFEST_BAG_DESTINATION="$BAG_DESTINATION" \
  MANIFEST_GIT_AVAILABLE="$git_available" \
  MANIFEST_GIT_REVISION="$git_revision" \
  MANIFEST_GIT_DIRTY="$git_dirty" \
  MANIFEST_OUTPUT_PATH="$SRS_ROSBAG_DIR/manifest.json" \
  python3 <<'PYEOF'
import json
import os

# Read every field from the environment set by the caller above; this
# mirrors extract_model_name()'s use of python3 for structured parsing
# that bash itself handles poorly (JSON string escaping here).
manifest = {
    "world": os.environ["MANIFEST_WORLD"],
    "system": os.environ["MANIFEST_SYSTEM"],
    "ros_domain_id": os.environ["MANIFEST_ROS_DOMAIN_ID"],
    "gz_partition": os.environ["MANIFEST_GZ_PARTITION"],
    "command_line": os.environ["MANIFEST_COMMAND_LINE"],
    "recording_profile": os.environ["MANIFEST_RECORDING_PROFILE"],
    # Topic names never contain whitespace, so a plain split is exact.
    "topics": os.environ["MANIFEST_TOPICS"].split(),
    "storage_identifier": os.environ["MANIFEST_STORAGE_IDENTIFIER"],
    "bag_destination": os.environ["MANIFEST_BAG_DESTINATION"],
    "source_revision": (
        os.environ["MANIFEST_GIT_REVISION"]
        if os.environ["MANIFEST_GIT_AVAILABLE"] == "true"
        else None
    ),
    "dirty_worktree": (
        os.environ["MANIFEST_GIT_DIRTY"] == "true"
        if os.environ["MANIFEST_GIT_AVAILABLE"] == "true"
        else None
    ),
}

output_path = os.environ["MANIFEST_OUTPUT_PATH"]
with open(output_path, "w", encoding="utf-8") as manifest_file:
    json.dump(manifest, manifest_file, indent=2, sort_keys=True)
    manifest_file.write("\n")
PYEOF
}

#!
# @brief          Picks the QoS overrides for this run: the system's own
#                 file when it ships one, the Alpha default for alpha
#                 itself, otherwise a same-run rewrite of the Alpha default
#                 with /alpha/ swapped for /${SYSTEM}/ (a stale /alpha key
#                 would silently match nothing the recorder subscribes to).
resolve_qos_overrides() {
  local candidate="$ROOT/config/${SYSTEM}/rosbag_qos.yaml"
  if [ -f "$candidate" ]; then
    QOS_OVERRIDES_FILE="$candidate"
    return 0
  fi
  if [ "$SYSTEM" = "alpha" ]; then
    QOS_OVERRIDES_FILE="$ROOT/config/alpha/rosbag_qos.yaml"
    return 0
  fi
  QOS_OVERRIDES_FILE="$RUN_LOGS_DIR/rosbag_qos_${SYSTEM}.yaml"
  sed "s|/alpha/|/${SYSTEM}/|g" "$ROOT/config/alpha/rosbag_qos.yaml" \
    > "$QOS_OVERRIDES_FILE"
}

#!
# @brief          Starts `ros2 bag record` in the background against the
#                 assembled topic profile, before the foreground bridge
#                 starts. Recording begins with explicit topics and normal
#                 discovery (not --no-discovery), so the recorder is
#                 already subscribed and ready to catch the earliest
#                 messages once the bridge and alpha_node's nodes come up
#                 after it, rather than needing to start after them.
#
# Exits the launcher if the recorder exits immediately or never creates its
# bag destination: a run advertised as automatically recorded must not
# silently contain an empty ros/bags/ directory.
start_recorder() {
  assemble_record_topics
  resolve_qos_overrides
  BAG_DESTINATION="$SRS_ROSBAG_DIR/localisation"

  local recording_profile="core"
  [ "$RECORD_IMAGES" = "1" ] && recording_profile="core+images"
  msg "Recording $recording_profile: ${#RECORD_TOPICS[@]} topics"

  # -s mcap: Jazzy's default, chosen explicitly per D6 (no second rosbag
  # implementation or direct MCAP/SQLite parsing -- rosbag2_py owns this).
  # zstd_fast keeps recording overhead low during a live headless run;
  # trades some file size for that, acceptable for test-run artifacts.
  # --use-sim-time ties every recorded message's bag-received stamp to the
  # simulator's /clock, matching every other timing-sensitive tool here.
  ros2 bag record \
    -o "$BAG_DESTINATION" \
    -s mcap \
    --storage-preset-profile zstd_fast \
    --qos-profile-overrides-path "$QOS_OVERRIDES_FILE" \
    --use-sim-time \
    --topics "${RECORD_TOPICS[@]}" \
    >"$RUN_LOGS_DIR/rosbag_record.log" 2>&1 &
  RECORDER_PID=$!

  # Give the recorder a moment to open its storage backend before trusting
  # it; ros2_bag_record creates the bag directory/file as soon as the
  # storage backend opens, independent of whether any message has arrived.
  sleep 2
  if ! kill -0 "$RECORDER_PID" 2>/dev/null; then
    err "Recorder exited; see logs/rosbag_record.log"
    exit 1
  fi
  if [ ! -d "$BAG_DESTINATION" ]; then
    err "Recorder made no bag: $(rel_path "$BAG_DESTINATION")"
    exit 1
  fi

  write_recording_manifest
}

#!
# @brief          Polls (never blocks indefinitely) for PID $1 to exit,
#                 up to RECORDER_SHUTDOWN_TIMEOUT_S seconds.
#
# @param  $1      The PID to wait for.
#
# @return         0 if the process exited within the timeout; 1 if it was
#                 still running when the timeout elapsed.
_wait_for_recorder_exit() {
  local pid="$1"
  local waited=0
  while kill -0 "$pid" 2>/dev/null; do
    if [ "$waited" -ge "$RECORDER_SHUTDOWN_TIMEOUT_S" ]; then
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 0
}

#!
# @brief          Stops a running recorder cleanly (never a broad pkill)
#                 and confirms metadata.yaml actually exists before
#                 reporting success. Idempotent and safe to call when no
#                 recorder was started, or it has already exited.
#
#                 Uses SIGTERM, not SIGINT (2026-09-23 fix -- see the
#                 plan's "manual acceptance failure" section for the full
#                 evidence trail): `ros2 bag record` is always started as
#                 a background job (`... &`) from this non-interactive,
#                 job-control-disabled script. Bash sets SIGINT (and
#                 SIGQUIT) to be ignored on any such background child
#                 before exec -- the standard, POSIX-documented reason a
#                 backgrounded job is immune to the same Ctrl+C that
#                 interrupts the shell's own foreground command -- and
#                 rclpy/ros2cli's underlying CPython process never re-arms
#                 an inherited SIG_IGN. This was confirmed directly against
#                 two real orphaned recorders left over from the manual
#                 acceptance failure: `/proc/<pid>/status` showed SIGINT in
#                 SigIgn and SIGTERM in SigCgt, and sending SIGTERM
#                 directly to both produced fully finalized,
#                 `ros2 bag info`-readable bags. SIGINT could never have
#                 worked here regardless of how correctly this script
#                 handled its own Ctrl+C.
#
#                 Retries once if the recorder does not exit within one
#                 timeout window (a transient delay handling the first
#                 signal is not the same as truly refusing to stop). Never
#                 escalates to SIGKILL: a killed recorder cannot finalize
#                 metadata.yaml, so that would only guarantee the very
#                 data loss this function exists to prevent -- a recorder
#                 that survives two SIGTERM attempts is left running and
#                 reported as an explicit failure instead, identifying the
#                 unfinalized bag so a human can investigate.
#
# @return         0 if the recorder is confirmed stopped and its
#                 metadata.yaml exists (or no recorder was ever running);
#                 1 if it could not be stopped within the bounded retry
#                 budget, or it stopped but metadata.yaml is still
#                 missing. RECORDER_PID is left set (not cleared) on
#                 failure, so a caller can still report the offending PID.
stop_recorder() {
  [ -n "$RECORDER_PID" ] || return 0
  local recorder_pid="$RECORDER_PID"

  if kill -0 "$recorder_pid" 2>/dev/null; then
    local attempt
    for attempt in 1 2; do
      msg "Stopping rosbag recorder (try $attempt)"
      kill -TERM "$recorder_pid" 2>/dev/null || true
      _wait_for_recorder_exit "$recorder_pid" && break
    done
  fi

  if kill -0 "$recorder_pid" 2>/dev/null; then
    local message
    message="Recorder pid $recorder_pid ignored 2 SIGTERMs"
    message="$message (${RECORDER_SHUTDOWN_TIMEOUT_S}s each). Bag NOT"
    message="$message finalized: $(rel_path "$BAG_DESTINATION"). Left"
    message="$message running; stop it by hand. SIGKILL would lose"
    message="$message metadata.yaml."
    err "$message"
    return 1
  fi

  # The recorder is gone -- whether it had already exited on its own
  # before this call, or just responded to a signal above. Reap it if it
  # is still a zombie, then confirm the bag it was writing actually
  # finalized; a dead recorder process is not, by itself, proof of that.
  wait "$recorder_pid" 2>/dev/null || true
  RECORDER_PID=""
  if [ -n "$BAG_DESTINATION" ] && [ ! -f "$BAG_DESTINATION/metadata.yaml" ]; then
    err "Recorder exited without metadata.yaml: $(rel_path "$BAG_DESTINATION")"
    return 1
  fi
  return 0
}

# ---------------------------------------------------------------------------- #
# ROS LAUNCH
# ---------------------------------------------------------------------------- #

launch_ros_nodes() {
  msg "Launching '$SYSTEM' nodes (domain $ROS_DOMAIN_ID)"

  ros2 launch space_robotics_simulator "${SYSTEM}_launch.py" \
    system_name:="$SYSTEM" use_sim_time:=true \
    ros_domain_id:="$ROS_DOMAIN_ID" \
    gz_partition:="$GZ_PARTITION" \
    launch_gazebo:=false launch_bridge:=false launch_rviz:=false &

  # RViz is started here rather than by the launch file so its console
  # output lands in the run's logs instead of the operator's terminal.
  if [ "$OPEN_RVIZ" = "1" ]; then
    msg "Starting RViz (logs/rviz.txt)"
    rviz2 -d "$RVIZ_SOURCE" >"$RUN_LOGS_DIR/rviz.txt" 2>&1 &
    RVIZ_PID=$!
  fi
}

#!
# @brief          Waits in the background for the start-up supervisor's
#                 READY on /<system>/system/state and prints it (or a warning
#                 after READY_TIMEOUT_S). Commands are gated by the driver
#                 either way; this only tells the operator when they will
#                 be obeyed. Started before the foreground bridge, which
#                 provides the /clock the supervisor needs.
start_ready_watcher() {
  msg "Waiting for READY (commands blocked)"
  python3 "$SCRIPT_DIR/wait_for_system_ready.py" \
    --system "$SYSTEM" --timeout-s "$READY_TIMEOUT_S" --prefix "[MSG] " &
  READY_WATCHER_PID=$!
}

run_bridge() {
  msg "Starting bridge (logs/bridge.txt)"
  if ! ros2 run ros_gz_bridge parameter_bridge \
      --ros-args -p config_file:="$BRIDGE_CONFIG" \
      >"$RUN_LOGS_DIR/bridge.txt" 2>&1; then
    err "Bridge exited; see logs/bridge.txt"
    return 1
  fi
}

# ---------------------------------------------------------------------------- #
# EXECUTE launch_simulator.sh
# ---------------------------------------------------------------------------- #

main() {
  # Captured before parse_arguments shifts/consumes "$@", for the recording
  # manifest's command_line field.
  ORIGINAL_ARGS=("$@")
  parse_arguments "$@"
  validate_names
  resolve_paths
  validate_source_files
  create_test_run
  start_terminal_capture
  trap finalize_test_run EXIT
  # A single terminal Ctrl+C (SIGINT) or SIGTERM must reach this same
  # cleanup path (2026-09-23 fix): previously only EXIT was trapped, so an
  # interrupt returned directly to the shell without running
  # finalize_test_run() at all -- confirmed by a real manual acceptance
  # test that found no recorder-finalization output, a missing
  # metadata.yaml, and an orphaned recorder process after Ctrl+C.
  trap 'handle_termination_signal INT' INT
  trap 'handle_termination_signal TERM' TERM
  # Defense in depth alongside start_terminal_capture's tee-immunity fix
  # (2026-09-23): ignore SIGPIPE so a write to a broken stdout/stderr pipe
  # fails that one write (non-zero return) instead of killing this whole
  # script outright before cleanup can run. See start_terminal_capture's
  # comment for why this does not hide a genuine pipe failure elsewhere in
  # this script (e.g. pause_simulation's own pipeline).
  trap '' PIPE
  build_workspace
  extract_model_name
  patch_camera_noise
  start_gazebo
  wait_for_world
  pause_simulation
  spawn_model
  wait_for_spawn
  unpause_simulation
  launch_ros_nodes
  start_recorder
  start_ready_watcher
  run_bridge
  wait "$GAZEBO_PID"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
