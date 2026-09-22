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

msg() {
  echo "[MSG] ${1}"
}

wrn() {
  echo "[WRN] ${1}"
}

err() {
  echo "[ERR] ${1}" >&2
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [world] [system] [--headless] [--rviz] [-h|--help]

  world        Gazebo world under worlds/ (default: lunar_surface; trailing .sdf stripped)
  system       Rover system under src/systems/ (default: alpha)
  --headless   Run Gazebo server without GUI
  --rviz       Open RViz with the system's .rviz config
  -h, --help   Show this help

ROS_DOMAIN_ID defaults to 73 so unrelated ROS sessions cannot publish a
second /clock into this simulator. GZ_PARTITION defaults to
lunar_simulator_<ROS_DOMAIN_ID> so unrelated Gazebo sessions cannot feed the
bridge. Export either value before launching to override it.
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

# Keep this simulator's global /clock topic isolated from unrelated ROS work.
# Respect an explicit caller override for multi-system or CI environments.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export ROS_DOMAIN_ID

# Gazebo Transport discovery is independent of DDS discovery. Isolate it as
# well so this bridge receives only this simulation's Gazebo /clock stream.
GZ_PARTITION="${GZ_PARTITION:-lunar_simulator_${ROS_DOMAIN_ID}}"
export GZ_PARTITION

# Gazebo world name (without .sdf extension). Default: lunar_surface.
WORLD="lunar_surface"

# Rover system name. Must match a directory under src/systems/.
# Default: alpha.
SYSTEM="alpha"

# Absolute paths (populated by resolve_paths)
SCRIPT_DIR=""
ROOT=""

WORLD_FILE=""          # Path to world SDF file
MODEL_DIR=""           # Directory containing model.sdf and model.config
MODEL_FILE=""          # Path to model.sdf
MODEL_CONFIG=""        # Path to model.config
LAUNCH_FILE=""         # Path to launch/<system>_launch.py
BRIDGE_CONFIG=""       # Path to config/<system>_ros_gz_bridge.yaml
RVIZ_SOURCE=""         # Path to src/systems/<system>/<system>.rviz
DRIVERS_YAML=""        # Path to parameters/systems/<system>/<system>_drivers/<system>_drivers.yaml

# Runtime values
MODEL_NAME=""          # Model name extracted from model.config
GENERATED_MODEL_FILE="" # Temporary SDF with camera noise patched
GAZEBO_PID=""          # Gazebo process ID
TEST_RUN_DIR=""        # Timestamped root for this simulation's artifacts
RUN_ROS_DIR=""         # Colcon, ROS, and rosbag artifacts
RUN_LOGS_DIR=""        # Terminal and simulator logs
SIMULATION_STARTED=0    # Whether this invocation reached Gazebo startup
INTERACTIVE_RUN=0       # Whether stdin/stdout began attached to a terminal

# ---------------------------------------------------------------------------- #
# ARGUMENT PARSING
# ---------------------------------------------------------------------------- #

parse_arguments() {
  # Positional arguments: world, system
  if [ $# -ge 1 ] && [[ "$1" != -* ]]; then
    WORLD="$1"
    shift
  fi
  if [ $# -ge 1 ] && [[ "$1" != -* ]]; then
    SYSTEM="$1"
    shift
  fi

  # Flags
  while [ $# -gt 0 ]; do
    case "$1" in
      --headless) RUN_SIMULATION_HEADLESS=1; shift ;;
      --rviz)     OPEN_RVIZ=1; shift ;;
      --help|-h)  usage; exit 0 ;;
      --)         shift; break ;;
      -*)         err "Unknown option: $1"; usage >&2; exit 1 ;;
      *)          break ;;
    esac
  done

  # Strip one trailing .sdf if present
  WORLD="${WORLD%.sdf}"
}

# ---------------------------------------------------------------------------- #
# VALIDATION
# ---------------------------------------------------------------------------- #

validate_names() {
  [[ "$WORLD" =~ ^[A-Za-z0-9_-]+$ ]] || { err "Invalid world '$WORLD'"; exit 1; }
  [[ "$SYSTEM" =~ ^[A-Za-z0-9_-]+$ ]] || { err "Invalid system '$SYSTEM'"; exit 1; }
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || { err "Invalid ROS_DOMAIN_ID '$ROS_DOMAIN_ID'"; exit 1; }
  [ "$ROS_DOMAIN_ID" -le 232 ] || { err "ROS_DOMAIN_ID must be in [0, 232]"; exit 1; }
  [[ "$GZ_PARTITION" =~ ^[A-Za-z0-9_-]+$ ]] || { err "Invalid GZ_PARTITION '$GZ_PARTITION'"; exit 1; }
}

resolve_paths() {
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  ROOT="$(dirname "$SCRIPT_DIR")"

  WORLD_FILE="$ROOT/worlds/${WORLD}.sdf"
  MODEL_DIR="$ROOT/src/systems/${SYSTEM}/${SYSTEM}_model"
  MODEL_FILE="$MODEL_DIR/model.sdf"
  MODEL_CONFIG="$MODEL_DIR/model.config"
  LAUNCH_FILE="$ROOT/launch/${SYSTEM}_launch.py"
  BRIDGE_CONFIG="$ROOT/config/${SYSTEM}_ros_gz_bridge.yaml"
  RVIZ_SOURCE="$ROOT/src/systems/${SYSTEM}/${SYSTEM}.rviz"
  DRIVERS_YAML="$ROOT/parameters/systems/${SYSTEM}/${SYSTEM}_drivers/${SYSTEM}_drivers.yaml"
}

validate_source_files() {
  local required_files=(
    "$WORLD_FILE"
    "$MODEL_FILE"
    "$MODEL_CONFIG"
    "$LAUNCH_FILE"
    "$BRIDGE_CONFIG"
    "$DRIVERS_YAML"
  )

  for f in "${required_files[@]}"; do
    [ -f "$f" ] || { err "Missing required file: $f"; exit 1; }
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
      err "Unable to create test run directory: $TEST_RUN_DIR"
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
  export LUNAR_SIMULATOR_ROSBAG_DIR="$RUN_ROS_DIR/bags"
}

start_terminal_capture() {
  if [ -t 0 ] && [ -t 1 ]; then
    INTERACTIVE_RUN=1
  fi
  exec {TERMINAL_LOG_FD}>>"$RUN_LOGS_DIR/terminal.txt"
  exec > >(tee -a "/dev/fd/$TERMINAL_LOG_FD") 2>&1
  msg "Recording test run in $TEST_RUN_DIR"
}

rename_test_run() {
  local run_name="$1"
  local renamed_test_run_dir

  [ -n "$run_name" ] || return 0
  if [[ ! "$run_name" =~ ^[A-Za-z0-9._-]+$ ]] ||
    [ "${#run_name}" -gt 100 ]; then
    err "Test run names must be 1-100 characters using only letters, numbers, '.', '_' or '-'"
    return 1
  fi

  renamed_test_run_dir="${TEST_RUN_DIR}-${run_name}"
  if [ -e "$renamed_test_run_dir" ]; then
    err "Test run directory already exists: $renamed_test_run_dir"
    return 1
  fi
  if ! mv -- "$TEST_RUN_DIR" "$renamed_test_run_dir"; then
    err "Unable to rename test run directory to: $renamed_test_run_dir"
    return 1
  fi

  TEST_RUN_DIR="$renamed_test_run_dir"
  RUN_ROS_DIR="$TEST_RUN_DIR/ros"
  RUN_LOGS_DIR="$TEST_RUN_DIR/logs"
  export TEST_RUN_DIR
  export COLCON_LOG_PATH="$RUN_ROS_DIR/build_logs"
  export ROS_LOG_DIR="$RUN_ROS_DIR/logs"
  export LUNAR_SIMULATOR_ROSBAG_DIR="$RUN_ROS_DIR/bags"
  msg "Named test run: $TEST_RUN_DIR"
}

prompt_for_test_run_name() {
  local run_name

  [ "$INTERACTIVE_RUN" -eq 1 ] || return 0
  while true; do
    printf "Name this test run (Enter to keep only the timestamp): "
    if ! IFS= read -r run_name; then
      return 0
    fi
    [ -n "$run_name" ] || return 0
    if rename_test_run "$run_name"; then
      return 0
    fi
  done
}

finalize_test_run() {
  local exit_status=$?

  trap - EXIT
  if [ -n "$GENERATED_MODEL_FILE" ]; then
    rm -f -- "$GENERATED_MODEL_FILE"
  fi
  if [ "$SIMULATION_STARTED" -eq 1 ]; then
    prompt_for_test_run_name || true
  fi
  exit "$exit_status"
}

# ---------------------------------------------------------------------------- #
# BUILD
# ---------------------------------------------------------------------------- #

build_workspace() {
  msg "Building workspace..."
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon build --symlink-install --base-paths "$ROOT"
  set +u
  source "$ROOT/install/setup.bash"
  set -u
}

# ---------------------------------------------------------------------------- #
# MODEL PREPARATION
# ---------------------------------------------------------------------------- #

extract_model_name() {
  msg "Extracting model name from model.config..."
  MODEL_NAME="$(python3 -c "
import xml.etree.ElementTree as ET, sys
tree = ET.parse(sys.argv[1])
print((tree.getroot().findtext('name') or '').strip())
" "$MODEL_CONFIG")"

  [[ -n "$MODEL_NAME" && "$MODEL_NAME" =~ ^[A-Za-z0-9_-]+$ ]] || {
    err "Bad model name in $MODEL_CONFIG"
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
  msg "Starting Gazebo..."
  export GZ_SIM_RESOURCE_PATH="$ROOT/worlds:$MODEL_DIR"

  if [ "$RUN_SIMULATION_HEADLESS" = "1" ]; then
    gz sim -s -v4 "$WORLD_FILE" &
  else
    gz sim -v4 "$WORLD_FILE" &
  fi
  GAZEBO_PID=$!
  SIMULATION_STARTED=1
}

wait_for_world() {
  msg "Waiting for Gazebo world '$WORLD'..."
  local ready=0
  for _ in {1..30}; do
    if gz service -l 2>/dev/null | grep -q "/world/$WORLD/create"; then
      ready=1
      break
    fi
    sleep 1
  done
  [ "$ready" = "1" ] || { err "Gazebo world did not become ready"; exit 1; }
}

pause_simulation() {
  msg "Pausing simulation..."
  gz service -s "/world/$WORLD/control" \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
    -r "pause: true" | grep -q "data: true"
}

spawn_model() {
  msg "Spawning model '$MODEL_NAME'..."
  gz service -s "/world/$WORLD/create" \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    -r "sdf_filename: \"$GENERATED_MODEL_FILE\", name: \"$MODEL_NAME\", allow_renaming: false, pose: {position: {x: 0.0, y: 0.0, z: 0.02}}"
}

wait_for_spawn() {
  msg "Waiting for model to spawn..."
  local spawned=0
  for _ in {1..15}; do
    if gz model --list 2>/dev/null | sed -n 's/^[[:space:]]*-[[:space:]]*//p' | grep -Fxq "$MODEL_NAME"; then
      spawned=1
      break
    fi
    sleep 1
  done
  [ "$spawned" = "1" ] || { err "Model not found after spawn"; exit 1; }
}

unpause_simulation() {
  msg "Starting simulation..."
  gz service -s "/world/$WORLD/control" \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
    -r "pause: false" >/dev/null
}

# ---------------------------------------------------------------------------- #
# ROS LAUNCH
# ---------------------------------------------------------------------------- #

launch_ros_nodes() {
  msg "Launching ROS nodes for '$SYSTEM' in ROS domain $ROS_DOMAIN_ID and Gazebo partition $GZ_PARTITION..."
  local launch_rviz="false"
  [ "$OPEN_RVIZ" = "1" ] && launch_rviz="true"

  ros2 launch lunar_simulator "${SYSTEM}_launch.py" \
    system_name:="$SYSTEM" use_sim_time:=true \
    ros_domain_id:="$ROS_DOMAIN_ID" \
    gz_partition:="$GZ_PARTITION" \
    launch_gazebo:=false launch_bridge:=false launch_rviz:="$launch_rviz" &
}

run_bridge() {
  msg "Starting ros-gz bridge..."
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$BRIDGE_CONFIG"
}

# ---------------------------------------------------------------------------- #
# EXECUTE launch_simulator.sh
# ---------------------------------------------------------------------------- #

main() {
  parse_arguments "$@"
  validate_names
  resolve_paths
  validate_source_files
  create_test_run
  start_terminal_capture
  trap finalize_test_run EXIT
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
  run_bridge
  wait "$GAZEBO_PID"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
