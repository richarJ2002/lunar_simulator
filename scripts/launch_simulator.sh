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
# BUILD
# ---------------------------------------------------------------------------- #

build_workspace() {
  msg "Building workspace..."
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  colcon build --symlink-install --base-paths "$ROOT" >/dev/null
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

  trap 'rm -f "$GENERATED_MODEL_FILE"' EXIT
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
  msg "Launching ROS nodes for '$SYSTEM'..."
  local launch_rviz="false"
  [ "$OPEN_RVIZ" = "1" ] && launch_rviz="true"

  ros2 launch lunar_simulator "${SYSTEM}_launch.py" \
    system_name:="$SYSTEM" use_sim_time:=true \
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

main "$@"