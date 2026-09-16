#!/usr/bin/env bash
# Verify lunar_simulator build, tests, and live Gazebo/ROS results.
# Appended to the fix plan so every phase can be re-verified consistently.
# Usage: ./scripts/verify_results.sh  (run from workspace root)
set -o pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS_DIR="$ROOT/results"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$RESULTS_DIR/verify_$STAMP.log"
mkdir -p "$RESULTS_DIR"

{
echo "=== lunar_simulator verification $STAMP ==="
echo "ROOT=$ROOT"

echo ""
echo "--- 1. Build (narrow package) ---"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lunar_simulator 2>&1 | tail -5

echo ""
echo "--- 2. Unit tests ---"
source "$ROOT/install/setup.bash"
colcon test --packages-select lunar_simulator --event-handlers console_direct+ 2>&1 | tail -15
colcon test-result --all 2>&1 | tail -10

echo ""
echo "--- 3. Static checks ---"
xmllint --noout "$ROOT/worlds/lunar_surface.sdf" && echo "WORLD_XML_OK"
python3 -m py_compile "$ROOT"/launch/*.py && echo "LAUNCH_PY_OK"
bash -n "$ROOT/scripts/launch_simulator" && echo "LAUNCH_SH_OK"
bash -n "$ROOT/scripts/verify_results.sh" && echo "VERIFY_SH_OK"
gz sdf -k "$ROOT/worlds/lunar_surface.sdf" 2>&1 | tail -2
python3 -c "import yaml; d=yaml.safe_load(open('$ROOT/config/ros_gz_bridge.yaml')); assert isinstance(d, list) and len(d) == 8 and all(x['ros_topic_name'].startswith('/alpha/') for x in d), d; print('ALPHA_BRIDGE_YAML_OK', len(d))"

echo ""
echo "--- 4. Gazebo headless load (bounded) ---"
export GZ_SIM_RESOURCE_PATH="$ROOT/worlds:$ROOT/src/systems/alpha_system/alpha_model"
timeout 10 gz sim -s -v3 "$ROOT/worlds/lunar_surface.sdf" > /tmp/lunar_verify_gz.log 2>&1
echo "GZ_EXIT:$?"
grep -E "World \[lunar_surface\] initialized|Error|Err" /tmp/lunar_verify_gz.log | head -5 || true

echo ""
echo "--- 5. Live ROS nodes (single instance, bounded) ---"
echo "PRE-CHECK (must be empty; stale nodes inflate rates/names):"
timeout 5 ros2 node list 2>&1 | sort || true
(ros2 run lunar_simulator alpha_node > /tmp/v_alpha.log 2>&1 & echo $! > /tmp/v_pa)
sleep 5
echo "TOPICS:"
ros2 topic list 2>&1 | sort
echo "NODES:"
ros2 node list 2>&1 | sort
echo "HZ /localisation/ground_truth/odometry (window 30):"
timeout 10 ros2 topic hz /localisation/ground_truth/odometry --window 30 2>&1 | head -4 || true
echo "TF sample:"
timeout 5 ros2 topic echo /tf --once 2>&1 | head -18 || true
echo "PARAMS:"
ros2 param get /alpha_system_node publish_rate_hz 2>&1 || true
kill "$(cat /tmp/v_pa)" 2>/dev/null || true
sleep 1

echo ""
echo "--- 6. Launch portability (from /tmp, no Gazebo start) ---"
timeout 15 ros2 launch lunar_simulator alpha_system_launch.py --show-args 2>&1 | head -12 || true

echo ""
echo "=== verification complete ==="
} 2>&1 | tee "$OUT"

echo "Results written to $OUT"
