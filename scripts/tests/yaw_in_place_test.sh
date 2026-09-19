#!/bin/bash
#
# @File:         yaw_in_place_test.sh
#
# @Brief:        Spins Alpha in place (no translation) and checks the fused
#                estimate stays close to ground truth throughout. Isolates
#                the wheel-odometry rotational-slip issue from any
#                translation-coupled effect -- see circle_test.sh and
#                CLAUDE.md's Key Invariants.
#
# @Date:         18/09/2026
#

set -euo pipefail

usage() {
  echo "Usage: $(basename "$0") [--angular-z RAD_PS] [--duration S] [--rviz] [--help]"
  echo "  --angular-z   commanded yaw rate in rad/s (default: 0.02)"
  echo "  --duration    total spin/check duration in seconds (default: 90)"
  echo "  --rviz        open RViz showing ground truth (green) vs kalman filter (red) paths"
  echo "  --help        show this help"
}

# A pure in-place turn's limiting wheel is whichever is farthest from the
# rotation center (the rear wheels, ~0.937 m out); 0.02 rad/s keeps that
# wheel comfortably under Alpha's physical maximum_wheel_speed_radps (0.14,
# matching the ExoMars rover's own hardware limit -- see
# AckermannControllerNode's own doc comment) with no ackermann_controller
# scale-down needed.
ANGULAR_Z_RADPS="0.02"
DURATION_S="90"
ENABLE_RVIZ=0
while [ $# -gt 0 ]; do
  case "$1" in
    --angular-z) ANGULAR_Z_RADPS="$2"; shift 2 ;;
    --duration) DURATION_S="$2"; shift 2 ;;
    --rviz) ENABLE_RVIZ=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=test_common.sh
source "$SCRIPT_DIR/test_common.sh"
trap tc::cleanup EXIT

echo "[test] === yaw_in_place_test: angular.z=${ANGULAR_Z_RADPS} rad/s, ${DURATION_S}s ==="

tc::start_simulator "$ENABLE_RVIZ"
tc::publish_twist 0.0 "$ANGULAR_Z_RADPS"

# No translation at all, so a tight horizontal bound is appropriate here
# (unlike circle_test.sh, there is no legitimate translation for fusion
# lag to show up as); z_err keeps the same bound as circle_test.sh since
# the same underlying cyclical residual can still appear from attitude
# changes alone.
if tc::check_bounded_error "$DURATION_S" 10 2.0 2.0; then
  RESULT=0
else
  RESULT=$?
fi

if [ "$RESULT" -eq 0 ]; then
  echo "[test] PASS: yaw_in_place_test"
else
  echo "[test] FAIL: yaw_in_place_test (fused estimate exceeded bounds -- see samples above)" >&2
fi

exit "$RESULT"
