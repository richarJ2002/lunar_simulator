#!/bin/bash
#
# @File:         square_test.sh
#
# @Brief:        Drives Alpha around a four-sided square path (straight leg,
#                90 degree in-place turn, repeated four times) and checks
#                the fused estimate stays close to ground truth throughout.
#                Exercises straight-line driving and sharp turns in the
#                same run, unlike circle_test.sh (continuous combined
#                motion) and yaw_in_place_test.sh (turning only).
#
# @Date:         18/09/2026
#

set -euo pipefail

usage() {
  echo "Usage: $(basename "$0") [--side-length M] [--linear-x M_PS] [--angular-z RAD_PS] [--rviz] [--help]"
  echo "  --side-length  length of each side in metres (default: 2.0)"
  echo "  --linear-x     commanded forward speed in m/s (default: 0.015)"
  echo "  --angular-z    commanded yaw rate for each corner turn in rad/s (default: 0.02)"
  echo "  --rviz         open RViz showing ground truth (green) vs kalman filter (red) paths"
  echo "  --help         show this help"
}

# Each straight leg is pure translation and each corner turn is pure
# rotation, so both defaults sit comfortably under Alpha's physical
# maximum_wheel_speed_radps (0.14, matching the ExoMars rover's own
# ~0.02 m/s hardware limit -- see AckermannControllerNode's own doc
# comment) with no ackermann_controller scale-down needed, matching
# circle_test.sh's and yaw_in_place_test.sh's own updated defaults.
SIDE_LENGTH_M="2.0"
LINEAR_X_MPS="0.015"
ANGULAR_Z_RADPS="0.02"
ENABLE_RVIZ=0
while [ $# -gt 0 ]; do
  case "$1" in
    --side-length) SIDE_LENGTH_M="$2"; shift 2 ;;
    --linear-x) LINEAR_X_MPS="$2"; shift 2 ;;
    --angular-z) ANGULAR_Z_RADPS="$2"; shift 2 ;;
    --rviz) ENABLE_RVIZ=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=test_common.sh
source "$SCRIPT_DIR/test_common.sh"
trap tc::cleanup EXIT

# Duration of each straight leg and each 90 degree corner turn, derived
# from the requested speed/rate rather than hardcoded, so --linear-x and
# --angular-z change the path timing consistently with what was actually
# commanded. Rounded to whole seconds: tc::check_bounded_error divides by
# its interval using plain bash arithmetic, which cannot parse a decimal.
DRIVE_DURATION_S="$(python3 -c "print(round(${SIDE_LENGTH_M} / ${LINEAR_X_MPS}))")"
TURN_DURATION_S="$(python3 -c "import math; print(round((math.pi / 2) / ${ANGULAR_Z_RADPS}))")"
CHECK_INTERVAL_S=5

echo "[test] === square_test: side=${SIDE_LENGTH_M} m, linear.x=${LINEAR_X_MPS} m/s, angular.z=${ANGULAR_Z_RADPS} rad/s ==="
echo "[test] each side: ${DRIVE_DURATION_S}s straight, ${TURN_DURATION_S}s turn"

tc::start_simulator "$ENABLE_RVIZ"

OVERALL_RESULT=0
for SIDE in 1 2 3 4; do
  echo "[test] --- side $SIDE/4: straight leg ---"
  tc::publish_twist "$LINEAR_X_MPS" 0.0
  tc::check_bounded_error "$DRIVE_DURATION_S" "$CHECK_INTERVAL_S" 5.0 2.0 || OVERALL_RESULT=1

  echo "[test] --- side $SIDE/4: corner turn ---"
  tc::publish_twist 0.0 "$ANGULAR_Z_RADPS"
  tc::check_bounded_error "$TURN_DURATION_S" "$CHECK_INTERVAL_S" 5.0 2.0 || OVERALL_RESULT=1
done

if [ "$OVERALL_RESULT" -eq 0 ]; then
  echo "[test] PASS: square_test"
else
  echo "[test] FAIL: square_test (fused estimate exceeded bounds on at least one leg -- see samples above)" >&2
fi

exit "$OVERALL_RESULT"
