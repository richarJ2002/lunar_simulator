#!/bin/bash
#
# @File:         circle_test.sh
#
# @Brief:        Drives Alpha in a circle and checks the fused estimate
#                stays close to ground truth throughout. Regression test
#                for the wheel-odometry rotational-slip issue documented
#                in CLAUDE.md's Key Invariants (mitigated via
#                gyroOnlyYawRad and capped yaw-variance inflation, see
#                AlphaKalmanFilterNode.h).
#
# @Date:         18/09/2026
#

set -euo pipefail

usage() {
  echo "Usage: $(basename "$0") [--linear-x M_PS] [--angular-z RAD_PS] [--duration S] [--rviz] [--help]"
  echo "  --linear-x    commanded forward speed in m/s (default: 0.015)"
  echo "  --angular-z   commanded yaw rate in rad/s (default: 0.01)"
  echo "  --duration    total drive/check duration in seconds (default: 120)"
  echo "  --rviz        open RViz showing ground truth (green) vs kalman filter (red) paths"
  echo "  --help        show this help"
}

# Defaults keep the same 1.5:1 linear:angular ratio as before, scaled down
# to fit Alpha's physical maximum_wheel_speed_radps (0.14, matching the
# ExoMars rover's own ~0.02 m/s hardware limit -- see
# AckermannControllerNode's own doc comment); a combined drive+turn command
# this close to the per-wheel limit on both axes simultaneously still
# triggers a small additional proportional scale-down from
# ackermann_controller, which is expected, not a bug.
LINEAR_X_MPS="0.015"
ANGULAR_Z_RADPS="0.01"
DURATION_S="120"
ENABLE_RVIZ=0
while [ $# -gt 0 ]; do
  case "$1" in
    --linear-x) LINEAR_X_MPS="$2"; shift 2 ;;
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

echo "[test] === circle_test: linear.x=${LINEAR_X_MPS} m/s, angular.z=${ANGULAR_Z_RADPS} rad/s, ${DURATION_S}s ==="

tc::start_simulator "$ENABLE_RVIZ"
tc::publish_twist "$LINEAR_X_MPS" "$ANGULAR_Z_RADPS"

# Bounds looser than ordinary fusion noise/lag but far tighter than the
# catastrophic runaway (234,546 m) this test is designed to catch -- see
# CLAUDE.md's Key Invariants note on maximum_wheel_yaw_variance_inflation.
# z_err's 2.0 m bound accommodates the smaller, already-documented
# cyclical Z oscillation seen during sustained circular driving (also
# CLAUDE.md), which is a known, separate, non-emergency residual.
if tc::check_bounded_error "$DURATION_S" 10 5.0 2.0; then
  RESULT=0
else
  RESULT=$?
fi

if [ "$RESULT" -eq 0 ]; then
  echo "[test] PASS: circle_test"
else
  echo "[test] FAIL: circle_test (fused estimate exceeded bounds -- see samples above)" >&2
fi

exit "$RESULT"
