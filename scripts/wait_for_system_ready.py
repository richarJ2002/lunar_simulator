"""!
@brief  Waits until the start-up supervisor's latched system state says
        READY, so scripts/launch_simulator.sh and the scripts/tests/ harness
        wait on the same readiness contract the command gate enforces
        instead of on a log line.
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Sequence

# Topic the supervisor publishes its latched state on (see
# parameters/systems/alpha/alpha_supervisor/startup_supervisor.yaml).
DEFAULT_TOPIC = "/alpha/system/state"

# Wall-clock seconds to wait before giving up.
DEFAULT_TIMEOUT_S = 120.0


def wait_for_ready(topic: str, timeout_s: float) -> Optional[float]:
    """!
    @brief   Subscribes to the latched system state and blocks until it is
             READY or the wall-clock timeout expires.

    @param   topic
             System state topic (diagnostic_msgs/DiagnosticArray whose
             "system_state" status carries a "state" value).
    @param   timeout_s
             Longest wall-clock wait, seconds.

    @return  The READY message's header stamp in simulated seconds, or
             `None` on timeout.
    """
    # Imported here so `-h` works without a sourced ROS environment.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    # The latest READY stamp seen, filled in by the callback.
    ready_stamp_s: list[float] = []

    def on_state(message: DiagnosticArray) -> None:
        """!
        @brief   Records the stamp of a READY state.

        @param   message
                 One system state message.
        """
        # Only the supervisor's own status carries the state value.
        for status in message.status:
            values = {entry.key: entry.value for entry in status.values}
            if status.name == "system_state" and values.get("state") == "READY":
                stamp = message.header.stamp
                ready_stamp_s.append(stamp.sec + stamp.nanosec * 1e-9)

    rclpy.init()
    try:
        node = rclpy.create_node("wait_for_system_ready")
        # Match the supervisor's latched publisher so a READY published
        # before this script started is still delivered.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(DiagnosticArray, topic, on_state, latched)
        deadline = time.monotonic() + timeout_s
        # Spin in short slices so the timeout is honoured promptly.
        while not ready_stamp_s and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
    return ready_stamp_s[0] if ready_stamp_s else None


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Command-line entry point.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  `0` once READY is seen, `1` on timeout.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    ready_s = wait_for_ready(args.topic, args.timeout_s)
    # Launcher-style lines, each within the 40-character console rule.
    if ready_s is None:
        print(f"{args.prefix}Not READY after {args.timeout_s:.0f} s", flush=True)
        return 1
    print(f"{args.prefix}READY at sim {ready_s:.1f} s", flush=True)
    return 0


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Wait until the start-up supervisor reports READY."
    )
    parser.add_argument(
        "--topic", default=DEFAULT_TOPIC, help=f"System state topic (default: {DEFAULT_TOPIC})."
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"Wall-clock seconds to wait (default: {DEFAULT_TIMEOUT_S:.0f}).",
    )
    parser.add_argument(
        "--prefix", default="", help="Text printed before the result line (e.g. '[MSG] ')."
    )
    return parser


if __name__ == "__main__":
    raise SystemExit(main())
