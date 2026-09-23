"""!
@brief  The post-processing time contract: combining ROS stamps into
        nanoseconds, converting to elapsed simulation seconds, and splitting
        a series into discontinuity-free segments so no plot or metric ever
        interpolates through a clock reset, a visual reset or a data gap
        larger than the configured maximum (see the plan's "Time contract"
        section). Pure functions only; no bag or ROS message access here.
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

import numpy as np
import numpy.typing as npt

from python_tools.data.models import ResetReason, SeriesSegment

# sensor_msgs/msg/Imu's documented convention for "orientation not
# estimated": orientation_covariance[0] == -1. Shared here because both
# extractors.py and this module's callers need the same literal value.
ORIENTATION_UNAVAILABLE_COVARIANCE_MARKER = -1.0

# Nanoseconds per second, used throughout this module's time conversions.
NANOSECONDS_PER_SECOND = 1_000_000_000


def header_stamp_to_ns(seconds: int, nanoseconds: int) -> int:
    """!
    @brief   Combines a builtin_interfaces/msg/Time-style (sec, nanosec)
             pair into one signed 64-bit nanosecond count.

    @param   seconds
             The message header stamp's `sec` field.
    @param   nanoseconds
             The message header stamp's `nanosec` field (0 to 999,999,999).

    @return  The combined timestamp, nanoseconds since the ROS/Unix epoch.
    """
    # Widen to Python int before multiplying so this never overflows a
    # fixed-width type regardless of the caller's own integer width.
    return int(seconds) * NANOSECONDS_PER_SECOND + int(nanoseconds)


def elapsed_seconds(
    times_ns: npt.NDArray[np.int64], start_time_ns: int
) -> npt.NDArray[np.float64]:
    """!
    @brief   Converts absolute nanosecond timestamps to elapsed simulation
             seconds relative to the run's earliest valid captured
             timestamp, per the plan's display-normalization requirement.

    @param   times_ns
             Absolute timestamps, nanoseconds since epoch, shape (N,).
    @param   start_time_ns
             The run's elapsed-zero reference timestamp.

    @return  Elapsed seconds, shape (N,), as float64.
    """
    # Subtract the shared reference first in integer nanoseconds (exact),
    # then convert to floating-point seconds only at the very end so large
    # absolute epoch values never lose precision in the subtraction itself.
    return (times_ns.astype(np.int64) - np.int64(start_time_ns)).astype(
        np.float64
    ) / NANOSECONDS_PER_SECOND


def segment_series(
    times_ns: npt.NDArray[np.int64],
    maximum_gap_s: float,
    reset_event_times_ns: Optional[npt.NDArray[np.int64]] = None,
) -> tuple[SeriesSegment, ...]:
    """!
    @brief   Splits one topic's timestamps into contiguous segments so nothing
             downstream ever interpolates or computes a rate across a
             backward time jump, an oversized gap, or an explicit visual
             reset event, per the plan's time contract.

    @param   times_ns
             Strictly non-decreasing-within-segment candidate timestamps,
             shape (N,), already sorted by arrival order.
    @param   maximum_gap_s
             The largest inter-sample gap, in seconds, that is still
             considered one continuous segment.
    @param   reset_event_times_ns
             Timestamps of /alpha/localisation/visual/reset events, if this
             series is visual odometry's own series; None for every other
             topic, which has no such reset concept.

    @return  Segments covering every index of `times_ns` exactly once, in
             ascending order.
    """
    # An empty series has no segments at all.
    if times_ns.size == 0:
        return ()

    # Track where the current segment began and collect finished segments.
    segments: list[SeriesSegment] = []
    segment_start_index = 0
    maximum_gap_ns = int(maximum_gap_s * NANOSECONDS_PER_SECOND)

    # Pre-sort reset event times once so each can be located with a single
    # forward-moving pointer instead of rescanning on every sample.
    reset_times = (
        np.sort(reset_event_times_ns)
        if reset_event_times_ns is not None
        else np.empty(0, dtype=np.int64)
    )
    reset_pointer = 0

    # Walk every consecutive pair, closing the current segment and opening a
    # new one whenever a discontinuity is found.
    for index in range(1, times_ns.size):
        previous_time_ns = int(times_ns[index - 1])
        current_time_ns = int(times_ns[index])
        reason: Optional[ResetReason] = None

        # A later sample arriving before an earlier one already seen is
        # always a discontinuity, regardless of magnitude.
        if current_time_ns < previous_time_ns:
            reason = ResetReason.BACKWARD_TIME_JUMP
        # Otherwise, a sufficiently large forward gap is also one.
        elif current_time_ns - previous_time_ns > maximum_gap_ns:
            reason = ResetReason.MAXIMUM_GAP_EXCEEDED

        # Advance the reset-event pointer past every reset that occurred at
        # or before the previous sample; any reset strictly between the
        # previous and current sample means this pair straddles a reset.
        while (
            reset_pointer < reset_times.size
            and reset_times[reset_pointer] <= previous_time_ns
        ):
            reset_pointer += 1
        if (
            reason is None
            and reset_pointer < reset_times.size
            and reset_times[reset_pointer] <= current_time_ns
        ):
            reason = ResetReason.VISUAL_RESET_EVENT

        # A discontinuity was found: close the segment ending at `index`
        # (exclusive) and start a new one at `index`.
        if reason is not None:
            segments.append(SeriesSegment(segment_start_index, index, None))
            segment_start_index = index
            # Record the reason on the segment that begins here, since a
            # segment's own start is the discontinuity that produced it;
            # the sentinel `None` above is replaced immediately below.
            segments[-1] = SeriesSegment(
                segments[-1].start_index, segments[-1].end_index, reason
            )

    # The reason recorded on the very first emitted segment describes the
    # discontinuity that ENDED it, not the (nonexistent) one that began it;
    # rebuild the segment list so each segment's `reason` instead describes
    # why the discontinuity BEFORE it (relative to the prior segment)
    # occurred, matching SeriesSegment's documented semantics of describing
    # why the previous segment ended.
    resegmented: list[SeriesSegment] = []
    previous_reason: Optional[ResetReason] = None
    for closed_segment in segments:
        resegmented.append(
            SeriesSegment(
                closed_segment.start_index,
                closed_segment.end_index,
                previous_reason,
            )
        )
        previous_reason = closed_segment.reason
    # Append the final, still-open segment running to the end of the array.
    resegmented.append(
        SeriesSegment(segment_start_index, times_ns.size, previous_reason)
    )
    return tuple(resegmented)


def is_valid_timestamp_ns(value_ns: int) -> bool:
    """!
    @brief   Reports whether one nanosecond timestamp is a real capture
             time rather than the ROS convention for "unset" (an all-zero
             header stamp) or a negative value.

    @param   value_ns
             A single candidate timestamp, nanoseconds since epoch.

    @return  `True` if `value_ns` is a plausible real capture time.
    """
    # An int64 cannot hold NaN/inf, so "non-finite" here means the ROS
    # convention for an unset stamp (exact epoch zero) or a negative value.
    return value_ns > 0


def filter_finite_timestamps(
    times_ns: npt.NDArray[np.int64],
) -> npt.NDArray[np.bool_]:
    """!
    @brief   Builds a boolean mask rejecting non-finite or clearly invalid
             timestamps, per the plan's "reject non-finite timestamps"
             requirement. Vectorized form of `is_valid_timestamp_ns`.

    @param   times_ns
             Candidate timestamps, nanoseconds since epoch, shape (N,).

    @return  A boolean mask, shape (N,), True where the timestamp is valid.
    """
    return times_ns > 0


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    # Describe the tool so `-h` output is self-explanatory.
    parser = argparse.ArgumentParser(
        description=(
            "Print elapsed-second conversion and segmentation for a "
            "whitespace-separated list of nanosecond timestamps, for "
            "manual inspection of the time contract."
        )
    )
    # Positional argument: the raw nanosecond timestamps to process.
    parser.add_argument(
        "timestamps_ns",
        type=int,
        nargs="+",
        help="Nanosecond timestamps to segment, in arrival order.",
    )
    # Optional flag controlling the maximum accepted inter-sample gap.
    parser.add_argument(
        "--maximum-gap-s",
        type=float,
        default=1.0,
        help="Largest inter-sample gap treated as one segment (default: 1.0).",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints elapsed seconds and detected segments
             for a manually supplied list of nanosecond timestamps.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    # Convert the parsed integers into the array shape every function here
    # expects.
    times_ns = np.array(args.timestamps_ns, dtype=np.int64)
    # Report elapsed seconds relative to the first supplied timestamp.
    print("elapsed_s:", elapsed_seconds(times_ns, int(times_ns[0])))
    # Report the detected segments so a user can see split points directly.
    for segment in segment_series(times_ns, args.maximum_gap_s):
        print(segment)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
