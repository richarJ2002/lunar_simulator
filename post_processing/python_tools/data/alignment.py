"""!
@brief  Ground-truth alignment: bracketed linear interpolation for
        position/twist and normalized shortest-path quaternion
        interpolation for attitude, plus the quaternion geodesic distance
        and display-only Euler conversion used to report attitude error.
        Never extrapolates beyond the ground-truth series' own time
        coverage, per the plan's alignment contract. Pure NumPy; no bag or
        Plotly access here.
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

import numpy as np
import numpy.typing as npt

from python_tools.data.models import OdometrySeries, SeriesSegment

# ZYX intrinsic Euler convention (yaw about Z, then pitch about the new Y,
# then roll about the new X) -- matching tf2::getEulerYPR/REP-103's body-
# frame roll/pitch/yaw. Euler angles from `quaternion_to_euler_zyx` use
# this convention exclusively; they are display-only per the plan, never
# fed back into a geodesic error calculation.
EULER_CONVENTION = "ZYX intrinsic (yaw-pitch-roll), REP-103 body frame"


def normalize_quaternion(
    quaternions_xyzw: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Normalizes an array of quaternions to unit norm.

    @param   quaternions_xyzw
             Quaternions, (x, y, z, w), shape (N, 4).

    @return  The normalized quaternions, shape (N, 4). A row with zero norm
             is returned unchanged (there is no meaningful unit quaternion
             to normalize it to).
    """
    norms = np.linalg.norm(quaternions_xyzw, axis=-1, keepdims=True)
    # Guard against a genuinely zero-norm row (malformed input) rather than
    # dividing by zero and producing NaN for the whole row.
    safe_norms = np.where(norms == 0.0, 1.0, norms)
    return quaternions_xyzw / safe_norms


def quaternion_geodesic_angle(
    quaternions_a_xyzw: npt.NDArray[np.float64],
    quaternions_b_xyzw: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes the geodesic (shortest-rotation) angle between pairs
             of quaternions, robust to the unit-quaternion double cover
             (q and -q represent the same rotation).

    @param   quaternions_a_xyzw
             First quaternion of each pair, (x, y, z, w), shape (N, 4).
    @param   quaternions_b_xyzw
             Second quaternion of each pair, same shape.

    @return  The geodesic angle between each pair, radians, shape (N,), in
             `[0, pi]`.
    """
    a = normalize_quaternion(quaternions_a_xyzw)
    b = normalize_quaternion(quaternions_b_xyzw)
    # The dot product's absolute value gives the shorter of the two
    # geodesic paths a quaternion pair can represent (since q and -q are
    # the same rotation); clip guards against a >1 magnitude from floating
    # point error before arccos.
    dot = np.clip(np.abs(np.sum(a * b, axis=-1)), -1.0, 1.0)
    return 2.0 * np.arccos(dot)


def slerp(
    quaternions_start_xyzw: npt.NDArray[np.float64],
    quaternions_end_xyzw: npt.NDArray[np.float64],
    fractions: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Normalized shortest-path spherical linear interpolation
             between paired start/end quaternions.

    @param   quaternions_start_xyzw
             Interval start quaternions, (x, y, z, w), shape (N, 4).
    @param   quaternions_end_xyzw
             Interval end quaternions, same shape.
    @param   fractions
             Interpolation fraction within each interval, shape (N,), in
             `[0, 1]`.

    @return  The interpolated, unit-normalized quaternions, shape (N, 4).
    """
    start = normalize_quaternion(quaternions_start_xyzw)
    end = normalize_quaternion(quaternions_end_xyzw)
    # Take the shortest path: if the dot product is negative, the two
    # quaternions are more than 90 degrees apart in 4-D and slerp would
    # take the long way around unless one endpoint is negated first (q and
    # -q represent the same rotation, so this changes nothing physically).
    dot = np.sum(start * end, axis=-1)
    end = np.where((dot < 0.0)[:, np.newaxis], -end, end)
    dot = np.abs(dot)
    dot = np.clip(dot, -1.0, 1.0)
    angle = np.arccos(dot)
    # Near-parallel quaternions fall back to linear interpolation (sin of
    # a near-zero angle would otherwise divide by a near-zero number).
    small_angle = angle < 1e-6
    sin_angle = np.sin(angle)
    safe_sin_angle = np.where(small_angle, 1.0, sin_angle)
    weight_start = np.where(
        small_angle,
        1.0 - fractions,
        np.sin((1.0 - fractions) * angle) / safe_sin_angle,
    )
    weight_end = np.where(
        small_angle, fractions, np.sin(fractions * angle) / safe_sin_angle
    )
    interpolated = (
        weight_start[:, np.newaxis] * start + weight_end[:, np.newaxis] * end
    )
    return normalize_quaternion(interpolated)


def _bracket_indices(
    query_times_s: npt.NDArray[np.float64], reference_times_s: npt.NDArray[np.float64]
) -> tuple[npt.NDArray[np.intp], npt.NDArray[np.intp], npt.NDArray[np.bool_]]:
    """!
    @brief   Finds, for every query time, the bracketing pair of indices
             into a sorted reference time array, and marks which queries
             fall strictly within the reference series' coverage.

    @param   query_times_s
             Times to bracket, shape (M,).
    @param   reference_times_s
             Strictly increasing reference times, shape (N,).

    @return  `(lower_index, upper_index, in_coverage)`: the bracketing
             index pair for every query (clamped, meaningless where
             `in_coverage` is False), and a mask of queries within
             `[reference_times_s[0], reference_times_s[-1]]`, inclusive of
             both endpoints.
    """
    reference_count = reference_times_s.size
    if reference_count == 0:
        # No reference samples at all: nothing can be in coverage.
        out_of_range = np.zeros(query_times_s.shape, dtype=np.intp)
        return out_of_range, out_of_range, np.zeros(query_times_s.shape, dtype=np.bool_)

    # side="left" gives, for each query, the index of the first reference
    # time >= query; that index minus one is the lower bracket.
    upper_index = np.searchsorted(reference_times_s, query_times_s, side="left")
    lower_index = upper_index - 1
    in_coverage = (lower_index >= 0) & (upper_index < reference_count)

    # An exact match sits AT upper_index under side="left". Collapse it to
    # a zero-width bracket there instead of leaving it spanning to the
    # *previous* sample, for two reasons: (1) a query exactly equal to
    # reference_times_s[0] would otherwise leave lower_index at -1 and be
    # wrongly excluded from coverage (Defect 6 in the plan's 2026-09-22
    # audit); (2) an exact match at any other index must not have its
    # in-coverage/maximum-gap-s eligibility depend on how far away an
    # unrelated neighboring sample happens to be -- hitting a sample
    # exactly is always trustworthy regardless of what is next to it.
    # `np.clip` first keeps the fancy-index lookup below in bounds even
    # for a query beyond the last sample (upper_index == reference_count);
    # `is_within_range` then excludes exactly that case from `exact_match`
    # so it is never mistaken for a real match against the clamped index.
    safe_upper_index = np.clip(upper_index, 0, reference_count - 1)
    is_within_range = upper_index < reference_count
    exact_match = is_within_range & (query_times_s == reference_times_s[safe_upper_index])
    lower_index = np.where(exact_match, safe_upper_index, lower_index)
    upper_index = np.where(exact_match, safe_upper_index, upper_index)
    in_coverage = in_coverage | exact_match

    # Clamp indices so array indexing below never goes out of bounds for
    # an out-of-coverage query; its result is discarded via the mask.
    lower_index = np.clip(lower_index, 0, reference_count - 1)
    upper_index = np.clip(upper_index, 0, reference_count - 1)
    return lower_index, upper_index, in_coverage


def interpolate_ground_truth(
    query_times_s: npt.NDArray[np.float64],
    truth: OdometrySeries,
    maximum_gap_s: float,
) -> tuple[
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.float64],
    npt.NDArray[np.bool_],
]:
    """!
    @brief   Interpolates ground truth to a set of query times using
             bracketed linear interpolation for position/twist and
             normalized shortest-path slerp for orientation, per the
             plan's alignment contract.

    @param   query_times_s
             Times to interpolate ground truth to (typically an estimator's
             own sample times), shape (M,).
    @param   truth
             The ground-truth `OdometrySeries` to interpolate.
    @param   maximum_gap_s
             The largest bracket width still considered a valid
             interpolation; a query bracketed by two truth samples farther
             apart than this is marked invalid rather than trusted, since
             the truth signal may have gone stale across a real gap.

    @return  `(position_m, orientation_xyzw, linear_velocity_mps,
             angular_velocity_radps, valid)`: interpolated truth at every
             query time, shape (M, 3)/(M, 4)/(M, 3)/(M, 3), and a validity
             mask, shape (M,), True where the query fell within truth
             coverage and the bracket was not too wide.
    """
    lower, upper, in_coverage = _bracket_indices(query_times_s, truth.times_s)
    lower_times = truth.times_s[lower]
    upper_times = truth.times_s[upper]
    bracket_width = upper_times - lower_times
    # A zero-width bracket (query time exactly matches a truth sample, or a
    # single-sample truth series) interpolates trivially at fraction 0.
    safe_width = np.where(bracket_width == 0.0, 1.0, bracket_width)
    fraction = np.clip((query_times_s - lower_times) / safe_width, 0.0, 1.0)

    position = (
        truth.position_m[lower]
        + fraction[:, np.newaxis] * (truth.position_m[upper] - truth.position_m[lower])
    )
    linear_velocity = (
        truth.linear_velocity_mps[lower]
        + fraction[:, np.newaxis]
        * (truth.linear_velocity_mps[upper] - truth.linear_velocity_mps[lower])
    )
    angular_velocity = (
        truth.angular_velocity_radps[lower]
        + fraction[:, np.newaxis]
        * (truth.angular_velocity_radps[upper] - truth.angular_velocity_radps[lower])
    )
    orientation = slerp(
        truth.orientation_xyzw[lower], truth.orientation_xyzw[upper], fraction
    )

    valid = in_coverage & (bracket_width <= maximum_gap_s)
    return position, orientation, linear_velocity, angular_velocity, valid


def bracket_width_s(
    query_times_s: npt.NDArray[np.float64], reference_times_s: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes, for every query time, the width of the bracketing
             interval in a reference time series -- how stale the nearest
             surrounding reference samples are relative to each other. Used
             to report ground-truth alignment age alongside an
             interpolated comparison.

    @param   query_times_s
             Times to bracket, shape (M,).
    @param   reference_times_s
             Strictly increasing reference times, shape (N,).

    @return  Bracket width, seconds, shape (M,); `inf` where the query
             falls outside the reference series' coverage entirely.
    """
    lower, upper, in_coverage = _bracket_indices(query_times_s, reference_times_s)
    width = reference_times_s[upper] - reference_times_s[lower]
    return np.where(in_coverage, width, np.inf)


def quaternion_to_euler_zyx(
    quaternions_xyzw: npt.NDArray[np.float64],
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """!
    @brief   Converts quaternions to display-only roll/pitch/yaw using the
             ZYX intrinsic convention documented in `EULER_CONVENTION`.
             Not used for any error calculation -- see
             `quaternion_geodesic_angle` for that.

    @param   quaternions_xyzw
             Quaternions, (x, y, z, w), shape (N, 4).

    @return  `(roll_rad, pitch_rad, yaw_rad)`, each shape (N,).
    """
    x, y, z, w = (
        quaternions_xyzw[:, 0],
        quaternions_xyzw[:, 1],
        quaternions_xyzw[:, 2],
        quaternions_xyzw[:, 3],
    )
    # Standard closed-form ZYX intrinsic Euler extraction.
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Clip guards the asin argument against floating-point overshoot past
    # +/-1 at the gimbal-lock poles.
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def unwrap_per_segment(
    angles_rad: npt.NDArray[np.float64], segments: Sequence[SeriesSegment]
) -> npt.NDArray[np.float64]:
    """!
    @brief   Unwraps a wrapped-angle series independently within each
             continuous segment, so a real discontinuity (a clock reset, a
             visual reset, a data gap) is never smoothed over by unwrap()
             reaching across it.

    @param   angles_rad
             Wrapped angles, radians, shape (N,).
    @param   segments
             The series' own discontinuity-free index ranges.

    @return  The unwrapped angles, shape (N,).
    """
    unwrapped = np.array(angles_rad, dtype=np.float64, copy=True)
    for segment in segments:
        segment_slice = slice(segment.start_index, segment.end_index)
        unwrapped[segment_slice] = np.unwrap(angles_rad[segment_slice])
    return unwrapped


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone quaternion self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Print the geodesic angle, in degrees, between two "
            "whitespace-separated x y z w quaternions."
        )
    )
    parser.add_argument(
        "quaternion_a", type=float, nargs=4, help="First quaternion: x y z w."
    )
    parser.add_argument(
        "quaternion_b", type=float, nargs=4, help="Second quaternion: x y z w."
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the geodesic angle between two
             quaternions supplied on the command line.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    angle_rad = quaternion_geodesic_angle(
        np.array([args.quaternion_a]), np.array([args.quaternion_b])
    )[0]
    print(f"{np.degrees(angle_rad):.4f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
