"""!
@brief  Shared, pure metric functions operating on the typed series in
        python_tools.data.models and the alignment primitives in
        python_tools.data.alignment: position/attitude/velocity error
        summaries, sample-rate and gap health, covariance-derived standard
        deviations and normalized residuals, and six-wheel speed
        conversion. Every function here is deterministic and side-effect
        free, so it is testable against synthetic arrays without a bag.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
import numpy.typing as npt

from python_tools.data.alignment import quaternion_geodesic_angle
from python_tools.data.models import WHEEL_COUNT


@dataclass(frozen=True)
class ErrorSummary:
    """!
    @brief  Standard summary statistics for one error magnitude series.
    """

    # Root-mean-square of the error magnitudes.
    rmse: float
    # Median error magnitude.
    median: float
    # 95th-percentile error magnitude.
    p95: float
    # Maximum error magnitude observed.
    maximum: float
    # Number of samples the summary was computed over (after dropping
    # invalid/unaligned entries), so a near-empty summary is distinguishable
    # from a genuinely tight one.
    sample_count: int


def summarize_errors(magnitudes: npt.NDArray[np.float64]) -> ErrorSummary:
    """!
    @brief   Computes RMSE/median/p95/maximum over a series of error
             magnitudes, ignoring any NaN (unaligned/invalid) entries.

    @param   magnitudes
             Non-negative error magnitudes, shape (N,); NaN marks an entry
             with no valid ground-truth alignment.

    @return  The computed `ErrorSummary`. All fields are NaN and
             `sample_count` is 0 if no valid entries remain.
    """
    valid = magnitudes[np.isfinite(magnitudes)]
    if valid.size == 0:
        return ErrorSummary(
            rmse=float("nan"),
            median=float("nan"),
            p95=float("nan"),
            maximum=float("nan"),
            sample_count=0,
        )
    return ErrorSummary(
        rmse=float(np.sqrt(np.mean(valid**2))),
        median=float(np.median(valid)),
        p95=float(np.percentile(valid, 95)),
        maximum=float(np.max(valid)),
        sample_count=int(valid.size),
    )


def position_error_by_axis(
    estimate_position_m: npt.NDArray[np.float64],
    truth_position_m: npt.NDArray[np.float64],
    valid: npt.NDArray[np.bool_],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes signed per-axis position error, masking out entries
             with no valid ground-truth alignment.

    @param   estimate_position_m
             Estimator position, metres, shape (N, 3).
    @param   truth_position_m
             Interpolated ground-truth position, metres, same shape.
    @param   valid
             Per-sample ground-truth alignment validity, shape (N,).

    @return  Signed per-axis error, metres, shape (N, 3); NaN where
             `valid` is False.
    """
    error = estimate_position_m - truth_position_m
    return np.where(valid[:, np.newaxis], error, np.nan)


def horizontal_norm(vectors_xyz: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes the horizontal (X-Y plane) magnitude of a per-sample
             3-vector series.

    @param   vectors_xyz
             3-vectors, shape (N, 3).

    @return  The horizontal magnitude, shape (N,).
    """
    return np.linalg.norm(vectors_xyz[:, :2], axis=-1)


def norm_3d(vectors_xyz: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes the full 3-D magnitude of a per-sample 3-vector
             series.

    @param   vectors_xyz
             3-vectors, shape (N, 3).

    @return  The 3-D magnitude, shape (N,).
    """
    return np.linalg.norm(vectors_xyz, axis=-1)


def attitude_error_deg(
    estimate_orientation_xyzw: npt.NDArray[np.float64],
    truth_orientation_xyzw: npt.NDArray[np.float64],
    valid: npt.NDArray[np.bool_],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes the quaternion geodesic attitude error, in degrees,
             masking out entries with no valid ground-truth alignment.

    @param   estimate_orientation_xyzw
             Estimator orientation, (x, y, z, w), shape (N, 4).
    @param   truth_orientation_xyzw
             Interpolated ground-truth orientation, same shape.
    @param   valid
             Per-sample ground-truth alignment validity, shape (N,).

    @return  Geodesic attitude error, degrees, shape (N,); NaN where
             `valid` is False.
    """
    angle_rad = quaternion_geodesic_angle(
        estimate_orientation_xyzw, truth_orientation_xyzw
    )
    return np.where(valid, np.degrees(angle_rad), np.nan)


def wrapped_angle_difference(
    angle_a_rad: npt.NDArray[np.float64], angle_b_rad: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes `angle_a - angle_b`, wrapped to `[-pi, pi]`, for a
             display-only per-axis Euler error (never used for a geodesic
             error metric -- see `attitude_error_deg` for that).

    @param   angle_a_rad
             Minuend angles, radians, any shape.
    @param   angle_b_rad
             Subtrahend angles, radians, same shape.

    @return  The wrapped difference, radians, same shape.
    """
    difference = angle_a_rad - angle_b_rad
    # Wrap into (-pi, pi] using the standard atan2(sin, cos) identity,
    # which is exact and avoids the branch-heavy modulo-based approach.
    return np.arctan2(np.sin(difference), np.cos(difference))


def velocity_error(
    estimate_velocity: npt.NDArray[np.float64],
    truth_velocity: npt.NDArray[np.float64],
    valid: npt.NDArray[np.bool_],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes signed body-frame velocity error (linear or
             angular), masking out entries with no valid ground-truth
             alignment.

    @param   estimate_velocity
             Estimator body-frame velocity, shape (N, 3).
    @param   truth_velocity
             Interpolated ground-truth body-frame velocity, same shape.
    @param   valid
             Per-sample ground-truth alignment validity, shape (N,).

    @return  Signed error, shape (N, 3); NaN where `valid` is False.
    """
    error = estimate_velocity - truth_velocity
    return np.where(valid[:, np.newaxis], error, np.nan)


def sample_rate_hz(times_s: npt.NDArray[np.float64]) -> Optional[float]:
    """!
    @brief   Computes a series' overall effective sample rate.

    @param   times_s
             Sample times, seconds, shape (N,), ascending.

    @return  `(N - 1) / (times_s[-1] - times_s[0])`, or `None` if fewer
             than two samples are available or the span is zero.
    """
    if times_s.size < 2:
        return None
    span = float(times_s[-1] - times_s[0])
    if span <= 0.0:
        return None
    return (times_s.size - 1) / span


def inter_sample_gaps_s(times_s: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes every inter-sample gap in a time series.

    @param   times_s
             Sample times, seconds, shape (N,), ascending.

    @return  Gaps between consecutive samples, seconds, shape (N - 1,).
    """
    return np.diff(times_s)


def absence_intervals_s(
    times_s: npt.NDArray[np.float64], maximum_gap_s: float
) -> list[tuple[float, float]]:
    """!
    @brief   Finds every interval where a series produced no samples for
             longer than the configured maximum gap.

    @param   times_s
             Sample times, seconds, shape (N,), ascending.
    @param   maximum_gap_s
             The largest gap still considered normal sampling, not an
             absence.

    @return  A list of `(start_s, end_s)` absence intervals, in ascending
             order.
    """
    if times_s.size < 2:
        return []
    gaps = inter_sample_gaps_s(times_s)
    absence_starts = np.nonzero(gaps > maximum_gap_s)[0]
    return [(float(times_s[i]), float(times_s[i + 1])) for i in absence_starts]


def covariance_std_dev(
    covariance: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Extracts the per-axis standard deviation from a series of 6x6
             pose/twist covariance matrices.

    @param   covariance
             Row-major 6x6 covariance matrices, shape (N, 6, 6).

    @return  Standard deviation per axis, shape (N, 6); zero for a
             negative diagonal entry (numerically invalid covariance)
             rather than a NaN from `sqrt` of a negative number.
    """
    diagonal = np.diagonal(covariance, axis1=-2, axis2=-1)
    return np.sqrt(np.clip(diagonal, 0.0, None))


def normalized_residual(
    error: npt.NDArray[np.float64], std_dev: npt.NDArray[np.float64]
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes a per-axis normalized residual (error divided by its
             reported standard deviation), for a +/-3-sigma consistency
             plot.

    @param   error
             Signed error, any shape.
    @param   std_dev
             Reported standard deviation, same shape as `error`.

    @return  The normalized residual, same shape; NaN where `std_dev` is
             zero (an unpopulated or degenerate covariance) rather than a
             division-by-zero `inf`.
    """
    safe_std_dev = np.where(std_dev > 0.0, std_dev, np.nan)
    return error / safe_std_dev


@dataclass(frozen=True)
class SigmaCoverage:
    """!
    @brief  Fraction of errors inside the reported one- and three-sigma
            bounds, the empirical check of whether a covariance is
            consistent (about 68.3 % and 99.7 % for a Gaussian error).
    """

    # Fraction of valid samples with |error| <= 1 sigma, in [0, 1].
    within_one_sigma: float
    # Fraction of valid samples with |error| <= 3 sigma, in [0, 1].
    within_three_sigma: float
    # Number of samples with a finite error and a positive sigma.
    sample_count: int


def high_frequency_residual(
    times_s: npt.NDArray[np.float64],
    values: npt.NDArray[np.float64],
    window_s: float = 2.0,
) -> npt.NDArray[np.float64]:
    """!
    @brief   Computes each sample's residual against a centred moving mean
             of its own series, isolating fast jitter from slow drift.

    The mean is taken over every sample whose time lies within
    `window_s / 2` of the sample, so an irregular or bursty publication
    rate does not change the window's physical length. Samples closer than
    `window_s / 2` to either end have a truncated window and are returned
    as NaN rather than biased.

    @param   times_s
             Sample times, seconds, shape (N,), ascending.
    @param   values
             Sampled values, shape (N,) or (N, K); columns are smoothed
             independently.
    @param   window_s
             Full width of the centred averaging window, seconds.

    @return  Residual `values - moving_mean`, same shape as `values`; NaN
             for edge samples and for any non-finite input.

    @throws  ValueError
             If `window_s` is not positive or the shapes disagree.
    """
    # Reject a window that could never contain more than one sample.
    if not window_s > 0.0:
        raise ValueError("window_s must be positive")
    # Treat a 1-D series as a single column so both shapes share one path.
    columns = values.reshape(values.shape[0], -1) if values.ndim > 1 else values[:, np.newaxis]
    # Every row of values must have one sample time.
    if columns.shape[0] != times_s.shape[0]:
        raise ValueError("times_s and values must have the same length")
    # An empty series has no residual to report.
    if times_s.size == 0:
        return np.full(values.shape, np.nan)
    # Index range [start, end) of the samples inside each centred window.
    half_window_s = 0.5 * window_s
    window_start = np.searchsorted(times_s, times_s - half_window_s, side="left")
    window_end = np.searchsorted(times_s, times_s + half_window_s, side="right")
    # Prefix sums turn every window mean into two lookups (O(N) overall).
    # Non-finite samples contribute neither value nor count, so one bad
    # sample cannot poison every later window.
    is_finite = np.isfinite(columns)
    finite_values = np.where(is_finite, columns, 0.0)
    zero_row = np.zeros((1, columns.shape[1]))
    prefix_sum = np.vstack([zero_row, np.cumsum(finite_values, axis=0)])
    prefix_count = np.vstack([zero_row, np.cumsum(is_finite, axis=0)])
    window_count = prefix_count[window_end] - prefix_count[window_start]
    # A window with no finite sample has an undefined mean.
    with np.errstate(invalid="ignore", divide="ignore"):
        moving_mean = (prefix_sum[window_end] - prefix_sum[window_start]) / window_count
    residual = np.where(is_finite, columns - moving_mean, np.nan)
    # Mask samples whose window would extend beyond either end of the data.
    is_interior = (times_s - half_window_s >= times_s[0]) & (
        times_s + half_window_s <= times_s[-1]
    )
    residual[~is_interior, :] = np.nan
    # Return the same dimensionality the caller supplied.
    return residual.reshape(values.shape)


def maximum_step_norm(positions: npt.NDArray[np.float64]) -> float:
    """!
    @brief   Finds the largest change between consecutive samples of a
             vector series, e.g. the largest per-message position jump.

    @param   positions
             Sampled vectors, shape (N, K).

    @return  Largest Euclidean norm of a consecutive difference, in the
             series' own unit; NaN when fewer than two finite samples
             exist.
    """
    # A single sample has no step.
    if positions.shape[0] < 2:
        return float("nan")
    # Norm of each consecutive difference, ignoring non-finite pairs.
    step_norms = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    finite_steps = step_norms[np.isfinite(step_norms)]
    # No finite step means there is nothing meaningful to report.
    if finite_steps.size == 0:
        return float("nan")
    return float(np.max(finite_steps))


def duplicate_stamp_count(times_ns: npt.NDArray[np.int64]) -> int:
    """!
    @brief   Counts messages whose header stamp equals the previous
             message's stamp exactly.

    @param   times_ns
             Header stamps in publication order, nanoseconds, shape (N,).

    @return  Number of samples i > 0 with `times_ns[i] == times_ns[i-1]`.
    """
    # Integer nanosecond stamps compare exactly; no tolerance is needed.
    return int(np.count_nonzero(np.diff(times_ns) == 0))


def identical_sample_count(samples: npt.NDArray[np.float64]) -> int:
    """!
    @brief   Counts samples that repeat the previous sample exactly, e.g. an
             estimator republishing an unchanged state.

    Exact equality is the intended contract: a republished state carries
    bit-identical values, whereas a genuinely propagated one does not.

    @param   samples
             Per-sample value vectors, shape (N, K), in publication order.

    @return  Number of rows i > 0 equal to row i-1 in every column.
    """
    # Fewer than two rows cannot contain a repeat.
    if samples.shape[0] < 2:
        return 0
    # A row repeats when every one of its values equals the previous row's.
    is_repeat = np.all(samples[1:] == samples[:-1], axis=1)
    return int(np.count_nonzero(is_repeat))


def sigma_coverage(
    error: npt.NDArray[np.float64], std_dev: npt.NDArray[np.float64]
) -> SigmaCoverage:
    """!
    @brief   Measures how often an error falls inside its own reported one-
             and three-sigma bounds.

    @param   error
             Signed error, any shape; NaN marks an unaligned sample.
    @param   std_dev
             Reported standard deviation, same shape and unit as `error`.

    @return  The coverage fractions over samples with a finite error and a
             positive, finite sigma; NaN fractions when there are none.
    """
    # Only samples with both a usable error and a usable sigma count.
    is_usable = np.isfinite(error) & np.isfinite(std_dev) & (std_dev > 0.0)
    usable_count = int(np.count_nonzero(is_usable))
    # Without usable samples the coverage is undefined, not zero.
    if usable_count == 0:
        return SigmaCoverage(float("nan"), float("nan"), 0)
    # Normalise each usable error by its own sigma.
    normalized = np.abs(error[is_usable]) / std_dev[is_usable]
    return SigmaCoverage(
        within_one_sigma=float(np.count_nonzero(normalized <= 1.0)) / usable_count,
        within_three_sigma=float(np.count_nonzero(normalized <= 3.0)) / usable_count,
        sample_count=usable_count,
    )


def direction_of_travel_error_deg(
    times_s: npt.NDArray[np.float64],
    estimate_position_m: npt.NDArray[np.float64],
    truth_position_m: npt.NDArray[np.float64],
    valid: npt.NDArray[np.bool_],
    window_s: float = 30.0,
    minimum_truth_travel_m: float = 0.25,
) -> npt.NDArray[np.float64]:
    """!
    @brief   Compares an estimator's horizontal direction of travel with
             ground truth's over sliding time windows.

    For each valid sample at time t, the window runs from the first valid
    sample at or after `t - window_s` to that sample. The error is the
    signed angle from the truth displacement to the estimate displacement
    in the X-Y plane of the shared fixed frame, so a translation-direction
    bias shows up even when heading (yaw) itself is accurate.

    @param   times_s
             Sample times, seconds, shape (N,), ascending.
    @param   estimate_position_m
             Estimator position, metres, shape (N, 3), fixed frame.
    @param   truth_position_m
             Ground-truth position interpolated to `times_s`, metres,
             shape (N, 3), same frame.
    @param   valid
             Per-sample ground-truth alignment validity, shape (N,).
    @param   window_s
             Window length, seconds.
    @param   minimum_truth_travel_m
             Ground-truth horizontal travel below which a window's
             direction is undefined and reported as NaN. The 0.25 m
             default keeps centimetre-level position noise from dominating
             the angle (2 cm across 0.25 m is already about 5 degrees).

    @return  Signed direction error, degrees in (-180, 180], shape (N,);
             NaN for invalid samples, short windows and near-stationary
             windows.

    @throws  ValueError
             If `window_s` or `minimum_truth_travel_m` is not positive.
    """
    # Both limits define physical scales and must be positive.
    if not window_s > 0.0 or not minimum_truth_travel_m > 0.0:
        raise ValueError("window_s and minimum_truth_travel_m must be positive")
    # Start from "undefined" everywhere and fill in each defined window.
    direction_error_deg = np.full(times_s.shape, np.nan)
    valid_indices = np.nonzero(valid)[0]
    # A window needs at least two valid samples.
    if valid_indices.size < 2:
        return direction_error_deg
    valid_times_s = times_s[valid_indices]
    # First valid sample inside each window, found for all windows at once.
    window_start = np.searchsorted(valid_times_s, valid_times_s - window_s, side="left")
    for window_end, window_first in enumerate(window_start):
        # Skip windows that have not yet accumulated their full length.
        if valid_times_s[window_end] - valid_times_s[0] < window_s:
            continue
        end_index = valid_indices[window_end]
        start_index = valid_indices[window_first]
        # Horizontal displacement of truth and estimate across the window.
        truth_displacement = truth_position_m[end_index, :2] - truth_position_m[start_index, :2]
        estimate_displacement = (
            estimate_position_m[end_index, :2] - estimate_position_m[start_index, :2]
        )
        # A near-stationary truth window has no meaningful direction.
        if np.linalg.norm(truth_displacement) < minimum_truth_travel_m:
            continue
        # Signed angle from truth to estimate via atan2(cross, dot).
        cross = (
            truth_displacement[0] * estimate_displacement[1]
            - truth_displacement[1] * estimate_displacement[0]
        )
        dot = float(np.dot(truth_displacement, estimate_displacement))
        direction_error_deg[end_index] = np.degrees(np.arctan2(cross, dot))
    return direction_error_deg


def wheel_speed_mps(
    wheel_angular_velocity_radps: npt.NDArray[np.float64],
    wheel_radius_m: float,
    direction_multipliers: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """!
    @brief   Converts per-wheel drive-joint angular velocity to
             circumferential speed, using the snapshotted wheel radius and
             per-wheel direction multiplier (never confuse this with body
             velocity -- see the wheel odometry report's own comparison
             against ground-truth body twist for that).

    @param   wheel_angular_velocity_radps
             Per-wheel drive-joint angular velocity, rad/s, shape (N, 6).
    @param   wheel_radius_m
             The snapshotted wheel radius, metres (one value, shared by
             every wheel per wheel_odometry.yaml/ackermann_controller.yaml).
    @param   direction_multipliers
             Per-wheel direction multiplier, shape (6,), in WHEEL_ORDER.

    @return  Per-wheel circumferential speed, m/s, shape (N, 6).

    @throws  ValueError
             If `direction_multipliers` does not carry exactly one entry
             per wheel.
    """
    if direction_multipliers.shape != (WHEEL_COUNT,):
        raise ValueError(
            f"direction_multipliers must have shape ({WHEEL_COUNT},), got "
            f"{direction_multipliers.shape}"
        )
    return wheel_angular_velocity_radps * wheel_radius_m * direction_multipliers


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone error-summary self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Print RMSE/median/p95/maximum for a whitespace-separated "
            "list of error magnitudes."
        )
    )
    parser.add_argument(
        "magnitudes", type=float, nargs="+", help="Non-negative error magnitudes."
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the error summary for manually supplied
             magnitudes.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    summary = summarize_errors(np.array(args.magnitudes, dtype=np.float64))
    print(
        f"rmse={summary.rmse:.6f} median={summary.median:.6f} "
        f"p95={summary.p95:.6f} max={summary.maximum:.6f} "
        f"n={summary.sample_count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
