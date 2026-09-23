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
