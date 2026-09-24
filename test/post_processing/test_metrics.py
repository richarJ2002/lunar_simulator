"""!
@brief  Tests for python_tools.data.metrics: error summaries with known
        synthetic values, wrapped angle differences, sample-rate/gap
        health, covariance-derived standard deviation and normalized
        residual, and six-wheel speed conversion.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.data import metrics
from python_tools.data.models import WHEEL_COUNT


class SummarizeErrorsTests(unittest.TestCase):
    """!
    @brief  Tests for `summarize_errors` against a known synthetic sample.
    """

    def test_known_values(self) -> None:
        """!
        @brief  RMSE/median/p95/max match a hand-computed reference for a
                fixed [1, 2, 3, 4] sample.
        """
        summary = metrics.summarize_errors(np.array([1.0, 2.0, 3.0, 4.0]))
        self.assertAlmostEqual(summary.rmse, np.sqrt((1 + 4 + 9 + 16) / 4))
        self.assertAlmostEqual(summary.median, 2.5)
        self.assertAlmostEqual(summary.maximum, 4.0)
        self.assertEqual(summary.sample_count, 4)

    def test_ignores_nan_entries(self) -> None:
        """!
        @brief  NaN entries (unaligned samples) are excluded from the
                summary rather than propagating NaN into every statistic.
        """
        summary = metrics.summarize_errors(np.array([2.0, np.nan, 2.0]))
        self.assertEqual(summary.sample_count, 2)
        self.assertAlmostEqual(summary.rmse, 2.0)

    def test_all_nan_produces_empty_summary(self) -> None:
        """!
        @brief  A series with no valid samples produces a `sample_count`
                of 0 and NaN statistics, not a crash.
        """
        summary = metrics.summarize_errors(np.array([np.nan, np.nan]))
        self.assertEqual(summary.sample_count, 0)
        self.assertTrue(np.isnan(summary.rmse))


class WrappedAngleDifferenceTests(unittest.TestCase):
    """!
    @brief  Tests for `wrapped_angle_difference`.
    """

    def test_wraps_across_the_boundary(self) -> None:
        """!
        @brief  The difference between angles just past +/-pi wraps to a
                small value instead of a near-2pi one.
        """
        difference = metrics.wrapped_angle_difference(
            np.array([-np.pi + 0.1]), np.array([np.pi - 0.1])
        )
        self.assertAlmostEqual(float(difference[0]), 0.2, places=6)

    def test_no_wrap_needed(self) -> None:
        """!
        @brief  A difference well within [-pi, pi] passes through as-is.
        """
        difference = metrics.wrapped_angle_difference(np.array([0.5]), np.array([0.2]))
        self.assertAlmostEqual(float(difference[0]), 0.3, places=6)


class SampleHealthTests(unittest.TestCase):
    """!
    @brief  Tests for `sample_rate_hz`/`inter_sample_gaps_s`/
            `absence_intervals_s`.
    """

    def test_sample_rate_hz(self) -> None:
        """!
        @brief  10 samples over 1 second report 9 Hz (N-1 intervals).
        """
        times_s = np.linspace(0.0, 1.0, 10)
        self.assertAlmostEqual(metrics.sample_rate_hz(times_s), 9.0)

    def test_sample_rate_none_for_single_sample(self) -> None:
        """!
        @brief  Fewer than two samples cannot define a rate.
        """
        self.assertIsNone(metrics.sample_rate_hz(np.array([1.0])))

    def test_absence_intervals_detects_large_gap(self) -> None:
        """!
        @brief  A gap larger than the maximum is reported as one absence
                interval; smaller gaps are not.
        """
        times_s = np.array([0.0, 0.1, 5.0, 5.1])
        intervals = metrics.absence_intervals_s(times_s, maximum_gap_s=1.0)
        self.assertEqual(intervals, [(0.1, 5.0)])


class CovarianceTests(unittest.TestCase):
    """!
    @brief  Tests for `covariance_std_dev`/`normalized_residual`.
    """

    def test_std_dev_from_diagonal(self) -> None:
        """!
        @brief  Standard deviation is the square root of each diagonal
                entry.
        """
        covariance = np.zeros((1, 6, 6))
        covariance[0, 0, 0] = 4.0
        covariance[0, 1, 1] = 9.0
        std_dev = metrics.covariance_std_dev(covariance)
        np.testing.assert_allclose(std_dev[0, :2], [2.0, 3.0])

    def test_negative_diagonal_clips_to_zero(self) -> None:
        """!
        @brief  A numerically invalid negative diagonal entry produces a
                zero standard deviation, not a NaN from sqrt of a negative
                number.
        """
        covariance = np.zeros((1, 6, 6))
        covariance[0, 0, 0] = -1.0
        std_dev = metrics.covariance_std_dev(covariance)
        self.assertEqual(std_dev[0, 0], 0.0)

    def test_normalized_residual(self) -> None:
        """!
        @brief  Error divided by its standard deviation gives the expected
                normalized residual.
        """
        residual = metrics.normalized_residual(np.array([2.0]), np.array([0.5]))
        self.assertAlmostEqual(float(residual[0]), 4.0)

    def test_normalized_residual_zero_std_dev_is_nan(self) -> None:
        """!
        @brief  A zero standard deviation (unpopulated covariance) yields
                NaN, not an infinite division-by-zero result.
        """
        residual = metrics.normalized_residual(np.array([2.0]), np.array([0.0]))
        self.assertTrue(np.isnan(residual[0]))


class WheelSpeedTests(unittest.TestCase):
    """!
    @brief  Tests for `wheel_speed_mps`.
    """

    def test_converts_angular_velocity_to_speed(self) -> None:
        """!
        @brief  velocity * radius * direction gives the expected speed for
                a uniform wheel array.
        """
        angular = np.full((2, WHEEL_COUNT), 2.0)
        direction = np.ones(WHEEL_COUNT)
        speed = metrics.wheel_speed_mps(angular, wheel_radius_m=0.5, direction_multipliers=direction)
        np.testing.assert_allclose(speed, np.full((2, WHEEL_COUNT), 1.0))

    def test_rejects_wrong_direction_shape(self) -> None:
        """!
        @brief  A direction-multiplier array of the wrong length raises.
        """
        angular = np.zeros((1, WHEEL_COUNT))
        with self.assertRaises(ValueError):
            metrics.wheel_speed_mps(angular, 0.5, np.array([1.0, 1.0]))


class HighFrequencyResidualTests(unittest.TestCase):
    """!
    @brief  Tests for `high_frequency_residual`: a centred moving mean
            removes slow motion and keeps fast jitter.
    """

    def test_linear_ramp_has_zero_interior_residual(self) -> None:
        """!
        @brief  A centred mean of a linear ramp equals the ramp itself, so
                steady motion contributes no high-frequency residual.
        """
        times_s = np.arange(0.0, 10.0, 0.1)
        # 2.05 s keeps both window edges off the 0.1 s sample grid; an edge
        # landing exactly on a sample would be included or excluded by
        # floating-point rounding alone, making the window asymmetric.
        residual = metrics.high_frequency_residual(times_s, 0.02 * times_s, window_s=2.05)
        interior = residual[np.isfinite(residual)]
        self.assertGreater(interior.size, 50)
        np.testing.assert_allclose(interior, 0.0, atol=1e-12)

    def test_alternating_jitter_is_preserved(self) -> None:
        """!
        @brief  A +/-1 mm alternation on a constant keeps a residual of
                about 1 mm, because the window mean of the alternation is
                about zero.
        """
        times_s = np.arange(0.0, 20.0, 0.02)
        jitter_m = np.where(np.arange(times_s.size) % 2 == 0, 0.001, -0.001)
        residual = metrics.high_frequency_residual(times_s, 5.0 + jitter_m)
        self.assertAlmostEqual(float(np.nanstd(residual)), 0.001, delta=2e-5)

    def test_edges_and_nan_samples_are_masked(self) -> None:
        """!
        @brief  Samples whose window leaves the data, and non-finite input
                samples, report NaN without poisoning their neighbours.
        """
        times_s = np.arange(0.0, 10.0, 0.1)
        values = np.column_stack([np.zeros_like(times_s), np.ones_like(times_s)])
        values[50, 0] = np.nan
        residual = metrics.high_frequency_residual(times_s, values)
        self.assertTrue(np.isnan(residual[0, 0]))
        self.assertTrue(np.isnan(residual[-1, 1]))
        self.assertTrue(np.isnan(residual[50, 0]))
        np.testing.assert_allclose(residual[30:70, 1], 0.0, atol=1e-12)
        self.assertAlmostEqual(float(residual[60, 0]), 0.0)

    def test_rejects_non_positive_window(self) -> None:
        """!
        @brief  A zero-width window is a caller error.
        """
        with self.assertRaises(ValueError):
            metrics.high_frequency_residual(np.arange(3.0), np.zeros(3), window_s=0.0)


class PublicationArtifactTests(unittest.TestCase):
    """!
    @brief  Tests for the step, duplicate-stamp and repeated-sample counts
            used to detect a staircase-published estimate.
    """

    def test_maximum_step_norm(self) -> None:
        """!
        @brief  The largest consecutive jump is found and measured in 3-D.
        """
        positions = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.01, 0.17, 0.0]])
        self.assertAlmostEqual(metrics.maximum_step_norm(positions), 0.17)
        self.assertTrue(np.isnan(metrics.maximum_step_norm(positions[:1])))

    def test_duplicate_stamp_count(self) -> None:
        """!
        @brief  Only exactly repeated consecutive stamps are counted.
        """
        stamps = np.array([10, 10, 20, 30, 30, 30, 40], dtype=np.int64)
        self.assertEqual(metrics.duplicate_stamp_count(stamps), 3)

    def test_identical_sample_count(self) -> None:
        """!
        @brief  A row counts only if every column repeats the previous row.
        """
        samples = np.array([[1.0, 2.0], [1.0, 2.0], [1.0, 2.5], [1.0, 2.5]])
        self.assertEqual(metrics.identical_sample_count(samples), 2)
        self.assertEqual(metrics.identical_sample_count(samples[:1]), 0)


class SigmaCoverageTests(unittest.TestCase):
    """!
    @brief  Tests for `sigma_coverage`.
    """

    def test_known_coverage(self) -> None:
        """!
        @brief  Errors of 0.5, 2 and 4 sigma give 1/3 inside one sigma and
                2/3 inside three sigma; unusable samples are skipped.
        """
        error = np.array([0.5, -2.0, 4.0, np.nan, 1.0])
        std_dev = np.array([1.0, 1.0, 1.0, 1.0, 0.0])
        coverage = metrics.sigma_coverage(error, std_dev)
        self.assertEqual(coverage.sample_count, 3)
        self.assertAlmostEqual(coverage.within_one_sigma, 1.0 / 3.0)
        self.assertAlmostEqual(coverage.within_three_sigma, 2.0 / 3.0)

    def test_gaussian_samples_match_expected_fractions(self) -> None:
        """!
        @brief  Consistent Gaussian errors cover about 68.3 % and 99.7 %.
        """
        generator = np.random.default_rng(7)
        error = generator.normal(0.0, 2.0, 200000)
        coverage = metrics.sigma_coverage(error, np.full(error.shape, 2.0))
        self.assertAlmostEqual(coverage.within_one_sigma, 0.6827, delta=0.005)
        self.assertAlmostEqual(coverage.within_three_sigma, 0.9973, delta=0.001)

    def test_no_usable_samples(self) -> None:
        """!
        @brief  With no usable sample the coverage is undefined, not zero.
        """
        coverage = metrics.sigma_coverage(np.array([np.nan]), np.array([1.0]))
        self.assertEqual(coverage.sample_count, 0)
        self.assertTrue(np.isnan(coverage.within_one_sigma))


class DirectionOfTravelErrorTests(unittest.TestCase):
    """!
    @brief  Tests for `direction_of_travel_error_deg`.
    """

    def test_rotated_estimate_reports_its_rotation(self) -> None:
        """!
        @brief  An estimate travelling 10 degrees left of truth reports a
                steady +10 degree error once a full window has elapsed.
        """
        times_s = np.arange(0.0, 60.0, 0.5)
        truth = np.column_stack([0.02 * times_s, np.zeros_like(times_s), np.zeros_like(times_s)])
        angle_rad = np.radians(10.0)
        estimate = np.column_stack(
            [truth[:, 0] * np.cos(angle_rad), truth[:, 0] * np.sin(angle_rad), truth[:, 2]]
        )
        valid = np.ones(times_s.shape, dtype=bool)
        error_deg = metrics.direction_of_travel_error_deg(times_s, estimate, truth, valid)
        self.assertTrue(np.all(np.isnan(error_deg[times_s < 30.0])))
        np.testing.assert_allclose(error_deg[times_s >= 30.0], 10.0, atol=1e-9)

    def test_stationary_truth_is_undefined(self) -> None:
        """!
        @brief  A stationary rover has no direction of travel to compare.
        """
        times_s = np.arange(0.0, 60.0, 1.0)
        truth = np.zeros((times_s.size, 3))
        estimate = truth + 0.001
        valid = np.ones(times_s.shape, dtype=bool)
        error_deg = metrics.direction_of_travel_error_deg(times_s, estimate, truth, valid)
        self.assertTrue(np.all(np.isnan(error_deg)))

    def test_rejects_non_positive_limits(self) -> None:
        """!
        @brief  Non-positive window or travel limits are caller errors.
        """
        with self.assertRaises(ValueError):
            metrics.direction_of_travel_error_deg(
                np.arange(3.0), np.zeros((3, 3)), np.zeros((3, 3)), np.ones(3, dtype=bool), window_s=0.0
            )


if __name__ == "__main__":
    unittest.main()
