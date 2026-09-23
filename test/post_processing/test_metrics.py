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


if __name__ == "__main__":
    unittest.main()
