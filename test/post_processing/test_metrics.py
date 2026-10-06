"""Tests for data metrics.

Contents:
    SummarizeErrorsTests: error summary statistics.
    WrappedAngleDifferenceTests: angle wrapping.
    SampleHealthTests: rate and absence detection.
    CovarianceTests: std dev and residuals.
    WheelSpeedTests: wheel speed conversion.
    HighFrequencyResidualTests: jitter extraction.
    PublicationArtifactTests: staircase detection.
    SigmaCoverageTests: sigma coverage fractions.
    DirectionOfTravelErrorTests: heading error.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.data import metrics
from python_tools.data.models import WHEEL_COUNT


class SummarizeErrorsTests(unittest.TestCase):
    """Tests for summarize_errors."""

    def test_known_values(self) -> None:
        """RMSE and quantiles match a reference."""
        summary = metrics.summarize_errors(
            np.array([1.0, 2.0, 3.0, 4.0])
        )
        self.assertAlmostEqual(
            summary.rmse, np.sqrt((1 + 4 + 9 + 16) / 4)
        )
        self.assertAlmostEqual(summary.median, 2.5)
        self.assertAlmostEqual(summary.maximum, 4.0)
        self.assertEqual(summary.sample_count, 4)

    def test_ignores_nan_entries(self) -> None:
        """NaN entries are excluded from the summary."""
        summary = metrics.summarize_errors(
            np.array([2.0, np.nan, 2.0])
        )
        self.assertEqual(summary.sample_count, 2)
        self.assertAlmostEqual(summary.rmse, 2.0)

    def test_all_nan_produces_empty_summary(self) -> None:
        """All-NaN input gives an empty summary."""
        summary = metrics.summarize_errors(
            np.array([np.nan, np.nan])
        )
        self.assertEqual(summary.sample_count, 0)
        self.assertTrue(np.isnan(summary.rmse))


class WrappedAngleDifferenceTests(unittest.TestCase):
    """Tests for wrapped_angle_difference."""

    def test_wraps_across_the_boundary(self) -> None:
        """Angles past +/-pi wrap to a small value."""
        difference = metrics.wrapped_angle_difference(
            np.array([-np.pi + 0.1]), np.array([np.pi - 0.1])
        )
        self.assertAlmostEqual(float(difference[0]), 0.2, places=6)

    def test_no_wrap_needed(self) -> None:
        """Small differences pass through as-is."""
        difference = metrics.wrapped_angle_difference(
            np.array([0.5]), np.array([0.2])
        )
        self.assertAlmostEqual(float(difference[0]), 0.3, places=6)


class SampleHealthTests(unittest.TestCase):
    """Tests for sample rate and absence detection."""

    def test_sample_rate_hz(self) -> None:
        """Ten samples over one second report 9 Hz."""
        times_s = np.linspace(0.0, 1.0, 10)
        self.assertAlmostEqual(metrics.sample_rate_hz(times_s), 9.0)

    def test_sample_rate_none_for_single_sample(self) -> None:
        """Single sample cannot define a rate."""
        self.assertIsNone(metrics.sample_rate_hz(np.array([1.0])))

    def test_absence_intervals_detects_large_gap(self) -> None:
        """Large gaps report one absence interval."""
        times_s = np.array([0.0, 0.1, 5.0, 5.1])
        intervals = metrics.absence_intervals_s(
            times_s, maximum_gap_s=1.0
        )
        self.assertEqual(intervals, [(0.1, 5.0)])


class CovarianceTests(unittest.TestCase):
    """Tests for covariance helpers."""

    def test_std_dev_from_diagonal(self) -> None:
        """Std dev is the root of the diagonal."""
        covariance = np.zeros((1, 6, 6))
        covariance[0, 0, 0] = 4.0
        covariance[0, 1, 1] = 9.0
        std_dev = metrics.covariance_std_dev(covariance)
        np.testing.assert_allclose(std_dev[0, :2], [2.0, 3.0])

    def test_negative_diagonal_clips_to_zero(self) -> None:
        """Negative diagonal gives zero std dev."""
        covariance = np.zeros((1, 6, 6))
        covariance[0, 0, 0] = -1.0
        std_dev = metrics.covariance_std_dev(covariance)
        self.assertEqual(std_dev[0, 0], 0.0)

    def test_normalized_residual(self) -> None:
        """Error over std dev gives the residual."""
        residual = metrics.normalized_residual(
            np.array([2.0]), np.array([0.5])
        )
        self.assertAlmostEqual(float(residual[0]), 4.0)

    def test_normalized_residual_zero_std_dev_is_nan(self) -> None:
        """Zero std dev gives NaN, not infinity."""
        residual = metrics.normalized_residual(
            np.array([2.0]), np.array([0.0])
        )
        self.assertTrue(np.isnan(residual[0]))


class WheelSpeedTests(unittest.TestCase):
    """Tests for wheel_speed_mps."""

    def test_converts_angular_velocity_to_speed(self) -> None:
        """Velocity times radius gives wheel speed."""
        angular = np.full((2, WHEEL_COUNT), 2.0)
        direction = np.ones(WHEEL_COUNT)
        speed = metrics.wheel_speed_mps(
            angular,
            wheel_radius_m=0.5,
            direction_multipliers=direction,
        )
        np.testing.assert_allclose(
            speed, np.full((2, WHEEL_COUNT), 1.0)
        )

    def test_rejects_wrong_direction_shape(self) -> None:
        """Wrong direction length raises."""
        angular = np.zeros((1, WHEEL_COUNT))
        with self.assertRaises(ValueError):
            metrics.wheel_speed_mps(
                angular, 0.5, np.array([1.0, 1.0])
            )


class HighFrequencyResidualTests(unittest.TestCase):
    """Tests for high_frequency_residual."""

    def test_linear_ramp_has_zero_interior_residual(self) -> None:
        """Linear ramp has zero interior residual."""
        times_s = np.arange(0.0, 10.0, 0.1)
        # 2.05 s keeps window edges off the sample grid;
        # an edge on a sample would hinge on rounding.
        residual = metrics.high_frequency_residual(
            times_s, 0.02 * times_s, window_s=2.05
        )
        interior = residual[np.isfinite(residual)]
        self.assertGreater(interior.size, 50)
        np.testing.assert_allclose(interior, 0.0, atol=1e-12)

    def test_alternating_jitter_is_preserved(self) -> None:
        """Alternating jitter keeps its residual."""
        times_s = np.arange(0.0, 20.0, 0.02)
        jitter_m = np.where(
            np.arange(times_s.size) % 2 == 0, 0.001, -0.001
        )
        residual = metrics.high_frequency_residual(
            times_s, 5.0 + jitter_m
        )
        self.assertAlmostEqual(
            float(np.nanstd(residual)), 0.001, delta=2e-5
        )

    def test_edges_and_nan_samples_are_masked(self) -> None:
        """Edges and NaNs report NaN cleanly."""
        times_s = np.arange(0.0, 10.0, 0.1)
        values = np.column_stack(
            [np.zeros_like(times_s), np.ones_like(times_s)]
        )
        values[50, 0] = np.nan
        residual = metrics.high_frequency_residual(times_s, values)
        self.assertTrue(np.isnan(residual[0, 0]))
        self.assertTrue(np.isnan(residual[-1, 1]))
        self.assertTrue(np.isnan(residual[50, 0]))
        np.testing.assert_allclose(
            residual[30:70, 1], 0.0, atol=1e-12
        )
        self.assertAlmostEqual(float(residual[60, 0]), 0.0)

    def test_rejects_non_positive_window(self) -> None:
        """Zero-width window is a caller error."""
        with self.assertRaises(ValueError):
            metrics.high_frequency_residual(
                np.arange(3.0), np.zeros(3), window_s=0.0
            )


class PublicationArtifactTests(unittest.TestCase):
    """Tests for staircase publication detection."""

    def test_maximum_step_norm(self) -> None:
        """Largest consecutive jump is measured."""
        positions = np.array(
            [[0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.01, 0.17, 0.0]]
        )
        self.assertAlmostEqual(
            metrics.maximum_step_norm(positions), 0.17
        )
        self.assertTrue(
            np.isnan(metrics.maximum_step_norm(positions[:1]))
        )

    def test_duplicate_stamp_count(self) -> None:
        """Only repeated stamps are counted."""
        stamps = np.array(
            [10, 10, 20, 30, 30, 30, 40], dtype=np.int64
        )
        self.assertEqual(metrics.duplicate_stamp_count(stamps), 3)

    def test_identical_sample_count(self) -> None:
        """Only fully repeated rows are counted."""
        samples = np.array(
            [[1.0, 2.0], [1.0, 2.0], [1.0, 2.5], [1.0, 2.5]]
        )
        self.assertEqual(metrics.identical_sample_count(samples), 2)
        self.assertEqual(
            metrics.identical_sample_count(samples[:1]), 0
        )


class SigmaCoverageTests(unittest.TestCase):
    """Tests for sigma_coverage."""

    def test_known_coverage(self) -> None:
        """Known errors give expected fractions."""
        error = np.array([0.5, -2.0, 4.0, np.nan, 1.0])
        std_dev = np.array([1.0, 1.0, 1.0, 1.0, 0.0])
        coverage = metrics.sigma_coverage(error, std_dev)
        self.assertEqual(coverage.sample_count, 3)
        self.assertAlmostEqual(
            coverage.within_one_sigma, 1.0 / 3.0
        )
        self.assertAlmostEqual(
            coverage.within_three_sigma, 2.0 / 3.0
        )

    def test_gaussian_samples_match_expected_fractions(self) -> None:
        """Gaussian errors cover 68 and 99.7 percent."""
        generator = np.random.default_rng(7)
        error = generator.normal(0.0, 2.0, 200000)
        coverage = metrics.sigma_coverage(
            error, np.full(error.shape, 2.0)
        )
        self.assertAlmostEqual(
            coverage.within_one_sigma, 0.6827, delta=0.005
        )
        self.assertAlmostEqual(
            coverage.within_three_sigma, 0.9973, delta=0.001
        )

    def test_no_usable_samples(self) -> None:
        """No usable sample gives undefined coverage."""
        coverage = metrics.sigma_coverage(
            np.array([np.nan]), np.array([1.0])
        )
        self.assertEqual(coverage.sample_count, 0)
        self.assertTrue(np.isnan(coverage.within_one_sigma))


class DirectionOfTravelErrorTests(unittest.TestCase):
    """Tests for direction_of_travel_error_deg."""

    def test_rotated_estimate_reports_its_rotation(self) -> None:
        """10-degree rotation reports +10 degrees."""
        times_s = np.arange(0.0, 60.0, 0.5)
        truth = np.column_stack(
            [0.02 * times_s, np.zeros_like(times_s),
             np.zeros_like(times_s)]
        )
        angle_rad = np.radians(10.0)
        estimate = np.column_stack(
            [truth[:, 0] * np.cos(angle_rad),
             truth[:, 0] * np.sin(angle_rad), truth[:, 2]]
        )
        valid = np.ones(times_s.shape, dtype=bool)
        error_deg = metrics.direction_of_travel_error_deg(
            times_s, estimate, truth, valid
        )
        self.assertTrue(np.all(np.isnan(error_deg[times_s < 30.0])))
        np.testing.assert_allclose(
            error_deg[times_s >= 30.0], 10.0, atol=1e-9
        )

    def test_stationary_truth_is_undefined(self) -> None:
        """Stationary truth has no direction."""
        times_s = np.arange(0.0, 60.0, 1.0)
        truth = np.zeros((times_s.size, 3))
        estimate = truth + 0.001
        valid = np.ones(times_s.shape, dtype=bool)
        error_deg = metrics.direction_of_travel_error_deg(
            times_s, estimate, truth, valid
        )
        self.assertTrue(np.all(np.isnan(error_deg)))

    def test_rejects_non_positive_limits(self) -> None:
        """Non-positive limits are caller errors."""
        with self.assertRaises(ValueError):
            metrics.direction_of_travel_error_deg(
                np.arange(3.0),
                np.zeros((3, 3)),
                np.zeros((3, 3)),
                np.ones(3, dtype=bool),
                window_s=0.0,
            )


if __name__ == "__main__":
    unittest.main()
