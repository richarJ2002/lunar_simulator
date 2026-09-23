"""!
@brief  Tests for python_tools.data.alignment: quaternion geodesic
        distance, slerp, bracketed ground-truth interpolation (no
        extrapolation), Euler conversion, and per-segment unwrapping.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.data import alignment
from python_tools.data.models import OdometrySeries, ResetReason, SeriesSegment


def _make_truth_series(times_s: np.ndarray, positions: np.ndarray) -> OdometrySeries:
    """!
    @brief   Builds a minimal `OdometrySeries` for interpolation tests,
             with identity orientation and zero twist/covariance.

    @param   times_s
             Sample times, shape (N,).
    @param   positions
             Sample positions, shape (N, 3).

    @return  The assembled series.
    """
    n = times_s.shape[0]
    return OdometrySeries(
        times_s=times_s,
        times_ns=(times_s * 1e9).astype(np.int64),
        frame_id="map",
        child_frame_id="base_link",
        position_m=positions,
        orientation_xyzw=np.tile([0.0, 0.0, 0.0, 1.0], (n, 1)),
        linear_velocity_mps=np.zeros((n, 3)),
        angular_velocity_radps=np.zeros((n, 3)),
        pose_covariance=np.zeros((n, 6, 6)),
        twist_covariance=np.zeros((n, 6, 6)),
        segments=(SeriesSegment(0, n, None),),
    )


class QuaternionGeodesicAngleTests(unittest.TestCase):
    """!
    @brief  Tests for `quaternion_geodesic_angle`.
    """

    def test_identical_quaternions_have_zero_angle(self) -> None:
        """!
        @brief  Two identical unit quaternions are zero degrees apart.
        """
        q = np.array([[0.0, 0.0, 0.0, 1.0]])
        angle = alignment.quaternion_geodesic_angle(q, q)
        np.testing.assert_allclose(angle, [0.0], atol=1e-9)

    def test_ninety_degree_rotation(self) -> None:
        """!
        @brief  A 90-degree yaw quaternion is exactly pi/2 from identity.
        """
        identity = np.array([[0.0, 0.0, 0.0, 1.0]])
        ninety_z = np.array([[0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]])
        angle = alignment.quaternion_geodesic_angle(identity, ninety_z)
        np.testing.assert_allclose(angle, [np.pi / 2], atol=1e-9)

    def test_sign_equivalence(self) -> None:
        """!
        @brief  q and -q represent the same rotation, so the geodesic
                angle between q and -q is zero, not pi.
        """
        q = np.array([[0.1, 0.2, 0.3, 0.9]])
        q = q / np.linalg.norm(q, axis=-1, keepdims=True)
        angle = alignment.quaternion_geodesic_angle(q, -q)
        np.testing.assert_allclose(angle, [0.0], atol=1e-9)

    def test_near_180_degrees(self) -> None:
        """!
        @brief  A near-antipodal rotation reports an angle near pi without
                numerical breakdown from the arccos clip.
        """
        identity = np.array([[0.0, 0.0, 0.0, 1.0]])
        near_180_z = np.array([[0.0, 0.0, np.sin(np.pi / 2 - 1e-8), np.cos(np.pi / 2 - 1e-8)]])
        angle = alignment.quaternion_geodesic_angle(identity, near_180_z)
        self.assertAlmostEqual(float(angle[0]), np.pi, places=5)


class SlerpTests(unittest.TestCase):
    """!
    @brief  Tests for `slerp`.
    """

    def test_midpoint_is_half_angle(self) -> None:
        """!
        @brief  Slerping halfway between identity and a 90-degree yaw
                produces a 45-degree yaw.
        """
        identity = np.array([[0.0, 0.0, 0.0, 1.0]])
        ninety_z = np.array([[0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]])
        midpoint = alignment.slerp(identity, ninety_z, np.array([0.5]))
        angle_to_identity = alignment.quaternion_geodesic_angle(identity, midpoint)
        np.testing.assert_allclose(angle_to_identity, [np.pi / 4], atol=1e-6)

    def test_endpoints_are_exact(self) -> None:
        """!
        @brief  Fraction 0 and 1 return the start/end quaternions exactly.
        """
        start = np.array([[0.0, 0.0, 0.0, 1.0]])
        end = np.array([[0.0, 0.0, 1.0, 0.0]])
        np.testing.assert_allclose(alignment.slerp(start, end, np.array([0.0])), start, atol=1e-9)
        np.testing.assert_allclose(alignment.slerp(start, end, np.array([1.0])), end, atol=1e-9)


class InterpolateGroundTruthTests(unittest.TestCase):
    """!
    @brief  Tests for `interpolate_ground_truth`'s bracketing and
            no-extrapolation contract.
    """

    def test_linear_interpolation_between_samples(self) -> None:
        """!
        @brief  A query exactly halfway between two truth samples
                interpolates position linearly.
        """
        truth = _make_truth_series(
            np.array([0.0, 2.0]), np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        )
        position, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([1.0]), truth, maximum_gap_s=10.0
        )
        np.testing.assert_allclose(position, [[1.0, 0.0, 0.0]])
        self.assertTrue(valid[0])

    def test_query_before_coverage_is_invalid(self) -> None:
        """!
        @brief  A query time before the truth series' first sample is
                marked invalid, never extrapolated.
        """
        truth = _make_truth_series(
            np.array([5.0, 6.0]), np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        )
        _, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([0.0]), truth, maximum_gap_s=10.0
        )
        self.assertFalse(valid[0])

    def test_query_after_coverage_is_invalid(self) -> None:
        """!
        @brief  A query time after the truth series' last sample is
                marked invalid, never extrapolated.
        """
        truth = _make_truth_series(
            np.array([0.0, 1.0]), np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        )
        _, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([5.0]), truth, maximum_gap_s=10.0
        )
        self.assertFalse(valid[0])

    def test_wide_bracket_exceeding_maximum_gap_is_invalid(self) -> None:
        """!
        @brief  A query bracketed by truth samples farther apart than
                `maximum_gap_s` is marked invalid, even though it is
                technically within coverage.
        """
        truth = _make_truth_series(
            np.array([0.0, 100.0]), np.array([[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]])
        )
        _, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([50.0]), truth, maximum_gap_s=1.0
        )
        self.assertFalse(valid[0])

    def test_query_exactly_at_first_sample_is_valid(self) -> None:
        """!
        @brief  Regression for Defect 6 in the plan's 2026-09-22 audit: a
                query time exactly equal to the truth series' first
                timestamp used to be excluded from coverage (a
                `searchsorted(..., side="left")` off-by-one at the lower
                boundary only). It must interpolate to that exact sample,
                not be treated as extrapolation.
        """
        truth = _make_truth_series(
            np.array([5.0, 6.0, 7.0]),
            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
        )
        position, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([5.0]), truth, maximum_gap_s=10.0
        )
        self.assertTrue(valid[0])
        np.testing.assert_allclose(position, [[0.0, 0.0, 0.0]])

    def test_query_exactly_at_last_sample_is_valid(self) -> None:
        """!
        @brief  A query time exactly equal to the truth series' last
                timestamp interpolates to that exact sample (this
                direction was already correct before Defect 6's fix;
                pinned here so a future change cannot regress it while
                fixing the other boundary).
        """
        truth = _make_truth_series(
            np.array([5.0, 6.0, 7.0]),
            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
        )
        position, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([7.0]), truth, maximum_gap_s=10.0
        )
        self.assertTrue(valid[0])
        np.testing.assert_allclose(position, [[2.0, 0.0, 0.0]])

    def test_values_just_before_and_after_coverage_are_invalid(self) -> None:
        """!
        @brief  Queries an infinitesimal amount before the first sample or
                after the last sample are still excluded -- the Defect 6
                fix admits the exact boundary, not a widened range past
                it.
        """
        truth = _make_truth_series(
            np.array([5.0, 6.0, 7.0]),
            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
        )
        _, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([5.0 - 1e-9, 7.0 + 1e-9]), truth, maximum_gap_s=10.0
        )
        self.assertFalse(valid[0])
        self.assertFalse(valid[1])

    def test_single_sample_reference_exact_match_is_valid(self) -> None:
        """!
        @brief  A truth series with only one sample still resolves an
                exact-match query to that sample (the degenerate case
                where the first and last sample are the same one).
        """
        truth = _make_truth_series(np.array([3.0]), np.array([[9.0, 8.0, 7.0]]))
        position, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([3.0]), truth, maximum_gap_s=10.0
        )
        self.assertTrue(valid[0])
        np.testing.assert_allclose(position, [[9.0, 8.0, 7.0]])

    def test_single_sample_reference_non_matching_query_is_invalid(self) -> None:
        """!
        @brief  A truth series with only one sample rejects any query that
                does not exactly match it -- never extrapolated.
        """
        truth = _make_truth_series(np.array([3.0]), np.array([[9.0, 8.0, 7.0]]))
        _, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([3.1]), truth, maximum_gap_s=10.0
        )
        self.assertFalse(valid[0])

    def test_bracketing_resolves_correctly_around_an_internal_gap(self) -> None:
        """!
        @brief  A truth series spanning multiple discontinuous segments
                (a large internal gap) still resolves each query to the
                correct *local* bracket rather than bridging across the
                gap, and each segment's own boundary samples remain valid.
        """
        truth = _make_truth_series(
            np.array([0.0, 1.0, 2.0, 100.0, 101.0, 102.0]),
            np.array(
                [
                    [0.0, 0, 0], [1.0, 0, 0], [2.0, 0, 0],
                    [100.0, 0, 0], [101.0, 0, 0], [102.0, 0, 0],
                ]
            ),
        )
        position, _, _, _, valid = alignment.interpolate_ground_truth(
            np.array([0.0, 1.5, 2.0, 100.0, 100.5, 102.0]), truth, maximum_gap_s=10.0
        )
        # Every query lands within its own segment's tight local bracket.
        np.testing.assert_allclose(
            position,
            [[0.0, 0, 0], [1.5, 0, 0], [2.0, 0, 0], [100.0, 0, 0], [100.5, 0, 0], [102.0, 0, 0]],
        )
        self.assertTrue(np.all(valid))
        # A query inside the gap itself is rejected: its nearest bracket
        # (2.0, 100.0) is far wider than maximum_gap_s.
        _, _, _, _, gap_valid = alignment.interpolate_ground_truth(
            np.array([50.0]), truth, maximum_gap_s=10.0
        )
        self.assertFalse(gap_valid[0])


class EulerConversionTests(unittest.TestCase):
    """!
    @brief  Tests for `quaternion_to_euler_zyx`.
    """

    def test_identity_is_zero(self) -> None:
        """!
        @brief  The identity quaternion has zero roll/pitch/yaw.
        """
        q = np.array([[0.0, 0.0, 0.0, 1.0]])
        roll, pitch, yaw = alignment.quaternion_to_euler_zyx(q)
        np.testing.assert_allclose([roll[0], pitch[0], yaw[0]], [0.0, 0.0, 0.0], atol=1e-9)

    def test_ninety_degree_yaw(self) -> None:
        """!
        @brief  A 90-degree yaw quaternion reports yaw=pi/2, roll=pitch=0.
        """
        q = np.array([[0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]])
        roll, pitch, yaw = alignment.quaternion_to_euler_zyx(q)
        np.testing.assert_allclose([roll[0], pitch[0], yaw[0]], [0.0, 0.0, np.pi / 2], atol=1e-6)


class UnwrapPerSegmentTests(unittest.TestCase):
    """!
    @brief  Tests for `unwrap_per_segment`.
    """

    def test_unwraps_within_a_segment(self) -> None:
        """!
        @brief  A wrap-around within one segment is unwrapped smoothly.
        """
        angles = np.array([3.0, -3.1, -3.0])  # crosses the +/-pi boundary
        segments = (SeriesSegment(0, 3, None),)
        unwrapped = alignment.unwrap_per_segment(angles, segments)
        np.testing.assert_allclose(np.diff(unwrapped), np.diff(np.unwrap(angles)))

    def test_does_not_unwrap_across_a_segment_boundary(self) -> None:
        """!
        @brief  A jump across a discontinuity segment boundary is left
                as-is, not smoothed by unwrap() reaching across it.
        """
        angles = np.array([0.0, 3.0, -3.0, 0.0])
        segments = (
            SeriesSegment(0, 2, None),
            SeriesSegment(2, 4, ResetReason.MAXIMUM_GAP_EXCEEDED),
        )
        unwrapped = alignment.unwrap_per_segment(angles, segments)
        # The second segment is unwrapped independently, starting fresh
        # from its own first value (-3.0), not continuing from the first
        # segment's trajectory.
        self.assertAlmostEqual(unwrapped[2], -3.0)


if __name__ == "__main__":
    unittest.main()
