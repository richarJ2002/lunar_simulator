"""Tests for bag timestamp helpers.

Contents:
    HeaderStampToNsTests: stamp combination.
    TimestampValidityTests: validity filtering.
    ElapsedSecondsTests: elapsed conversion.
    SegmentSeriesTests: discontinuity segmentation.

Each test module makes python_tools importable itself,
like every post_processing entry point does.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.bag.timestamps import (
    elapsed_seconds,
    filter_finite_timestamps,
    header_stamp_to_ns,
    is_valid_timestamp_ns,
    segment_series,
)
from python_tools.data.models import ResetReason


class HeaderStampToNsTests(unittest.TestCase):
    """Tests for header_stamp_to_ns."""

    def test_combines_seconds_and_nanoseconds(self) -> None:
        """Sec and nanosec combine correctly."""
        self.assertEqual(
            header_stamp_to_ns(5, 250_000_000), 5_250_000_000
        )

    def test_zero_stamp_is_zero(self) -> None:
        """Unset stamp combines to zero."""
        self.assertEqual(header_stamp_to_ns(0, 0), 0)


class TimestampValidityTests(unittest.TestCase):
    """Tests for timestamp validity filtering."""

    def test_positive_is_valid(self) -> None:
        """Positive timestamp is valid."""
        self.assertTrue(is_valid_timestamp_ns(1))

    def test_zero_is_invalid(self) -> None:
        """Zero stamp is invalid."""
        self.assertFalse(is_valid_timestamp_ns(0))

    def test_negative_is_invalid(self) -> None:
        """Negative timestamp is invalid."""
        self.assertFalse(is_valid_timestamp_ns(-1))

    def test_filter_finite_timestamps_masks_vector(self) -> None:
        """Vector form masks element-wise."""
        times_ns = np.array([0, 5, -3, 10], dtype=np.int64)
        mask = filter_finite_timestamps(times_ns)
        np.testing.assert_array_equal(
            mask, [False, True, False, True]
        )


class ElapsedSecondsTests(unittest.TestCase):
    """Tests for elapsed_seconds."""

    def test_relative_to_start(self) -> None:
        """Elapsed time is relative to start."""
        times_ns = np.array(
            [1_000_000_000, 1_500_000_000, 2_000_000_000],
            dtype=np.int64,
        )
        result = elapsed_seconds(times_ns, 1_000_000_000)
        np.testing.assert_allclose(result, [0.0, 0.5, 1.0])


class SegmentSeriesTests(unittest.TestCase):
    """Tests for segment_series."""

    def test_empty_series_has_no_segments(self) -> None:
        """Empty array produces no segments."""
        self.assertEqual(
            segment_series(np.array([], dtype=np.int64), 1.0), ()
        )

    def test_continuous_series_is_one_segment(self) -> None:
        """Even spacing forms one segment."""
        times_ns = np.array([0, 1_000_000, 2_000_000], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0].start_index, 0)
        self.assertEqual(segments[0].end_index, 3)
        self.assertIsNone(segments[0].reason)

    def test_backward_jump_splits_segment(self) -> None:
        """Earlier timestamp starts a new segment."""
        times_ns = np.array([10, 20, 5, 15], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0].start_index, 0)
        self.assertEqual(segments[0].end_index, 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(
            segments[1].reason, ResetReason.BACKWARD_TIME_JUMP
        )

    def test_maximum_gap_exceeded_splits_segment(self) -> None:
        """Large forward gap starts a new segment."""
        one_second_ns = 1_000_000_000
        times_ns = np.array(
            [0, one_second_ns, 5 * one_second_ns], dtype=np.int64
        )
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(
            segments[1].reason, ResetReason.MAXIMUM_GAP_EXCEEDED
        )

    def test_reset_event_splits_segment(self) -> None:
        """Reset between samples splits the series."""
        one_second_ns = 1_000_000_000
        times_ns = np.array(
            [0, one_second_ns, 2 * one_second_ns], dtype=np.int64
        )
        reset_times_ns = np.array(
            [int(1.5 * one_second_ns)], dtype=np.int64
        )
        segments = segment_series(
            times_ns,
            maximum_gap_s=10.0,
            reset_event_times_ns=reset_times_ns,
        )
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(
            segments[1].reason, ResetReason.VISUAL_RESET_EVENT
        )

    def test_segments_cover_every_index_exactly_once(self) -> None:
        """Segments partition every index once."""
        times_ns = np.array(
            [0, 1, 2, 1, 2, 100_000_000_000, 100_000_000_001],
            dtype=np.int64,
        )
        segments = segment_series(times_ns, maximum_gap_s=0.001)
        covered = []
        for segment in segments:
            covered.extend(
                range(segment.start_index, segment.end_index)
            )
        self.assertEqual(covered, list(range(times_ns.size)))


if __name__ == "__main__":
    unittest.main()
