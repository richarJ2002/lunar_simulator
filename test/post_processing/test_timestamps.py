"""!
@brief  Tests for python_tools.bag.timestamps: header-stamp combination,
        elapsed-seconds conversion, discontinuity-free segmentation, and
        timestamp validity filtering.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

# `unittest discover -s test/post_processing` (this project's documented
# verification command) imports test modules as bare top-level names, so
# test/post_processing/__init__.py's own sys.path adjustment never runs;
# each test module makes python_tools importable itself, exactly like every
# post_processing_*.py entry point already does for itself.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.bag.timestamps import (
    elapsed_seconds,
    filter_finite_timestamps,
    header_stamp_to_ns,
    is_valid_timestamp_ns,
    segment_series,
)
from python_tools.data.models import ResetReason


class HeaderStampToNsTests(unittest.TestCase):
    """!
    @brief  Tests for `header_stamp_to_ns`.
    """

    def test_combines_seconds_and_nanoseconds(self) -> None:
        """!
        @brief  A (sec, nanosec) pair combines to the expected nanosecond
                count.
        """
        self.assertEqual(header_stamp_to_ns(5, 250_000_000), 5_250_000_000)

    def test_zero_stamp_is_zero(self) -> None:
        """!
        @brief  An unset (sec=0, nanosec=0) stamp combines to exactly 0.
        """
        self.assertEqual(header_stamp_to_ns(0, 0), 0)


class TimestampValidityTests(unittest.TestCase):
    """!
    @brief  Tests for `is_valid_timestamp_ns`/`filter_finite_timestamps`.
    """

    def test_positive_is_valid(self) -> None:
        """!
        @brief  A positive timestamp is valid.
        """
        self.assertTrue(is_valid_timestamp_ns(1))

    def test_zero_is_invalid(self) -> None:
        """!
        @brief  Exactly zero (the ROS "unset stamp" convention) is invalid.
        """
        self.assertFalse(is_valid_timestamp_ns(0))

    def test_negative_is_invalid(self) -> None:
        """!
        @brief  A negative timestamp is invalid.
        """
        self.assertFalse(is_valid_timestamp_ns(-1))

    def test_filter_finite_timestamps_masks_vector(self) -> None:
        """!
        @brief  The vectorized form masks the same way, element-wise.
        """
        times_ns = np.array([0, 5, -3, 10], dtype=np.int64)
        mask = filter_finite_timestamps(times_ns)
        np.testing.assert_array_equal(mask, [False, True, False, True])


class ElapsedSecondsTests(unittest.TestCase):
    """!
    @brief  Tests for `elapsed_seconds`.
    """

    def test_relative_to_start(self) -> None:
        """!
        @brief  Elapsed seconds are measured relative to the given start.
        """
        times_ns = np.array([1_000_000_000, 1_500_000_000, 2_000_000_000], dtype=np.int64)
        result = elapsed_seconds(times_ns, 1_000_000_000)
        np.testing.assert_allclose(result, [0.0, 0.5, 1.0])


class SegmentSeriesTests(unittest.TestCase):
    """!
    @brief  Tests for `segment_series`.
    """

    def test_empty_series_has_no_segments(self) -> None:
        """!
        @brief  An empty timestamp array produces no segments.
        """
        self.assertEqual(segment_series(np.array([], dtype=np.int64), 1.0), ())

    def test_continuous_series_is_one_segment(self) -> None:
        """!
        @brief  Evenly spaced timestamps within the gap bound form one
                segment with no reason recorded.
        """
        times_ns = np.array([0, 1_000_000, 2_000_000], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0].start_index, 0)
        self.assertEqual(segments[0].end_index, 3)
        self.assertIsNone(segments[0].reason)

    def test_backward_jump_splits_segment(self) -> None:
        """!
        @brief  A timestamp earlier than its predecessor starts a new
                segment tagged BACKWARD_TIME_JUMP.
        """
        times_ns = np.array([10, 20, 5, 15], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0].start_index, 0)
        self.assertEqual(segments[0].end_index, 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(segments[1].reason, ResetReason.BACKWARD_TIME_JUMP)

    def test_maximum_gap_exceeded_splits_segment(self) -> None:
        """!
        @brief  A forward gap larger than the maximum starts a new segment
                tagged MAXIMUM_GAP_EXCEEDED.
        """
        one_second_ns = 1_000_000_000
        times_ns = np.array([0, one_second_ns, 5 * one_second_ns], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=1.0)
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(segments[1].reason, ResetReason.MAXIMUM_GAP_EXCEEDED)

    def test_reset_event_splits_segment(self) -> None:
        """!
        @brief  A reset event strictly between two samples splits the
                series there, tagged VISUAL_RESET_EVENT, even though
                neither the gap nor a backward jump would have.
        """
        one_second_ns = 1_000_000_000
        times_ns = np.array([0, one_second_ns, 2 * one_second_ns], dtype=np.int64)
        reset_times_ns = np.array([int(1.5 * one_second_ns)], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=10.0, reset_event_times_ns=reset_times_ns)
        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[1].start_index, 2)
        self.assertEqual(segments[1].reason, ResetReason.VISUAL_RESET_EVENT)

    def test_segments_cover_every_index_exactly_once(self) -> None:
        """!
        @brief  For an arbitrary series with multiple discontinuities,
                the segments partition every index exactly once.
        """
        times_ns = np.array([0, 1, 2, 1, 2, 100_000_000_000, 100_000_000_001], dtype=np.int64)
        segments = segment_series(times_ns, maximum_gap_s=0.001)
        covered = []
        for segment in segments:
            covered.extend(range(segment.start_index, segment.end_index))
        self.assertEqual(covered, list(range(times_ns.size)))


if __name__ == "__main__":
    unittest.main()
