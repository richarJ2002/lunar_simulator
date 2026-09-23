"""!
@brief  Tests for python_tools.data.extractors: per-category collector
        round-trips and boundary cases (joint-name reordering, duplicate
        and missing joints, six-wheel cardinality rejection, image
        encoding support/rejection honoring `step`, PointCloud2 field
        layout validation, and bounded/evenly sampled frame selection).
        Requires the sourced ROS environment (message types), matching
        this repo's other Python tests.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from std_msgs.msg import Header

from python_tools.data import extractors
from python_tools.data.models import WHEEL_COUNT


def _stamp(seconds: int, nanoseconds: int = 0) -> Time:
    """!
    @brief   Builds a builtin_interfaces/msg/Time for test messages.

    @param   seconds
             The stamp's `sec` field.
    @param   nanoseconds
             The stamp's `nanosec` field.

    @return  The constructed stamp.
    """
    return Time(sec=seconds, nanosec=nanoseconds)


class JointStateCollectorTests(unittest.TestCase):
    """!
    @brief  Tests for `JointStateCollector`.
    """

    def test_resolves_by_name_across_reordered_messages(self) -> None:
        """!
        @brief  A second message with the same joint set in a different
                array order still resolves each column to the correct
                joint.
        """
        collector = extractors.JointStateCollector()
        first = JointState(header=Header(stamp=_stamp(1)), name=["a", "b"], position=[1.0, 2.0], velocity=[10.0, 20.0])
        second = JointState(header=Header(stamp=_stamp(2)), name=["b", "a"], position=[3.0, 4.0], velocity=[30.0, 40.0])
        self.assertTrue(collector.append(first))
        self.assertTrue(collector.append(second))
        series = collector.finalize(start_time_ns=1_000_000_000, maximum_gap_s=10.0)
        self.assertEqual(series.joint_names, ("a", "b"))
        # Second message: name[0]="b" pos=3.0, name[1]="a" pos=4.0, so
        # column "a" (index 0) must read 4.0, column "b" (index 1) 3.0.
        np.testing.assert_allclose(series.position_rad[1], [4.0, 3.0])

    def test_rejects_duplicate_joint_name(self) -> None:
        """!
        @brief  A message with a duplicate joint name is rejected, not
                silently corrupting the series.
        """
        collector = extractors.JointStateCollector()
        message = JointState(header=Header(stamp=_stamp(1)), name=["a", "a"], position=[1.0, 2.0], velocity=[0.0, 0.0])
        self.assertFalse(collector.append(message))
        self.assertEqual(collector.rejected_count, 1)

    def test_rejects_message_missing_a_joint(self) -> None:
        """!
        @brief  A later message whose joint set does not match the
                canonical set established by the first message is
                rejected.
        """
        collector = extractors.JointStateCollector()
        first = JointState(header=Header(stamp=_stamp(1)), name=["a", "b"], position=[0.0, 0.0], velocity=[0.0, 0.0])
        second = JointState(header=Header(stamp=_stamp(2)), name=["a"], position=[0.0], velocity=[0.0])
        self.assertTrue(collector.append(first))
        self.assertFalse(collector.append(second))
        self.assertEqual(collector.rejected_count, 1)

    def test_empty_collector_finalizes_to_none(self) -> None:
        """!
        @brief  A collector with no accepted messages finalizes to `None`.
        """
        collector = extractors.JointStateCollector()
        self.assertIsNone(collector.finalize(start_time_ns=0, maximum_gap_s=1.0))


class WheelActuatorCollectorTests(unittest.TestCase):
    """!
    @brief  Tests for `WheelActuatorCollector`'s cardinality validation.
    """

    def test_rejects_wrong_cardinality(self) -> None:
        """!
        @brief  A message without exactly six wheel values is rejected.
        """
        from actuator_msgs.msg import Actuators

        collector = extractors.WheelActuatorCollector()
        bad_message = Actuators(header=Header(stamp=_stamp(1)), velocity=[1.0, 2.0], position=[0.0, 0.0])
        self.assertFalse(collector.append(bad_message))
        self.assertEqual(collector.rejected_count, 1)

    def test_accepts_six_wheel_message(self) -> None:
        """!
        @brief  A properly shaped six-wheel message is accepted.
        """
        from actuator_msgs.msg import Actuators

        collector = extractors.WheelActuatorCollector()
        message = Actuators(
            header=Header(stamp=_stamp(1)),
            velocity=[1.0] * WHEEL_COUNT,
            position=[0.0] * WHEEL_COUNT,
        )
        self.assertTrue(collector.append(message))


class ImuCollectorTests(unittest.TestCase):
    """!
    @brief  Tests for `ImuCollector`'s orientation-availability detection.
    """

    def test_detects_available_orientation(self) -> None:
        """!
        @brief  orientation_covariance[0] != -1 means orientation is
                carried through, not NaN-filled.
        """
        from sensor_msgs.msg import Imu

        collector = extractors.ImuCollector()
        message = Imu(header=Header(stamp=_stamp(1)))
        message.orientation.w = 1.0
        message.orientation_covariance[0] = 0.01
        collector.append(message)
        series = collector.finalize(start_time_ns=1_000_000_000, maximum_gap_s=10.0)
        self.assertTrue(series.has_orientation)
        self.assertFalse(np.any(np.isnan(series.orientation_xyzw)))

    def test_detects_unavailable_orientation(self) -> None:
        """!
        @brief  orientation_covariance[0] == -1 (the sensor_msgs/Imu
                convention) NaN-fills orientation instead of a misleading
                identity quaternion.
        """
        from sensor_msgs.msg import Imu

        collector = extractors.ImuCollector()
        message = Imu(header=Header(stamp=_stamp(1)))
        message.orientation_covariance[0] = -1.0
        collector.append(message)
        series = collector.finalize(start_time_ns=1_000_000_000, maximum_gap_s=10.0)
        self.assertFalse(series.has_orientation)
        self.assertTrue(np.all(np.isnan(series.orientation_xyzw)))


class ImageDecodeTests(unittest.TestCase):
    """!
    @brief  Tests for `_decode_image_to_rgb` across every supported
            encoding, and for step-aware row handling.
    """

    def test_decodes_rgb8(self) -> None:
        """!
        @brief  An rgb8 image decodes to itself (already RGB).
        """
        image = Image(height=1, width=2, encoding="rgb8", step=6, data=bytes([255, 0, 0, 0, 255, 0]))
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [255, 0, 0])
        np.testing.assert_array_equal(rgb[0, 1], [0, 255, 0])

    def test_decodes_bgr8_with_channel_swap(self) -> None:
        """!
        @brief  A bgr8 image's channels are swapped to RGB order.
        """
        image = Image(height=1, width=1, encoding="bgr8", step=3, data=bytes([10, 20, 30]))
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [30, 20, 10])

    def test_decodes_mono8_by_broadcasting(self) -> None:
        """!
        @brief  A mono8 image's single channel is broadcast to all three
                RGB channels.
        """
        image = Image(height=1, width=1, encoding="mono8", step=1, data=bytes([128]))
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [128, 128, 128])

    def test_honors_step_with_row_padding(self) -> None:
        """!
        @brief  Row padding beyond width*bytes_per_pixel is not
                misinterpreted as pixel data.
        """
        # width=1, rgb8 (3 bytes/px), but step=6 (3 bytes of padding/row).
        row0 = bytes([1, 2, 3, 99, 99, 99])
        row1 = bytes([4, 5, 6, 99, 99, 99])
        image = Image(height=2, width=1, encoding="rgb8", step=6, data=row0 + row1)
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [1, 2, 3])
        np.testing.assert_array_equal(rgb[1, 0], [4, 5, 6])

    def test_unsupported_encoding_is_rejected_by_the_collector(self) -> None:
        """!
        @brief  An image with an unsupported encoding is counted as
                rejected rather than decoded (or crashing).
        """
        collector = extractors.ImageCollector("topic", maximum_frames=10)
        image = Image(header=Header(stamp=_stamp(1)), height=1, width=1, encoding="32FC1", step=4, data=bytes(4))
        collector.append(image)
        self.assertEqual(collector.rejected_encoding_count, 1)
        series = collector.finalize(start_time_ns=1_000_000_000)
        self.assertEqual(len(series.frames), 0)
        self.assertEqual(series.total_message_count, 1)


class PointCloudDecodeTests(unittest.TestCase):
    """!
    @brief  Tests for `_decode_xyz_point_cloud`.
    """

    def test_decodes_packed_xyz_floats(self) -> None:
        """!
        @brief  A standard packed-float32 XYZ cloud decodes to the
                expected point coordinates.
        """
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        message = PointCloud2(
            height=1,
            width=2,
            fields=[
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=24,
            data=points.tobytes(),
            is_dense=True,
        )
        decoded = extractors._decode_xyz_point_cloud(message)
        np.testing.assert_allclose(decoded, points)

    def test_rejects_unexpected_field_layout(self) -> None:
        """!
        @brief  A cloud whose fields do not match the expected packed XYZ
                layout raises rather than misreading its bytes.
        """
        message = PointCloud2(
            height=1,
            width=1,
            fields=[PointField(name="intensity", offset=0, datatype=PointField.FLOAT32, count=1)],
            point_step=4,
            row_step=4,
            data=bytes(4),
            is_dense=True,
        )
        with self.assertRaises(ValueError):
            extractors._decode_xyz_point_cloud(message)


class EvenlySampledIndicesTests(unittest.TestCase):
    """!
    @brief  Tests for `_evenly_sampled_indices`'s bounded-sampling
            behavior.
    """

    def test_returns_everything_under_the_bound(self) -> None:
        """!
        @brief  When the bound is not exceeded, every index is kept.
        """
        self.assertEqual(extractors._evenly_sampled_indices(3, 10), [0, 1, 2])

    def test_bounds_and_spans_the_full_range(self) -> None:
        """!
        @brief  Sampling to a smaller bound still includes the first and
                last index and respects the bound.
        """
        indices = extractors._evenly_sampled_indices(100, 5)
        self.assertLessEqual(len(indices), 5)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 99)

    def test_zero_bound_returns_nothing(self) -> None:
        """!
        @brief  A zero or negative bound returns no indices at all.
        """
        self.assertEqual(extractors._evenly_sampled_indices(10, 0), [])


class BoundedEvenSamplerTests(unittest.TestCase):
    """!
    @brief  Tests for `_BoundedEvenSampler`, the streaming, bounded,
            evenly spaced decimation `ImageCollector`/`PointCloudCollector`
            use to fix Defect 2 in the plan's 2026-09-22 audit: the
            previous implementation decoded and retained every frame
            during ingestion, only discarding down to the configured bound
            in `finalize()`. These tests assert the bound holds *during*
            ingestion (not only in the finalized result) and that a
            discarded item is never even constructed.
    """

    def test_buffer_never_exceeds_twice_the_bound_while_offering(self) -> None:
        """!
        @brief  Feeding far more items than the configured bound never
                lets the internal buffer grow past 2x the bound at any
                point during ingestion.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
            self.assertLessEqual(len(sampler._buffer), 2 * 5)

    def test_result_respects_the_bound_and_includes_first_and_last(self) -> None:
        """!
        @brief  The finalized sample never exceeds the bound, and includes
                index 0 and the true last offered item when a
                `last_factory` is supplied.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 9_999)
        self.assertLessEqual(len(result), 5)
        self.assertEqual(result[0], 0)
        self.assertEqual(result[-1], 9_999)

    def test_never_constructs_a_discarded_item(self) -> None:
        """!
        @brief  An item that does not land on the step schedule is never
                constructed at all -- proves this is bounded decoding, not
                just bounded retention.
        """
        constructed_indices: list[int] = []
        sampler = extractors._BoundedEvenSampler(maximum_count=3)
        for index in range(1_000):
            def factory(index=index) -> int:
                constructed_indices.append(index)
                return index

            sampler.offer(factory)
        # Far fewer than 1,000 factories were ever invoked.
        self.assertLess(len(constructed_indices), 200)

    def test_zero_bound_never_calls_the_factory(self) -> None:
        """!
        @brief  A zero bound never retains or constructs anything.
        """
        calls: list[int] = []
        sampler = extractors._BoundedEvenSampler(maximum_count=0)
        sampler.offer(lambda: calls.append(1))
        self.assertEqual(calls, [])
        self.assertEqual(sampler.finalize(), [])

    def test_zero_items_offered_returns_empty(self) -> None:
        """!
        @brief  A sampler that never receives an offer() call finalizes to
                an empty result regardless of its configured bound, with
                or without a last_factory.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        self.assertEqual(sampler.finalize(), [])
        self.assertEqual(sampler.finalize(last_factory=lambda: "unused"), [])

    def test_limit_one_retains_only_the_true_final_item(self) -> None:
        """!
        @brief  Regression for the 2026-09-23 correction: `_evenly_
                sampled_indices(n, 1)` used to be reached for a
                single-slot sampler and returned only index 0
                (`np.linspace(..., num=1)`'s own behavior), silently
                dropping the true last item the class's own docstring
                promised. With a single slot, finalize() must now keep the
                true final offered item (forced in via last_factory),
                never the first -- a single slot cannot honor both
                endpoints at once (see the class's one-slot contract).
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 999)
        self.assertEqual(result, [999])

    def test_limit_one_without_last_factory_keeps_one_scheduled_item(self) -> None:
        """!
        @brief  Without a last_factory to force in the true last item, a
                single-slot sampler still returns exactly one item: the
                most recent one that landed on its own step schedule --
                the best available answer when the caller has no true-last
                item to supply.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize()
        self.assertEqual(len(result), 1)

    def test_limit_one_with_a_single_item_offered(self) -> None:
        """!
        @brief  Offering exactly one item to a single-slot sampler returns
                that one item.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        sampler.offer(lambda: "only")
        self.assertEqual(sampler.finalize(last_factory=lambda: "only"), ["only"])

    def test_limit_two_retains_first_and_last(self) -> None:
        """!
        @brief  With exactly two slots, both the true first and true last
                offered items survive -- the class's contract for
                maximum_count >= 2, pinned here at the smallest such
                value.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=2)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 999)
        self.assertEqual(result, [0, 999])

    def test_cardinality_never_exceeds_the_configured_limit(self) -> None:
        """!
        @brief  Across a sweep of stream lengths (including zero, one, and
                many items offered) and limits (including 0, 1, and 2),
                the finalized result never exceeds maximum_count.
        """
        for maximum_count in (0, 1, 2, 3, 7, 50):
            for total in (0, 1, 2, 10, 10_000):
                sampler = extractors._BoundedEvenSampler(maximum_count=maximum_count)
                for index in range(total):
                    sampler.offer(lambda index=index: index)
                last_factory = (lambda t=total: t - 1) if total > 0 else None
                result = sampler.finalize(last_factory=last_factory)
                self.assertLessEqual(
                    len(result),
                    maximum_count,
                    f"maximum_count={maximum_count} total={total}",
                )

    def test_ingestion_buffer_stays_within_its_documented_bound_for_a_single_slot(
        self,
    ) -> None:
        """!
        @brief  The `2 * maximum_count` ingestion-time bound (see
                `_thin()`) holds even at the smallest nonzero bound, where
                the margin for error is tightest.
        """
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
            self.assertLessEqual(len(sampler._buffer), 2 * 1)


class ImageCollectorBoundedMemoryTests(unittest.TestCase):
    """!
    @brief  Black-box confirmation that `ImageCollector` bounds decoding
            during ingestion (Defect 2), not just the finalized frame
            count, and still preserves the first/last frame and total
            message count.
    """

    def test_does_not_decode_every_message(self) -> None:
        """!
        @brief  Feeding far more messages than `maximum_frames` decodes
                only a small, bounded number of them, while `finalize()`
                still reports the true total message count and a
                bounded, first/last-inclusive frame set.
        """
        decode_calls: list[int] = []
        original_decode = extractors._decode_image_to_rgb

        def counting_decode(msg: object):
            decode_calls.append(1)
            return original_decode(msg)

        message_count = 2_000
        collector = extractors.ImageCollector("topic", maximum_frames=5)
        extractors._decode_image_to_rgb = counting_decode
        try:
            for index in range(message_count):
                image = Image(
                    header=Header(stamp=_stamp(index + 1)),
                    height=1,
                    width=1,
                    encoding="rgb8",
                    step=3,
                    data=bytes([1, 2, 3]),
                )
                collector.append(image)
        finally:
            extractors._decode_image_to_rgb = original_decode

        # Far fewer decodes during ingestion than the message count.
        self.assertLess(len(decode_calls), 200)
        series = collector.finalize(start_time_ns=0)
        self.assertLessEqual(len(series.frames), 5)
        self.assertEqual(series.total_message_count, message_count)
        self.assertAlmostEqual(series.frames[0].time_s, 1.0, places=6)
        self.assertAlmostEqual(series.frames[-1].time_s, float(message_count), places=6)

    def test_rejected_final_message_does_not_replace_the_last_valid_frame(self) -> None:
        """!
        @brief  A malformed (unsupported-encoding) message arriving last
                in the stream must never become the sampler's "true last"
                item -- `_last_eligible` is only updated for genuinely
                eligible messages. Checked with a single-slot sampler
                (maximum_frames=1), where the returned item IS the
                true-last item and would visibly carry the wrong
                timestamp if this were broken.
        """
        collector = extractors.ImageCollector("topic", maximum_frames=1)
        for index in range(20):
            image = Image(
                header=Header(stamp=_stamp(index + 1)),
                height=1,
                width=1,
                encoding="rgb8",
                step=3,
                data=bytes([1, 2, 3]),
            )
            collector.append(image)
        # The final message in the stream is malformed and must not become
        # the retained "last" frame.
        rejected_final = Image(
            header=Header(stamp=_stamp(21)),
            height=1,
            width=1,
            encoding="not_a_real_encoding",
            step=3,
            data=bytes([1, 2, 3]),
        )
        collector.append(rejected_final)

        series = collector.finalize(start_time_ns=0)
        self.assertEqual(series.total_message_count, 21)
        self.assertEqual(series.rejected_encoding_count, 1)
        self.assertEqual(len(series.frames), 1)
        self.assertAlmostEqual(series.frames[0].time_s, 20.0, places=6)


class PointCloudCollectorBoundedMemoryTests(unittest.TestCase):
    """!
    @brief  Black-box confirmation that `PointCloudCollector` bounds
            decoding during ingestion (Defect 2), not just the finalized
            snapshot count, while every message's point count is still
            tracked for the full-run rate/count series.
    """

    def test_does_not_decode_every_message(self) -> None:
        """!
        @brief  Feeding far more messages than `maximum_snapshots` decodes
                only a small, bounded number of full point clouds, while
                `finalize()` still reports every message's point count and
                a bounded, first/last-inclusive snapshot set.
        """
        decode_calls: list[int] = []
        original_decode = extractors._decode_xyz_point_cloud

        def counting_decode(msg: object):
            decode_calls.append(1)
            return original_decode(msg)

        message_count = 2_000
        collector = extractors.PointCloudCollector(maximum_snapshots=5)
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud_template = dict(
            height=1,
            width=1,
            fields=[
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=12,
            data=points.tobytes(),
            is_dense=True,
        )
        extractors._decode_xyz_point_cloud = counting_decode
        try:
            for index in range(message_count):
                cloud = PointCloud2(**cloud_template)
                collector.append(cloud, receive_time_ns=(index + 1) * 1_000_000_000)
        finally:
            extractors._decode_xyz_point_cloud = original_decode

        # Far fewer decodes during ingestion than the message count.
        self.assertLess(len(decode_calls), 200)
        series = collector.finalize(start_time_ns=0)
        self.assertEqual(len(series.point_counts), message_count)
        self.assertLessEqual(len(series.snapshots), 5)
        self.assertAlmostEqual(series.snapshots[0].time_s, 1.0, places=6)
        self.assertAlmostEqual(series.snapshots[-1].time_s, float(message_count), places=6)

    def test_rejected_final_message_does_not_replace_the_last_valid_snapshot(self) -> None:
        """!
        @brief  A message with an invalid (zero) timestamp arriving last
                in the stream must never become the sampler's "true last"
                item. Checked with a single-slot sampler
                (maximum_snapshots=1), where the returned item IS the
                true-last item and would visibly carry the wrong
                timestamp if this were broken.
        """
        collector = extractors.PointCloudCollector(maximum_snapshots=1)
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud_template = dict(
            height=1,
            width=1,
            fields=[
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=12,
            data=points.tobytes(),
            is_dense=True,
        )
        for index in range(20):
            collector.append(
                PointCloud2(**cloud_template), receive_time_ns=(index + 1) * 1_000_000_000
            )
        # Final message has an invalid (zero) timestamp -- must not become
        # the retained "last" snapshot.
        collector.append(PointCloud2(**cloud_template), receive_time_ns=0)

        series = collector.finalize(start_time_ns=0)
        # The invalid-timestamp final message is dropped from the
        # timestamp-aligned point_counts array too (masked the same way as
        # times_s), leaving the 20 genuinely valid messages.
        self.assertEqual(len(series.point_counts), 20)
        self.assertEqual(len(series.snapshots), 1)
        self.assertAlmostEqual(series.snapshots[0].time_s, 20.0, places=6)


if __name__ == "__main__":
    unittest.main()
