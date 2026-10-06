"""Tests for data extractors.

Contents:
    JointStateCollectorTests: name resolution cases.
    WheelActuatorCollectorTests: six-wheel validation.
    ImuCollectorTests: orientation availability.
    ImageDecodeTests: encoding and step handling.
    PointCloudDecodeTests: XYZ layout validation.
    EvenlySampledIndicesTests: bounded sampling.
    BoundedEvenSamplerTests: streaming decimation.
    ImageCollectorBoundedMemoryTests: bounded decoding.
    PointCloudCollectorBoundedMemoryTests: bounded decoding.

Requires the sourced ROS environment for message types.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from std_msgs.msg import Header

from python_tools.data import extractors
from python_tools.data.models import WHEEL_COUNT


def _stamp(seconds: int, nanoseconds: int = 0) -> Time:
    """Build a Time stamp for test messages."""
    return Time(sec=seconds, nanosec=nanoseconds)


class JointStateCollectorTests(unittest.TestCase):
    """Tests for JointStateCollector."""

    def test_resolves_by_name_across_reordered_messages(
        self,
    ) -> None:
        """Reordered joints still resolve by name."""
        collector = extractors.JointStateCollector()
        first = JointState(
            header=Header(stamp=_stamp(1)),
            name=["a", "b"],
            position=[1.0, 2.0],
            velocity=[10.0, 20.0],
        )
        second = JointState(
            header=Header(stamp=_stamp(2)),
            name=["b", "a"],
            position=[3.0, 4.0],
            velocity=[30.0, 40.0],
        )
        self.assertTrue(collector.append(first))
        self.assertTrue(collector.append(second))
        series = collector.finalize(
            start_time_ns=1_000_000_000, maximum_gap_s=10.0
        )
        self.assertEqual(series.joint_names, ("a", "b"))
        # Second message: name[0]="b" pos=3.0, name[1]="a"
        # pos=4.0, so column "a" reads 4.0, "b" reads 3.0.
        np.testing.assert_allclose(series.position_rad[1], [4.0, 3.0])

    def test_rejects_duplicate_joint_name(self) -> None:
        """Duplicate joint name is rejected."""
        collector = extractors.JointStateCollector()
        message = JointState(
            header=Header(stamp=_stamp(1)),
            name=["a", "a"],
            position=[1.0, 2.0],
            velocity=[0.0, 0.0],
        )
        self.assertFalse(collector.append(message))
        self.assertEqual(collector.rejected_count, 1)

    def test_rejects_message_missing_a_joint(self) -> None:
        """Message with changed joint set is rejected."""
        collector = extractors.JointStateCollector()
        first = JointState(
            header=Header(stamp=_stamp(1)),
            name=["a", "b"],
            position=[0.0, 0.0],
            velocity=[0.0, 0.0],
        )
        second = JointState(
            header=Header(stamp=_stamp(2)),
            name=["a"],
            position=[0.0],
            velocity=[0.0],
        )
        self.assertTrue(collector.append(first))
        self.assertFalse(collector.append(second))
        self.assertEqual(collector.rejected_count, 1)

    def test_empty_collector_finalizes_to_none(self) -> None:
        """Empty collector finalizes to None."""
        collector = extractors.JointStateCollector()
        self.assertIsNone(
            collector.finalize(start_time_ns=0, maximum_gap_s=1.0)
        )


class WheelActuatorCollectorTests(unittest.TestCase):
    """Tests for WheelActuatorCollector validation."""

    def test_rejects_wrong_cardinality(self) -> None:
        """Message without six wheels is rejected."""
        from actuator_msgs.msg import Actuators

        collector = extractors.WheelActuatorCollector()
        bad_message = Actuators(
            header=Header(stamp=_stamp(1)),
            velocity=[1.0, 2.0],
            position=[0.0, 0.0],
        )
        self.assertFalse(collector.append(bad_message))
        self.assertEqual(collector.rejected_count, 1)

    def test_accepts_six_wheel_message(self) -> None:
        """Proper six-wheel message is accepted."""
        from actuator_msgs.msg import Actuators

        collector = extractors.WheelActuatorCollector()
        message = Actuators(
            header=Header(stamp=_stamp(1)),
            velocity=[1.0] * WHEEL_COUNT,
            position=[0.0] * WHEEL_COUNT,
        )
        self.assertTrue(collector.append(message))


class ImuCollectorTests(unittest.TestCase):
    """Tests for ImuCollector orientation detection."""

    def test_detects_available_orientation(self) -> None:
        """Valid covariance carries orientation through."""
        from sensor_msgs.msg import Imu

        collector = extractors.ImuCollector()
        message = Imu(header=Header(stamp=_stamp(1)))
        message.orientation.w = 1.0
        message.orientation_covariance[0] = 0.01
        collector.append(message)
        series = collector.finalize(
            start_time_ns=1_000_000_000, maximum_gap_s=10.0
        )
        self.assertTrue(series.has_orientation)
        self.assertFalse(np.any(np.isnan(series.orientation_xyzw)))

    def test_detects_unavailable_orientation(self) -> None:
        """Covariance -1 NaN-fills orientation."""
        from sensor_msgs.msg import Imu

        collector = extractors.ImuCollector()
        message = Imu(header=Header(stamp=_stamp(1)))
        message.orientation_covariance[0] = -1.0
        collector.append(message)
        series = collector.finalize(
            start_time_ns=1_000_000_000, maximum_gap_s=10.0
        )
        self.assertFalse(series.has_orientation)
        self.assertTrue(np.all(np.isnan(series.orientation_xyzw)))


class ImageDecodeTests(unittest.TestCase):
    """Tests for image decoding and step handling."""

    def test_decodes_rgb8(self) -> None:
        """rgb8 image decodes to itself."""
        image = Image(
            height=1, width=2, encoding="rgb8", step=6,
            data=bytes([255, 0, 0, 0, 255, 0]),
        )
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [255, 0, 0])
        np.testing.assert_array_equal(rgb[0, 1], [0, 255, 0])

    def test_decodes_bgr8_with_channel_swap(self) -> None:
        """bgr8 channels swap to RGB order."""
        image = Image(
            height=1, width=1, encoding="bgr8", step=3,
            data=bytes([10, 20, 30]),
        )
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [30, 20, 10])

    def test_decodes_mono8_by_broadcasting(self) -> None:
        """mono8 channel broadcasts to RGB."""
        image = Image(
            height=1, width=1, encoding="mono8", step=1,
            data=bytes([128]),
        )
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [128, 128, 128])

    def test_honors_step_with_row_padding(self) -> None:
        """Row padding is not read as pixels."""
        # width=1, rgb8 (3 bytes/px), step=6 has padding.
        row0 = bytes([1, 2, 3, 99, 99, 99])
        row1 = bytes([4, 5, 6, 99, 99, 99])
        image = Image(
            height=2, width=1, encoding="rgb8", step=6,
            data=row0 + row1,
        )
        rgb = extractors._decode_image_to_rgb(image)
        np.testing.assert_array_equal(rgb[0, 0], [1, 2, 3])
        np.testing.assert_array_equal(rgb[1, 0], [4, 5, 6])

    def test_unsupported_encoding_is_rejected_by_the_collector(
        self,
    ) -> None:
        """Unsupported encoding counts as rejected."""
        collector = extractors.ImageCollector(
            "topic", maximum_frames=10
        )
        image = Image(
            header=Header(stamp=_stamp(1)),
            height=1, width=1, encoding="32FC1", step=4,
            data=bytes(4),
        )
        collector.append(image)
        self.assertEqual(collector.rejected_encoding_count, 1)
        series = collector.finalize(start_time_ns=1_000_000_000)
        self.assertEqual(len(series.frames), 0)
        self.assertEqual(series.total_message_count, 1)


class PointCloudDecodeTests(unittest.TestCase):
    """Tests for XYZ point-cloud decoding."""

    def test_decodes_packed_xyz_floats(self) -> None:
        """Packed float32 XYZ decodes correctly."""
        points = np.array(
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32
        )
        message = PointCloud2(
            height=1,
            width=2,
            fields=[
                PointField(
                    name="x", offset=0,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="y", offset=4,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="z", offset=8,
                    datatype=PointField.FLOAT32, count=1,
                ),
            ],
            point_step=12,
            row_step=24,
            data=points.tobytes(),
            is_dense=True,
        )
        decoded = extractors._decode_xyz_point_cloud(message)
        np.testing.assert_allclose(decoded, points)

    def test_rejects_unexpected_field_layout(self) -> None:
        """Unexpected field layout raises."""
        message = PointCloud2(
            height=1,
            width=1,
            fields=[
                PointField(
                    name="intensity", offset=0,
                    datatype=PointField.FLOAT32, count=1,
                )
            ],
            point_step=4,
            row_step=4,
            data=bytes(4),
            is_dense=True,
        )
        with self.assertRaises(ValueError):
            extractors._decode_xyz_point_cloud(message)


class EvenlySampledIndicesTests(unittest.TestCase):
    """Tests for bounded evenly sampled indices."""

    def test_returns_everything_under_the_bound(self) -> None:
        """Indices under the bound are all kept."""
        self.assertEqual(
            extractors._evenly_sampled_indices(3, 10), [0, 1, 2]
        )

    def test_bounds_and_spans_the_full_range(self) -> None:
        """Sampled set keeps first and last index."""
        indices = extractors._evenly_sampled_indices(100, 5)
        self.assertLessEqual(len(indices), 5)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 99)

    def test_zero_bound_returns_nothing(self) -> None:
        """Zero bound returns no indices."""
        self.assertEqual(
            extractors._evenly_sampled_indices(10, 0), []
        )


class BoundedEvenSamplerTests(unittest.TestCase):
    """Tests for streaming bounded even sampling."""

    def test_buffer_never_exceeds_twice_the_bound_while_offering(
        self,
    ) -> None:
        """Buffer stays within twice the bound."""
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
            self.assertLessEqual(len(sampler._buffer), 2 * 5)

    def test_result_respects_the_bound_and_includes_first_and_last(
        self,
    ) -> None:
        """Result respects bound with first and last."""
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 9_999)
        self.assertLessEqual(len(result), 5)
        self.assertEqual(result[0], 0)
        self.assertEqual(result[-1], 9_999)

    def test_never_constructs_a_discarded_item(self) -> None:
        """Off-schedule items are never constructed."""
        constructed_indices: list[int] = []
        sampler = extractors._BoundedEvenSampler(maximum_count=3)
        for index in range(1_000):
            def factory(index=index) -> int:
                constructed_indices.append(index)
                return index

            sampler.offer(factory)
        # Far fewer than 1,000 factories were invoked.
        self.assertLess(len(constructed_indices), 200)

    def test_zero_bound_never_calls_the_factory(self) -> None:
        """Zero bound constructs nothing."""
        calls: list[int] = []
        sampler = extractors._BoundedEvenSampler(maximum_count=0)
        sampler.offer(lambda: calls.append(1))
        self.assertEqual(calls, [])
        self.assertEqual(sampler.finalize(), [])

    def test_zero_items_offered_returns_empty(self) -> None:
        """No offers finalizes to empty."""
        sampler = extractors._BoundedEvenSampler(maximum_count=5)
        self.assertEqual(sampler.finalize(), [])
        self.assertEqual(
            sampler.finalize(last_factory=lambda: "unused"), []
        )

    def test_limit_one_retains_only_the_true_final_item(
        self,
    ) -> None:
        """Single slot keeps the true final item."""
        # With one slot only the last item survives via
        # last_factory, never the first.
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 999)
        self.assertEqual(result, [999])

    def test_limit_one_without_last_factory_keeps_one_scheduled_item(
        self,
    ) -> None:
        """Single slot keeps one scheduled item."""
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize()
        self.assertEqual(len(result), 1)

    def test_limit_one_with_a_single_item_offered(self) -> None:
        """Single offer returns that one item."""
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        sampler.offer(lambda: "only")
        self.assertEqual(
            sampler.finalize(last_factory=lambda: "only"),
            ["only"],
        )

    def test_limit_two_retains_first_and_last(self) -> None:
        """Two slots keep first and last items."""
        sampler = extractors._BoundedEvenSampler(maximum_count=2)
        for index in range(500):
            sampler.offer(lambda index=index: index)
        result = sampler.finalize(last_factory=lambda: 999)
        self.assertEqual(result, [0, 999])

    def test_cardinality_never_exceeds_the_configured_limit(
        self,
    ) -> None:
        """Finalized size never exceeds the limit."""
        for maximum_count in (0, 1, 2, 3, 7, 50):
            for total in (0, 1, 2, 10, 10_000):
                sampler = extractors._BoundedEvenSampler(
                    maximum_count=maximum_count
                )
                for index in range(total):
                    sampler.offer(lambda index=index: index)
                if total > 0:
                    last_factory = lambda t=total: t - 1
                else:
                    last_factory = None
                result = sampler.finalize(last_factory=last_factory)
                self.assertLessEqual(
                    len(result),
                    maximum_count,
                    f"maximum_count={maximum_count} "
                    f"total={total}",
                )

    def test_ingestion_buffer_stays_within_its_documented_bound(
        self,
    ) -> None:
        """Single-slot buffer stays within its bound."""
        sampler = extractors._BoundedEvenSampler(maximum_count=1)
        for index in range(10_000):
            sampler.offer(lambda index=index: index)
            self.assertLessEqual(len(sampler._buffer), 2 * 1)


class ImageCollectorBoundedMemoryTests(unittest.TestCase):
    """Tests for ImageCollector bounded decoding."""

    def test_does_not_decode_every_message(self) -> None:
        """Only a bounded number of images decode."""
        decode_calls: list[int] = []
        original_decode = extractors._decode_image_to_rgb

        def counting_decode(msg: object):
            decode_calls.append(1)
            return original_decode(msg)

        message_count = 2_000
        collector = extractors.ImageCollector(
            "topic", maximum_frames=5
        )
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

        # Far fewer decodes than the message count.
        self.assertLess(len(decode_calls), 200)
        series = collector.finalize(start_time_ns=0)
        self.assertLessEqual(len(series.frames), 5)
        self.assertEqual(series.total_message_count, message_count)
        self.assertAlmostEqual(
            series.frames[0].time_s, 1.0, places=6
        )
        self.assertAlmostEqual(
            series.frames[-1].time_s,
            float(message_count),
            places=6,
        )

    def test_rejected_final_message_does_not_replace_the_last_valid(
        self,
    ) -> None:
        """Rejected final message is not the last frame."""
        # Only eligible messages update the true-last item;
        # checked with a single slot where it would show.
        collector = extractors.ImageCollector(
            "topic", maximum_frames=1
        )
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
        # The final message is malformed and must not win.
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
        self.assertAlmostEqual(
            series.frames[0].time_s, 20.0, places=6
        )


class PointCloudCollectorBoundedMemoryTests(unittest.TestCase):
    """Tests for PointCloudCollector bounded decoding."""

    def test_does_not_decode_every_message(self) -> None:
        """Only a bounded number of clouds decode."""
        decode_calls: list[int] = []
        original_decode = extractors._decode_xyz_point_cloud

        def counting_decode(msg: object):
            decode_calls.append(1)
            return original_decode(msg)

        message_count = 2_000
        collector = extractors.PointCloudCollector(
            maximum_snapshots=5
        )
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud_template = dict(
            height=1,
            width=1,
            fields=[
                PointField(
                    name="x", offset=0,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="y", offset=4,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="z", offset=8,
                    datatype=PointField.FLOAT32, count=1,
                ),
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
                collector.append(
                    cloud,
                    receive_time_ns=(index + 1) * 1_000_000_000,
                )
        finally:
            extractors._decode_xyz_point_cloud = original_decode

        # Far fewer decodes than the message count.
        self.assertLess(len(decode_calls), 200)
        series = collector.finalize(start_time_ns=0)
        self.assertEqual(len(series.point_counts), message_count)
        self.assertLessEqual(len(series.snapshots), 5)
        self.assertAlmostEqual(
            series.snapshots[0].time_s, 1.0, places=6
        )
        self.assertAlmostEqual(
            series.snapshots[-1].time_s,
            float(message_count),
            places=6,
        )

    def test_rejected_final_message_does_not_replace_the_last_valid(
        self,
    ) -> None:
        """Invalid final message is not the last snapshot."""
        # A zero-timestamp message must not become true-last;
        # checked with a single slot where it would show.
        collector = extractors.PointCloudCollector(
            maximum_snapshots=1
        )
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        cloud_template = dict(
            height=1,
            width=1,
            fields=[
                PointField(
                    name="x", offset=0,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="y", offset=4,
                    datatype=PointField.FLOAT32, count=1,
                ),
                PointField(
                    name="z", offset=8,
                    datatype=PointField.FLOAT32, count=1,
                ),
            ],
            point_step=12,
            row_step=12,
            data=points.tobytes(),
            is_dense=True,
        )
        for index in range(20):
            collector.append(
                PointCloud2(**cloud_template),
                receive_time_ns=(index + 1) * 1_000_000_000,
            )
        # Final message has a zero timestamp; drop it.
        collector.append(
            PointCloud2(**cloud_template), receive_time_ns=0
        )

        series = collector.finalize(start_time_ns=0)
        # The invalid final message is dropped from the
        # timestamp-aligned counts too, leaving 20 valid.
        self.assertEqual(len(series.point_counts), 20)
        self.assertEqual(len(series.snapshots), 1)
        self.assertAlmostEqual(
            series.snapshots[0].time_s, 20.0, places=6
        )


if __name__ == "__main__":
    unittest.main()
