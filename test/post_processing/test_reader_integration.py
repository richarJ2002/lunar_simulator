"""Tests for reader integration.

Contents:
    ReadBagTests: read against a minimal bag.
    ValidationTests: run and bag error paths.

Bags are written to a temporary directory by the test
itself, never as committed binary bags.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

import rosbag2_py
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from std_msgs.msg import Header

from python_tools.bag import reader


def _write_minimal_bag(bag_path: Path) -> None:
    """Write a minimal bag with ground-truth messages."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id="mcap"
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0,
            name="/alpha/localisation/ground_truth/odometry",
            type="nav_msgs/msg/Odometry",
            serialization_format="cdr",
        )
    )
    for i in range(3):
        message = Odometry()
        message.header = Header(
            stamp=Time(sec=1, nanosec=i * 100_000_000)
        )
        message.header.frame_id = "alpha/startup_fixed"
        message.child_frame_id = "alpha/base_link"
        message.pose.pose.position.x = float(i)
        message.pose.pose.orientation.w = 1.0
        writer.write(
            "/alpha/localisation/ground_truth/odometry",
            serialize_message(message),
            1_000_000_000 + i * 100_000_000,
        )
    # One diagnostics array between odometry samples
    # exercises DiagnosticArray routing end to end.
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=1,
            name="/alpha/diagnostics",
            type="diagnostic_msgs/msg/DiagnosticArray",
            serialization_format="cdr",
        )
    )
    diagnostics = DiagnosticArray()
    diagnostics.header = Header(
        stamp=Time(sec=1, nanosec=150_000_000)
    )
    diagnostics.status = [
        DiagnosticStatus(
            level=DiagnosticStatus.WARN,
            name="inertial_odometry",
            message="IMU calibrating 40/100",
            values=[KeyValue(key="calibrated", value="false")],
        ),
        DiagnosticStatus(
            level=DiagnosticStatus.OK, name="visual_odometry"
        ),
    ]
    writer.write(
        "/alpha/diagnostics",
        serialize_message(diagnostics),
        1_150_000_000,
    )
    del writer


class ReadBagTests(unittest.TestCase):
    """Tests for read_bag against a real bag."""

    def setUp(self) -> None:
        """Create a fresh temporary bag for each test."""
        self._temp_dir = tempfile.mkdtemp()
        self.bag_path = Path(self._temp_dir) / "localisation"
        _write_minimal_bag(self.bag_path)

    def tearDown(self) -> None:
        """Remove the temporary bag directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def test_reads_published_topic(self) -> None:
        """Published topic extracts with elapsed time."""
        result = reader.read_bag(self.bag_path)
        series = result.odometry[
            "/alpha/localisation/ground_truth/odometry"
        ]
        self.assertEqual(series.times_s.tolist(), [0.0, 0.1, 0.2])
        self.assertEqual(result.storage_identifier, "mcap")

    def test_reads_recorded_diagnostics(self) -> None:
        """Recorded array flattens onto bag-elapsed time."""
        result = reader.read_bag(self.bag_path)
        series = result.diagnostic_arrays["/alpha/diagnostics"]
        self.assertEqual(
            [sample.name for sample in series.samples],
            ["inertial_odometry", "visual_odometry"],
        )
        self.assertAlmostEqual(series.samples[0].time_s, 0.15)
        self.assertEqual(series.samples[0].level, 1)
        self.assertEqual(
            series.samples[0].values, {"calibrated": "false"}
        )

    def test_health_reported_for_never_published_registered_topic(
        self,
    ) -> None:
        """Unpublished topic still gets health entry."""
        result = reader.read_bag(self.bag_path)
        health = result.topic_health["/alpha/imu"]
        self.assertEqual(health.message_count, 0)
        self.assertIsNone(health.effective_rate_hz)

    def test_published_topic_health_counts_and_rate(self) -> None:
        """Published topic reports count and rate."""
        result = reader.read_bag(self.bag_path)
        health = result.topic_health[
            "/alpha/localisation/ground_truth/odometry"
        ]
        self.assertEqual(health.message_count, 3)
        self.assertAlmostEqual(health.effective_rate_hz, 10.0)


class ValidationTests(unittest.TestCase):
    """Tests for run and bag validation errors."""

    def test_missing_test_run_directory_raises(self) -> None:
        """Nonexistent path raises BagValidationError."""
        with self.assertRaises(reader.BagValidationError):
            reader.validate_test_run_dir(
                Path("/nonexistent/path/for/test")
            )

    def test_directory_missing_parameters_or_ros_raises(
        self,
    ) -> None:
        """Directory without run shape raises."""
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(reader.BagValidationError):
                reader.validate_test_run_dir(Path(tmp))

    def test_old_run_with_no_bag_raises_actionable_error(
        self,
    ) -> None:
        """Old run without a bag names that case."""
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            (test_run_dir / "ros" / "bags").mkdir(parents=True)
            with self.assertRaises(
                reader.BagValidationError
            ) as context:
                reader.discover_bag_path(
                    test_run_dir, explicit_bag_path=None
                )
            self.assertIn("old run", str(context.exception))

    def test_bag_directory_without_metadata_raises(self) -> None:
        """Bag without metadata names that case."""
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            bag_dir = test_run_dir / "ros" / "bags" / "localisation"
            bag_dir.mkdir(parents=True)
            with self.assertRaises(
                reader.BagValidationError
            ) as context:
                reader.discover_bag_path(
                    test_run_dir, explicit_bag_path=None
                )
            self.assertIn("metadata.yaml", str(context.exception))


if __name__ == "__main__":
    unittest.main()
