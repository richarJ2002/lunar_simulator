"""!
@brief  Integration tests for python_tools.bag.reader against a small
        rosbag2 bag written to a temporary directory by this test itself
        (never a committed binary bag, per the plan). Covers a normal
        read, a registered-but-never-published topic's health, and the
        run/bag validation error paths for an old run with no bag, a
        directory that is not a valid bag, and a missing test-run
        directory shape.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

import rosbag2_py
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from std_msgs.msg import Header

from python_tools.bag import reader


def _write_minimal_bag(bag_path: Path) -> None:
    """!
    @brief   Writes a minimal bag with a few ground-truth Odometry
             messages on one registered topic, for reader integration
             tests.

    @param   bag_path
             Destination bag directory (created by the writer).

    @return  None
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
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
        message.header = Header(stamp=Time(sec=1, nanosec=i * 100_000_000))
        message.header.frame_id = "alpha/startup_fixed"
        message.child_frame_id = "alpha/base_link"
        message.pose.pose.position.x = float(i)
        message.pose.pose.orientation.w = 1.0
        writer.write(
            "/alpha/localisation/ground_truth/odometry",
            serialize_message(message),
            1_000_000_000 + i * 100_000_000,
        )
    del writer


class ReadBagTests(unittest.TestCase):
    """!
    @brief  Tests for `read_bag` against a real, minimal on-disk bag.
    """

    def setUp(self) -> None:
        """!
        @brief  Creates a fresh temporary bag for each test.
        """
        self._temp_dir = tempfile.mkdtemp()
        self.bag_path = Path(self._temp_dir) / "localisation"
        _write_minimal_bag(self.bag_path)

    def tearDown(self) -> None:
        """!
        @brief  Removes the temporary bag directory.
        """
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def test_reads_published_topic(self) -> None:
        """!
        @brief  A published, registered topic is extracted into the
                expected series with correct elapsed-time normalization.
        """
        result = reader.read_bag(self.bag_path)
        series = result.odometry["/alpha/localisation/ground_truth/odometry"]
        self.assertEqual(series.times_s.tolist(), [0.0, 0.1, 0.2])
        self.assertEqual(result.storage_identifier, "mcap")

    def test_health_reported_for_never_published_registered_topic(self) -> None:
        """!
        @brief  A registered topic that never appeared in the bag still
                gets a zero-message TopicHealth entry, not an omission.
        """
        result = reader.read_bag(self.bag_path)
        health = result.topic_health["/alpha/imu"]
        self.assertEqual(health.message_count, 0)
        self.assertIsNone(health.effective_rate_hz)

    def test_published_topic_health_counts_and_rate(self) -> None:
        """!
        @brief  A published topic's health reports the correct message
                count and effective rate.
        """
        result = reader.read_bag(self.bag_path)
        health = result.topic_health["/alpha/localisation/ground_truth/odometry"]
        self.assertEqual(health.message_count, 3)
        self.assertAlmostEqual(health.effective_rate_hz, 10.0)


class ValidationTests(unittest.TestCase):
    """!
    @brief  Tests for `validate_test_run_dir`/`discover_bag_path`'s
            actionable error paths.
    """

    def test_missing_test_run_directory_raises(self) -> None:
        """!
        @brief  A nonexistent path raises `BagValidationError`.
        """
        with self.assertRaises(reader.BagValidationError):
            reader.validate_test_run_dir(Path("/nonexistent/path/for/test"))

    def test_directory_missing_parameters_or_ros_raises(self) -> None:
        """!
        @brief  A directory that exists but lacks parameters/ and ros/
                (not a captured test run) raises.
        """
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(reader.BagValidationError):
                reader.validate_test_run_dir(Path(tmp))

    def test_old_run_with_no_bag_raises_actionable_error(self) -> None:
        """!
        @brief  A run directory with no ros/bags/localisation at all (an
                old run captured before automatic recording) raises a
                message naming that possibility.
        """
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            (test_run_dir / "ros" / "bags").mkdir(parents=True)
            with self.assertRaises(reader.BagValidationError) as context:
                reader.discover_bag_path(test_run_dir, explicit_bag_path=None)
            self.assertIn("old run", str(context.exception))

    def test_bag_directory_without_metadata_raises(self) -> None:
        """!
        @brief  A directory that exists but has no metadata.yaml (an
                unfinalized recording) raises a message naming that.
        """
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            bag_dir = test_run_dir / "ros" / "bags" / "localisation"
            bag_dir.mkdir(parents=True)
            with self.assertRaises(reader.BagValidationError) as context:
                reader.discover_bag_path(test_run_dir, explicit_bag_path=None)
            self.assertIn("metadata.yaml", str(context.exception))


if __name__ == "__main__":
    unittest.main()
