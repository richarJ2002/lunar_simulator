#!/usr/bin/env python3
"""Collect timestamp-aligned localisation and truth trajectory metrics."""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterable, Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class Sample:
    """One timestamped odometry sample in startup-fixed/body frames."""

    timestamp_s: float
    position_fixed_m: tuple[float, float, float]
    quaternion_body_to_fixed: tuple[float, float, float, float]
    velocity_body_mps: tuple[float, float, float]


def vector_dot(left: Sequence[float], right: Sequence[float]) -> float:
    """Return the dot product of two three-component vectors."""
    return sum(left[index] * right[index] for index in range(3))


def vector_norm(vector: Sequence[float]) -> float:
    """Return a three-component vector's Euclidean norm."""
    return math.sqrt(vector_dot(vector, vector))


def normalize_vector(vector: Sequence[float]) -> tuple[float, float, float]:
    """Return a unit vector, raising when its direction is undefined."""
    magnitude = vector_norm(vector)
    if not math.isfinite(magnitude) or magnitude <= 1.0e-12:
        raise ValueError("vector magnitude must be finite and non-zero")
    return tuple(component / magnitude for component in vector)  # type: ignore[return-value]


def normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    """Normalize an x-y-z-w quaternion."""
    magnitude = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(magnitude) or magnitude <= 1.0e-12:
        raise ValueError("quaternion norm must be finite and non-zero")
    return tuple(component / magnitude for component in quaternion)  # type: ignore[return-value]


def rotate_vector(
    quaternion_xyzw: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    """Rotate a vector from body into fixed using an x-y-z-w quaternion."""
    quaternion = normalize_quaternion(quaternion_xyzw)
    quaternion_vector = quaternion[:3]
    scalar = quaternion[3]
    first_cross = (
        quaternion_vector[1] * vector[2] - quaternion_vector[2] * vector[1],
        quaternion_vector[2] * vector[0] - quaternion_vector[0] * vector[2],
        quaternion_vector[0] * vector[1] - quaternion_vector[1] * vector[0],
    )
    second_cross = (
        quaternion_vector[1] * first_cross[2]
        - quaternion_vector[2] * first_cross[1],
        quaternion_vector[2] * first_cross[0]
        - quaternion_vector[0] * first_cross[2],
        quaternion_vector[0] * first_cross[1]
        - quaternion_vector[1] * first_cross[0],
    )
    return tuple(
        vector[index]
        + 2.0 * (scalar * first_cross[index] + second_cross[index])
        for index in range(3)
    )  # type: ignore[return-value]


def orientation_error_rad(
    estimate_xyzw: Sequence[float], truth_xyzw: Sequence[float]
) -> float:
    """Return shortest quaternion geodesic error in radians."""
    estimate = normalize_quaternion(estimate_xyzw)
    truth = normalize_quaternion(truth_xyzw)
    cosine_half_angle = min(1.0, abs(sum(a * b for a, b in zip(estimate, truth))))
    return 2.0 * math.acos(cosine_half_angle)


def sample_from_message(message: Odometry) -> Sample:
    """Copy the bounded fields required from one ROS odometry message."""
    stamp = message.header.stamp
    timestamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    pose = message.pose.pose
    twist = message.twist.twist
    return Sample(
        timestamp_s=timestamp_s,
        position_fixed_m=(pose.position.x, pose.position.y, pose.position.z),
        quaternion_body_to_fixed=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        velocity_body_mps=(twist.linear.x, twist.linear.y, twist.linear.z),
    )


class TrajectoryCollector(Node):
    """Align estimate samples to nearest truth timestamps with bounded storage."""

    def __init__(
        self,
        truth_topic: str,
        estimate_topic: str,
        gravity_fixed: Sequence[float],
        maximum_alignment_age_s: float,
        maximum_truth_samples: int,
    ) -> None:
        super().__init__("localisation_trajectory_collector")
        self._gravity_unit_fixed = normalize_vector(gravity_fixed)
        self._maximum_alignment_age_s = maximum_alignment_age_s
        self._truth_samples: Deque[Sample] = deque(maxlen=maximum_truth_samples)
        self.rows: list[dict[str, float]] = []

        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Odometry, truth_topic, self._handle_truth, sensor_qos)
        self.create_subscription(
            Odometry, estimate_topic, self._handle_estimate, sensor_qos
        )

    def _handle_truth(self, message: Odometry) -> None:
        """Retain one bounded truth sample."""
        self._truth_samples.append(sample_from_message(message))

    def _handle_estimate(self, message: Odometry) -> None:
        """Align and score one estimate sample against nearest truth."""
        estimate = sample_from_message(message)
        if not self._truth_samples:
            return
        truth = min(
            self._truth_samples,
            key=lambda sample: abs(sample.timestamp_s - estimate.timestamp_s),
        )
        alignment_age_s = abs(truth.timestamp_s - estimate.timestamp_s)
        if alignment_age_s > self._maximum_alignment_age_s:
            return

        position_error = tuple(
            estimate.position_fixed_m[index] - truth.position_fixed_m[index]
            for index in range(3)
        )
        truth_velocity_fixed = rotate_vector(
            truth.quaternion_body_to_fixed, truth.velocity_body_mps
        )
        estimate_velocity_fixed = rotate_vector(
            estimate.quaternion_body_to_fixed, estimate.velocity_body_mps
        )
        speed_mps = vector_norm(truth_velocity_fixed)
        along_unit = (
            normalize_vector(truth_velocity_fixed)
            if speed_mps > 1.0e-6
            else (1.0, 0.0, 0.0)
        )
        along_error_m = vector_dot(position_error, along_unit)
        cross_error_vector = tuple(
            position_error[index] - along_error_m * along_unit[index]
            for index in range(3)
        )
        gravity_error_m = vector_dot(position_error, self._gravity_unit_fixed)
        gravity_horizontal_vector = tuple(
            position_error[index]
            - gravity_error_m * self._gravity_unit_fixed[index]
            for index in range(3)
        )
        velocity_error = tuple(
            estimate_velocity_fixed[index] - truth_velocity_fixed[index]
            for index in range(3)
        )

        self.rows.append(
            {
                "estimate_timestamp_s": estimate.timestamp_s,
                "truth_timestamp_s": truth.timestamp_s,
                "alignment_age_s": alignment_age_s,
                "position_error_3d_m": vector_norm(position_error),
                "along_track_error_m": along_error_m,
                "cross_track_error_m": vector_norm(cross_error_vector),
                "gravity_axis_error_m": gravity_error_m,
                "gravity_horizontal_error_m": vector_norm(
                    gravity_horizontal_vector
                ),
                "orientation_error_rad": orientation_error_rad(
                    estimate.quaternion_body_to_fixed,
                    truth.quaternion_body_to_fixed,
                ),
                "velocity_error_mps": vector_norm(velocity_error),
            }
        )


def write_results(output_path: Path, rows: Iterable[dict[str, float]]) -> int:
    """Write aligned rows and return the number written."""
    collected_rows = list(rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(collected_rows[0]) if collected_rows else []
    with output_path.open("w", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        if fieldnames:
            writer.writeheader()
            writer.writerows(collected_rows)
    return len(collected_rows)


def parse_arguments() -> argparse.Namespace:
    """Parse collector command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument(
        "--truth-topic", default="/alpha/localisation/ground_truth/odometry"
    )
    parser.add_argument(
        "--estimate-topic", default="/alpha/localisation/kalman_filter/odometry"
    )
    parser.add_argument("--maximum-alignment-age-s", type=float, default=0.05)
    parser.add_argument("--maximum-truth-samples", type=int, default=2000)
    parser.add_argument("--gravity-x", type=float, required=True)
    parser.add_argument("--gravity-y", type=float, required=True)
    parser.add_argument("--gravity-z", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    if arguments.maximum_alignment_age_s <= 0.0:
        parser.error("--maximum-alignment-age-s must be positive")
    if arguments.maximum_truth_samples <= 0:
        parser.error("--maximum-truth-samples must be positive")
    return arguments


def main() -> None:
    """Run the bounded collector for the configured wall-clock duration."""
    arguments = parse_arguments()
    rclpy.init()
    collector = TrajectoryCollector(
        truth_topic=arguments.truth_topic,
        estimate_topic=arguments.estimate_topic,
        gravity_fixed=(arguments.gravity_x, arguments.gravity_y, arguments.gravity_z),
        maximum_alignment_age_s=arguments.maximum_alignment_age_s,
        maximum_truth_samples=arguments.maximum_truth_samples,
    )
    deadline = time.monotonic() + arguments.duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        row_count = write_results(arguments.output, collector.rows)
        collector.get_logger().info(
            f"wrote {row_count} aligned samples to {arguments.output}"
        )
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
