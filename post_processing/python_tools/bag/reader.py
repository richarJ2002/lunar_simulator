"""!
@brief  One-pass rosbag2 ingestion. Discovers a test run's bag, iterates it
        exactly once with `rosbag2_py.SequentialReader`, routes each
        registered topic's messages to the matching
        python_tools.data.extractors collector, and finalizes every
        collector into the typed series in python_tools.data.models. An
        unregistered topic's messages are counted but never deserialized,
        per the plan's "route only registered topics" requirement.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from python_tools.bag.timestamps import header_stamp_to_ns, is_valid_timestamp_ns
from python_tools.bag.topic_registry import ALL_TOPICS_BY_NAME, MessageCategory, topic_spec_for
from python_tools.data.extractors import (
    ImageCollector,
    ImuCollector,
    JointStateCollector,
    OdometryCollector,
    PointCloudCollector,
    TwistCommandCollector,
    VisualResetCollector,
    WheelActuatorCollector,
    WheelScalarCollector,
)
from python_tools.data.models import BagIngestResult, TopicHealth

# Default largest inter-sample gap, in seconds, still considered one
# continuous segment. Shared by every entry-point script's --maximum-
# alignment-gap-s default so the aggregate and standalone tools agree.
DEFAULT_MAXIMUM_ALIGNMENT_GAP_S = 1.0

# Default largest number of image/point-cloud frames retained per topic.
DEFAULT_MAXIMUM_IMAGE_FRAMES = 24

# The bag directory name scripts/launch_simulator.sh's start_recorder()
# writes, under a run's ros/bags/ directory.
DEFAULT_BAG_NAME = "localisation"


class BagValidationError(Exception):
    """!
    @brief  Raised for an actionable, user-facing problem with a test run
            or bag path, distinct from an unexpected internal failure.
    """


def discover_bag_path(test_run_dir: Path, explicit_bag_path: Optional[Path]) -> Path:
    """!
    @brief   Resolves the bag to read: an explicitly supplied path, or the
             run's standard default bag.

    @param   test_run_dir
             The test run directory (e.g. test_runs/<run>).
    @param   explicit_bag_path
             A caller-supplied `--bag` path, or `None` to discover it.

    @return  The resolved bag directory path.

    @throws  BagValidationError
             If no bag can be found, or the discovered/supplied path is not
             a valid rosbag2 bag directory (no metadata.yaml).
    """
    # An explicitly supplied path is used as-is, still validated below.
    if explicit_bag_path is not None:
        bag_path = explicit_bag_path
    else:
        # The standard location start_recorder() writes to.
        bag_path = test_run_dir / "ros" / "bags" / DEFAULT_BAG_NAME
        if not bag_path.is_dir():
            raise BagValidationError(
                f"No default telemetry bag found at {bag_path}. This looks "
                "like an old run captured before automatic recording was "
                "added, or the recorder failed to start; pass --bag "
                "explicitly if the bag lives elsewhere."
            )
    if not (bag_path / "metadata.yaml").is_file():
        raise BagValidationError(
            f"{bag_path} is not a valid rosbag2 bag: no metadata.yaml. The "
            "recording may not have shut down cleanly (metadata.yaml is "
            "only written on a finalized recorder shutdown)."
        )
    return bag_path


def validate_test_run_dir(test_run_dir: Path) -> None:
    """!
    @brief   Validates that a directory has the shape of a captured test
             run before anything tries to read from it.

    @param   test_run_dir
             The candidate test run directory.

    @return  None

    @throws  BagValidationError
             If `test_run_dir` does not exist, is not a directory, or is
             missing its `parameters/` or `ros/` subdirectories.
    """
    if not test_run_dir.is_dir():
        raise BagValidationError(f"--test-run is not a directory: {test_run_dir}")
    # Both subdirectories are always created by create_test_run() in
    # scripts/launch_simulator.sh; their absence means this is not a test
    # run directory at all.
    for required_subdirectory in ("parameters", "ros"):
        if not (test_run_dir / required_subdirectory).is_dir():
            raise BagValidationError(
                f"--test-run is missing {required_subdirectory}/: "
                f"{test_run_dir} does not look like a captured test run"
            )


def _resolve_message_class(message_type: str) -> object:
    """!
    @brief   Resolves a fully qualified ROS message type name to its
             Python message class.

    @param   message_type
             The type name as stored in the bag, e.g. "nav_msgs/msg/Odometry".

    @return  The resolved message class.

    @throws  BagValidationError
             If the type cannot be resolved (the message package is not
             installed in the sourced environment).
    """
    try:
        return get_message(message_type)
    except (ModuleNotFoundError, AttributeError, ValueError) as error:
        raise BagValidationError(
            f"ROS message type '{message_type}' is not available in the "
            "sourced environment (source install/setup.bash and "
            "/opt/ros/jazzy/setup.bash before running post-processing): "
            f"{error}"
        ) from error


def read_bag(
    bag_path: Path,
    maximum_alignment_gap_s: float = DEFAULT_MAXIMUM_ALIGNMENT_GAP_S,
    maximum_image_frames: int = DEFAULT_MAXIMUM_IMAGE_FRAMES,
) -> BagIngestResult:
    """!
    @brief   Reads one bag exactly once, extracting every registered
             topic's messages into typed series.

    @param   bag_path
             A validated rosbag2 bag directory (see `discover_bag_path`).
    @param   maximum_alignment_gap_s
             Largest inter-sample gap still considered one segment.
    @param   maximum_image_frames
             Largest number of image/point-cloud frames retained per topic.

    @return  The assembled `BagIngestResult`.

    @throws  BagValidationError
             If the bag cannot be opened at all.
    """
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open_uri(str(bag_path))
    except RuntimeError as error:
        raise BagValidationError(f"Unable to open bag {bag_path}: {error}") from error
    # Captured now: get_metadata() is unavailable once the reader closes.
    storage_identifier = reader.get_metadata().storage_identifier

    # Discover which topics are actually present and what type each one
    # was recorded as, so a registered topic that was simply never
    # published still gets a zero-message TopicHealth entry.
    present_topics = {
        metadata.name: metadata.type
        for metadata in reader.get_all_topics_and_types()
    }

    warnings: list[str] = []
    # One collector per registered, present topic; keyed by topic name so
    # multiple topics of the same category (e.g. two Odometry topics) never
    # collide.
    collectors: dict[str, object] = {}
    message_classes: dict[str, object] = {}
    for topic_name, message_type in present_topics.items():
        spec = topic_spec_for(topic_name)
        # An unregistered topic (future/foreign data in the bag) is left
        # entirely alone -- not deserialized, not health-tracked.
        if spec is None:
            continue
        if spec.message_type != message_type:
            warnings.append(
                f"{topic_name} was recorded as {message_type}, expected "
                f"{spec.message_type}; skipping"
            )
            continue
        try:
            message_classes[topic_name] = _resolve_message_class(message_type)
        except BagValidationError as error:
            warnings.append(str(error))
            continue
        collectors[topic_name] = _make_collector(spec.category, topic_name, maximum_image_frames)

    # Per-topic message counters, built up during the single pass and
    # finalized into TopicHealth after the loop.
    message_counts: dict[str, int] = {name: 0 for name in collectors}
    first_stamp_ns: dict[str, Optional[int]] = {name: None for name in collectors}
    last_stamp_ns: dict[str, Optional[int]] = {name: None for name in collectors}
    maximum_gap_ns: dict[str, int] = {name: 0 for name in collectors}
    deserialize_failures: dict[str, int] = {name: 0 for name in collectors}

    # Tracks the earliest valid captured timestamp across every registered
    # topic, used as the whole run's elapsed-zero reference.
    run_start_time_ns: Optional[int] = None
    run_end_time_ns: Optional[int] = None

    while reader.has_next():
        topic_name, data, receive_time_ns = reader.read_next()
        if topic_name not in collectors:
            continue
        message_counts[topic_name] += 1
        spec = topic_spec_for(topic_name)
        try:
            msg = deserialize_message(data, message_classes[topic_name])
        except Exception:  # noqa: BLE001 -- a corrupt message must not abort the whole run
            deserialize_failures[topic_name] += 1
            continue

        # Route to the right collector, and compute the sample's own
        # timestamp for health tracking (header stamp for stamped types,
        # bag receive time for headerless ones), per the time contract.
        accepted = True
        sample_time_ns = receive_time_ns
        # True for every category with a real header.stamp; determines
        # whether sample_time_ns comes from the header stamp or bag receive
        # time, per the time contract.
        is_header_stamped = spec.category in (
            MessageCategory.ODOMETRY,
            MessageCategory.IMU,
            MessageCategory.JOINT_STATE,
            MessageCategory.WHEEL_ACTUATOR_COMMAND,
            MessageCategory.POINT_CLOUD,
            MessageCategory.IMAGE,
        )
        if is_header_stamped:
            sample_time_ns = header_stamp_to_ns(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            )

        if spec.category is MessageCategory.ODOMETRY:
            collectors[topic_name].append(msg)
        elif spec.category is MessageCategory.IMU:
            collectors[topic_name].append(msg)
        elif spec.category is MessageCategory.JOINT_STATE:
            accepted = collectors[topic_name].append(msg)
        elif spec.category is MessageCategory.WHEEL_ACTUATOR_COMMAND:
            accepted = collectors[topic_name].append(msg)
        elif spec.category is MessageCategory.WHEEL_SCALAR:
            collectors[topic_name].append(msg, receive_time_ns)
        elif spec.category is MessageCategory.TWIST_COMMAND:
            collectors[topic_name].append(msg, receive_time_ns)
        elif spec.category is MessageCategory.VISUAL_RESET:
            collectors[topic_name].append(receive_time_ns)
        elif spec.category is MessageCategory.POINT_CLOUD:
            collectors[topic_name].append(msg, sample_time_ns)
        elif spec.category is MessageCategory.IMAGE:
            collectors[topic_name].append(msg)
        # MessageCategory.CLOCK carries no per-message extraction; only its
        # health (message count/rate) is tracked, using bag receive time.

        if not accepted:
            deserialize_failures[topic_name] += 1
            continue

        # A structurally valid message with an unset/invalid timestamp
        # (e.g. a never-stamped header) is still recorded by the collector
        # above -- its own finalize() will drop it from the final series --
        # but it must not corrupt this topic's health stats or the whole
        # run's elapsed-zero reference the way a real capture time would.
        if not is_valid_timestamp_ns(sample_time_ns):
            continue

        # Update this topic's health bookkeeping with the sample's time.
        if first_stamp_ns[topic_name] is None:
            first_stamp_ns[topic_name] = sample_time_ns
        else:
            gap_ns = sample_time_ns - (last_stamp_ns[topic_name] or sample_time_ns)
            if gap_ns > maximum_gap_ns[topic_name]:
                maximum_gap_ns[topic_name] = gap_ns
        last_stamp_ns[topic_name] = sample_time_ns

        # Update the whole-run elapsed-zero/end reference.
        if run_start_time_ns is None or sample_time_ns < run_start_time_ns:
            run_start_time_ns = sample_time_ns
        if run_end_time_ns is None or sample_time_ns > run_end_time_ns:
            run_end_time_ns = sample_time_ns

    reader.close() if hasattr(reader, "close") else None

    # A bag that opened but produced no valid samples on any registered
    # topic cannot be normalized to elapsed seconds at all.
    if run_start_time_ns is None:
        raise BagValidationError(
            f"Bag {bag_path} contains no valid messages on any recognized "
            "topic; nothing to report"
        )

    # Assemble TopicHealth for every registered topic, including ones that
    # were present but produced zero valid samples.
    topic_health: dict[str, TopicHealth] = {}
    for topic_name in collectors:
        spec = topic_spec_for(topic_name)
        count = message_counts[topic_name]
        first_s = (
            (first_stamp_ns[topic_name] - run_start_time_ns) / 1e9
            if first_stamp_ns[topic_name] is not None
            else None
        )
        last_s = (
            (last_stamp_ns[topic_name] - run_start_time_ns) / 1e9
            if last_stamp_ns[topic_name] is not None
            else None
        )
        duration_s = (last_s - first_s) if (first_s is not None and last_s is not None) else None
        effective_rate_hz = (
            (count - 1) / duration_s
            if duration_s and duration_s > 0 and count > 1
            else None
        )
        topic_health[topic_name] = TopicHealth(
            topic_name=topic_name,
            message_type=spec.message_type,
            message_count=count,
            first_stamp_s=first_s,
            last_stamp_s=last_s,
            effective_rate_hz=effective_rate_hz,
            maximum_gap_s=(maximum_gap_ns[topic_name] / 1e9) if count > 1 else None,
            deserialize_failures=deserialize_failures[topic_name],
        )
    # Every registered core/image topic this tool knows about is reported,
    # even when it was never present in the bag at all (a fully disabled
    # or never-published topic), so its absence is visible rather than
    # silently omitted from the report.
    for topic_name, spec in ALL_TOPICS_BY_NAME.items():
        if topic_name not in topic_health:
            topic_health[topic_name] = TopicHealth(
                topic_name=topic_name,
                message_type=spec.message_type,
                message_count=0,
                first_stamp_s=None,
                last_stamp_s=None,
                effective_rate_hz=None,
                maximum_gap_s=None,
                deserialize_failures=0,
            )

    # Finalize every collector into its typed series now that the true
    # run start time is known.
    odometry: dict[str, object] = {}
    imu: dict[str, object] = {}
    joint_states: dict[str, object] = {}
    wheel_actuators: dict[str, object] = {}
    wheel_scalars: dict[str, object] = {}
    twist_commands: dict[str, object] = {}
    visual_reset = None
    point_cloud = None
    images: dict[str, object] = {}
    for topic_name, collector in collectors.items():
        spec = topic_spec_for(topic_name)
        if spec.category is MessageCategory.ODOMETRY:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                odometry[topic_name] = series
        elif spec.category is MessageCategory.IMU:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                imu[topic_name] = series
        elif spec.category is MessageCategory.JOINT_STATE:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                joint_states[topic_name] = series
        elif spec.category is MessageCategory.WHEEL_ACTUATOR_COMMAND:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                wheel_actuators[topic_name] = series
        elif spec.category is MessageCategory.WHEEL_SCALAR:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                wheel_scalars[topic_name] = series
        elif spec.category is MessageCategory.TWIST_COMMAND:
            series = collector.finalize(run_start_time_ns, maximum_alignment_gap_s)
            if series is not None:
                twist_commands[topic_name] = series
        elif spec.category is MessageCategory.VISUAL_RESET:
            visual_reset = collector.finalize(run_start_time_ns)
        elif spec.category is MessageCategory.POINT_CLOUD:
            point_cloud = collector.finalize(run_start_time_ns)
        elif spec.category is MessageCategory.IMAGE:
            images[topic_name] = collector.finalize(run_start_time_ns)

    return BagIngestResult(
        start_time_ns=run_start_time_ns,
        end_time_ns=run_end_time_ns if run_end_time_ns is not None else run_start_time_ns,
        storage_identifier=storage_identifier,
        topic_health=topic_health,
        warnings=tuple(warnings),
        odometry=odometry,
        imu=imu,
        joint_states=joint_states,
        wheel_actuators=wheel_actuators,
        wheel_scalars=wheel_scalars,
        twist_commands=twist_commands,
        visual_reset=visual_reset,
        point_cloud=point_cloud,
        images=images,
    )


def _make_collector(
    category: MessageCategory, topic_name: str, maximum_image_frames: int
) -> object:
    """!
    @brief   Constructs the right collector instance for a topic's message
             category.

    @param   category
             The topic's registered `MessageCategory`.
    @param   topic_name
             The topic name, used only by image collectors for provenance.
    @param   maximum_image_frames
             Largest number of frames retained, for image/point-cloud
             collectors.

    @return  A newly constructed collector instance.
    """
    if category is MessageCategory.ODOMETRY:
        return OdometryCollector()
    if category is MessageCategory.IMU:
        return ImuCollector()
    if category is MessageCategory.JOINT_STATE:
        return JointStateCollector()
    if category is MessageCategory.WHEEL_ACTUATOR_COMMAND:
        return WheelActuatorCollector()
    if category is MessageCategory.WHEEL_SCALAR:
        return WheelScalarCollector()
    if category is MessageCategory.TWIST_COMMAND:
        return TwistCommandCollector()
    if category is MessageCategory.VISUAL_RESET:
        return VisualResetCollector()
    if category is MessageCategory.POINT_CLOUD:
        return PointCloudCollector(maximum_image_frames)
    if category is MessageCategory.IMAGE:
        return ImageCollector(topic_name, maximum_image_frames)
    # MessageCategory.CLOCK: health-only, no extraction collector needed;
    # reader.py still needs a placeholder so the topic is loop-tracked.
    return _NullCollector()


class _NullCollector:
    """!
    @brief  A no-op collector for topics that are only health-tracked
            (currently just /clock), so the reader's routing loop does not
            need a special case for "no extraction happens here".
    """

    def append(self, *_args: object, **_kwargs: object) -> bool:
        """!
        @brief   Discards the message; only called for its side effect of
                 letting the reader's health bookkeeping run.

        @return  Always `True` (nothing to reject).
        """
        return True

    def finalize(self, *_args: object, **_kwargs: object) -> None:
        """!
        @brief   No series is produced for a health-only topic.

        @return  Always `None`.
        """
        return None


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone bag-summary mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Print per-topic message counts and rates for one captured bag."
    )
    parser.add_argument(
        "test_run_dir", type=Path, help="Test run directory (e.g. test_runs/<run>)."
    )
    parser.add_argument(
        "--bag",
        type=Path,
        default=None,
        help="Explicit bag path, overriding the run's default bag location.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints per-topic health for one captured bag.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success, `1` on a validation error.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    try:
        validate_test_run_dir(args.test_run_dir)
        bag_path = discover_bag_path(args.test_run_dir, args.bag)
        result = read_bag(bag_path)
    except BagValidationError as error:
        print(f"error: {error}")
        return 1
    for topic_name, health in sorted(result.topic_health.items()):
        rate = f"{health.effective_rate_hz:.2f} Hz" if health.effective_rate_hz else "n/a"
        print(f"{topic_name}: {health.message_count} msgs, {rate}")
    if result.warnings:
        print("warnings:")
        for warning in result.warnings:
            print(f"  {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
