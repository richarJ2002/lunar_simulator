"""!
@brief  The single source of truth, on the Python side, for which topics
        post-processing understands, what ROS message type and extraction
        category each one carries, and whether it belongs to the always-on
        "core" recording profile or the optional "--record-images" profile.
        scripts/launch_simulator.sh keeps its own topic list in sync with
        this one by literal string (see that script's CORE_RECORDING_TOPICS/
        IMAGE_RECORDING_TOPICS comments) so that recording works even in an
        environment where this Python package's dependencies are not
        installed; python_tools.bag.reader is what actually depends on this
        module to route deserialized messages to the right extractor.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Sequence


class MessageCategory(Enum):
    """!
    @brief  Which typed extractor (python_tools.data.extractors) a topic's
            messages shall be routed to, independent of its exact ROS type
            string.
    """

    # rosgraph_msgs/msg/Clock: the simulated-time reference, not extracted
    # into its own series but used to validate simulated-time coverage.
    CLOCK = auto()
    # nav_msgs/msg/Odometry: pose + twist (+ optional covariance).
    ODOMETRY = auto()
    # sensor_msgs/msg/Imu: orientation (optional) + angular rate + accel.
    IMU = auto()
    # sensor_msgs/msg/JointState: named position/velocity per joint.
    JOINT_STATE = auto()
    # actuator_msgs/msg/Actuators: six-wheel drive/steer command.
    WHEEL_ACTUATOR_COMMAND = auto()
    # geometry_msgs/msg/Twist: the higher-level body velocity command.
    TWIST_COMMAND = auto()
    # std_msgs/msg/Float64MultiArray: one scalar per wheel, in WHEEL_ORDER.
    WHEEL_SCALAR = auto()
    # sensor_msgs/msg/PointCloud2: visual odometry's reconstructed points.
    POINT_CLOUD = auto()
    # std_msgs/msg/Empty: a visual-odometry reset event marker.
    VISUAL_RESET = auto()
    # sensor_msgs/msg/Image: a camera or annotated-feature frame.
    IMAGE = auto()


@dataclass(frozen=True)
class TopicSpec:
    """!
    @brief  Everything post-processing needs to know about one recorded
            topic before it has read a single message from it.
    """

    # Fully qualified topic name, e.g. "/alpha/localisation/wheel/odometry".
    topic_name: str
    # Fully qualified ROS message type name, e.g. "nav_msgs/msg/Odometry".
    message_type: str
    # Which extractor category this topic's messages shall be routed to.
    category: MessageCategory
    # True if this topic is recorded by every run (the "core" profile);
    # False if it is only recorded when --record-images was passed.
    core: bool


# The always-recorded core telemetry topics, matching
# scripts/launch_simulator.sh's CORE_RECORDING_TOPICS array exactly. Keep
# both lists in sync by hand if either changes; see this module's docstring
# for why they are not shared at runtime.
CORE_TOPICS: tuple[TopicSpec, ...] = (
    TopicSpec("/clock", "rosgraph_msgs/msg/Clock", MessageCategory.CLOCK, True),
    TopicSpec(
        "/alpha/drivers/ground_truth/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/ground_truth/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/drivers/imu", "sensor_msgs/msg/Imu", MessageCategory.IMU, True
    ),
    TopicSpec("/alpha/imu", "sensor_msgs/msg/Imu", MessageCategory.IMU, True),
    TopicSpec(
        "/alpha/localisation/inertial/filtered_imu",
        "sensor_msgs/msg/Imu",
        MessageCategory.IMU,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/inertial/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/drivers/joint_states",
        "sensor_msgs/msg/JointState",
        MessageCategory.JOINT_STATE,
        True,
    ),
    TopicSpec(
        "/alpha/joint_states",
        "sensor_msgs/msg/JointState",
        MessageCategory.JOINT_STATE,
        True,
    ),
    TopicSpec(
        "/alpha/control/cmd/velocity",
        "geometry_msgs/msg/Twist",
        MessageCategory.TWIST_COMMAND,
        True,
    ),
    TopicSpec(
        "/alpha/control/cmd/wheel_joint_states",
        "actuator_msgs/msg/Actuators",
        MessageCategory.WHEEL_ACTUATOR_COMMAND,
        True,
    ),
    TopicSpec(
        "/alpha/drivers/cmd/wheel_joint_states",
        "actuator_msgs/msg/Actuators",
        MessageCategory.WHEEL_ACTUATOR_COMMAND,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/wheel/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/wheel/slip_ratios",
        "std_msgs/msg/Float64MultiArray",
        MessageCategory.WHEEL_SCALAR,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/wheel/slip_observation",
        "std_msgs/msg/Float64MultiArray",
        MessageCategory.WHEEL_SCALAR,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/visual/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/visual/point_cloud",
        "sensor_msgs/msg/PointCloud2",
        MessageCategory.POINT_CLOUD,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/visual/reset",
        "std_msgs/msg/Empty",
        MessageCategory.VISUAL_RESET,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/kalman_filter/odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/localisation/kalman_filter/wheel_slip_ratio",
        "std_msgs/msg/Float64MultiArray",
        MessageCategory.WHEEL_SCALAR,
        True,
    ),
)

# The optional image topics, only present when a run was captured with
# --record-images, matching launch_simulator.sh's IMAGE_RECORDING_TOPICS.
IMAGE_TOPICS: tuple[TopicSpec, ...] = (
    TopicSpec(
        "/alpha/drivers/loccam/left",
        "sensor_msgs/msg/Image",
        MessageCategory.IMAGE,
        False,
    ),
    TopicSpec(
        "/alpha/drivers/loccam/right",
        "sensor_msgs/msg/Image",
        MessageCategory.IMAGE,
        False,
    ),
    TopicSpec(
        "/alpha/localisation/visual/features",
        "sensor_msgs/msg/Image",
        MessageCategory.IMAGE,
        False,
    ),
)

# Every topic this tool understands, core and optional combined, keyed by
# topic name for O(1) lookup during bag ingestion.
ALL_TOPICS_BY_NAME: dict[str, TopicSpec] = {
    spec.topic_name: spec for spec in (*CORE_TOPICS, *IMAGE_TOPICS)
}


def topic_spec_for(topic_name: str) -> Optional[TopicSpec]:
    """!
    @brief   Looks up the registered spec for a recorded topic name.

    @param   topic_name
             Fully qualified topic name as stored in the bag.

    @return  The topic's `TopicSpec`, or `None` if this tool does not
             recognize the topic (an unrelated or future topic present in
             the bag shall be ignored, per the plan's "route only registered
             topics" requirement, not treated as an error).
    """
    # A plain dict lookup is sufficient since topic names are unique.
    return ALL_TOPICS_BY_NAME.get(topic_name)


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone registry-listing mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    # Describe the tool so `-h` output is self-explanatory.
    parser = argparse.ArgumentParser(
        description=(
            "List the topics post-processing understands for a given "
            "recording profile, one per line."
        )
    )
    # Optional flag selecting which profile's topics to print.
    parser.add_argument(
        "--profile",
        choices=("core", "core+images"),
        default="core",
        help="Recording profile to list topics for (default: core).",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the registered topic list for a given
             recording profile, for manual inspection against
             scripts/launch_simulator.sh's own topic arrays.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    # Select the core topics, plus the image topics when requested.
    topics = list(CORE_TOPICS)
    if args.profile == "core+images":
        topics.extend(IMAGE_TOPICS)

    # Print one topic name per line, in registration order.
    for spec in topics:
        print(spec.topic_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
