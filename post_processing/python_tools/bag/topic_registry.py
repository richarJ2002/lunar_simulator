"""!
@brief  The single source of truth, on the Python side, for which topics
        post-processing understands, what ROS message type and extraction
        category each one carries, and whether it belongs to the always-on
        "core" recording profile or the optional "--record-images" profile.
        scripts/launch_simulator.sh keeps its own topic list in sync with
        this one by literal string (see that script's CORE_RECORD_SUFFIXES/
        IMAGE_RECORD_SUFFIXES comments) so that recording works even in an
        environment where this Python package's dependencies are not
        installed; python_tools.bag.reader is what actually depends on this
        module to route deserialized messages to the right extractor.
        Every registered topic except /clock lives under /<system>/: the
        /alpha/... names below are the default system's registration, and
        any other system's same-suffix topic resolves through them.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Sequence

# Rover system whose /<system>/... topics the registered names below use.
# "alpha" is the default-system example; old bags (captured before the
# recording manifest named a system) keep reading through it.
DEFAULT_SYSTEM = "alpha"

# Manifest system value for runs captured before automatic recording
# existed; always resolves to DEFAULT_SYSTEM.
UNKNOWN_SYSTEM = "unknown"


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
    # diagnostic_msgs/msg/DiagnosticArray: named key/value status records.
    DIAGNOSTIC_ARRAY = auto()


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
# scripts/launch_simulator.sh's assembled core set exactly for the default
# system. Keep both lists in sync by hand if either changes; see this
# module's docstring for why they are not shared at runtime.
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
    TopicSpec(
        "/alpha/control/filtered_odometry",
        "nav_msgs/msg/Odometry",
        MessageCategory.ODOMETRY,
        True,
    ),
    TopicSpec(
        "/alpha/diagnostics",
        "diagnostic_msgs/msg/DiagnosticArray",
        MessageCategory.DIAGNOSTIC_ARRAY,
        True,
    ),
    TopicSpec(
        "/alpha/system/state",
        "diagnostic_msgs/msg/DiagnosticArray",
        MessageCategory.DIAGNOSTIC_ARRAY,
        True,
    ),
)

# The optional image topics, only present when a run was captured with
# --record-images, matching launch_simulator.sh's assembled image set for
# the default system.
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

# Suffix (the topic name without its /<system>/ prefix) to the registered
# message type, category, and profile flag, for every namespaced topic.
# /clock has no system prefix and so has no suffix entry.
_SUFFIX_SPECS: dict[str, tuple[str, MessageCategory, bool]] = {}
for _registered_spec in (*CORE_TOPICS, *IMAGE_TOPICS):
    _parts = _registered_spec.topic_name.split("/", 2)
    if len(_parts) == 3:
        _SUFFIX_SPECS.setdefault(
            _parts[2],
            (
                _registered_spec.message_type,
                _registered_spec.category,
                _registered_spec.core,
            ),
        )


def normalize_system(system_name: Optional[str]) -> str:
    """!
    @brief   Resolves a run's system name to the topic namespace to read.

    @param   system_name
             The recording manifest's system value, or `None` when no
             manifest was captured.

    @return  `system_name` itself, or `DEFAULT_SYSTEM` when it is missing
              or `UNKNOWN_SYSTEM` (an old run from before automatic
              recording named a system).
    """
    if not system_name or system_name == UNKNOWN_SYSTEM:
        return DEFAULT_SYSTEM
    return system_name


def topic_name_for(system_name: Optional[str], suffix: str) -> str:
    """!
    @brief   Builds one fully qualified topic name for a rover system.

    @param   system_name
             The recording manifest's system value, resolved through
             `normalize_system` (old runs read via the default system).
    @param   suffix
             The topic name without its /<system>/ prefix, e.g.
             "localisation/wheel/odometry".

    @return  The fully qualified topic name, e.g.
              "/alpha/localisation/wheel/odometry".
    """
    return f"/{normalize_system(system_name)}/{suffix}"


def topic_spec_for(topic_name: str) -> Optional[TopicSpec]:
    """!
    @brief   Looks up the registered spec for a recorded topic name.

    @param   topic_name
              Fully qualified topic name as stored in the bag.

    @return  The topic's `TopicSpec`, or `None` if this tool does not
              recognize the topic (an unrelated or future topic present in
              the bag shall be ignored, per the plan's "route only registered
              topics" requirement, not treated as an error). A topic under
              any other system's namespace resolves through its suffix, so
              a /beta/... bag routes exactly like the registered /alpha/...
              one; only the suffix, never the system name, decides.
    """
    # A plain dict lookup is sufficient since topic names are unique.
    spec = ALL_TOPICS_BY_NAME.get(topic_name)
    if spec is not None:
        return spec
    parts = topic_name.split("/", 2)
    if len(parts) != 3:
        return None
    known = _SUFFIX_SPECS.get(parts[2])
    if known is None:
        return None
    message_type, category, core = known
    return TopicSpec(topic_name, message_type, category, core)


def topics_for_profile(profile: str, system_name: Optional[str] = None) -> list[TopicSpec]:
    """!
    @brief   Lists the registered topics for a recording profile and system.

    @param   profile
             "core", or "core+images" to include the optional image topics.
    @param   system_name
             The rover system, resolved through `normalize_system` (old
             runs list via the default system).

    @return  The profile's topic specs, in registration order.
    """
    specs = list(CORE_TOPICS)
    if profile == "core+images":
        specs.extend(IMAGE_TOPICS)
    if normalize_system(system_name) == DEFAULT_SYSTEM:
        return specs
    renamed: list[TopicSpec] = []
    for spec in specs:
        parts = spec.topic_name.split("/", 2)
        if len(parts) == 3:
            renamed.append(
                TopicSpec(
                    topic_name_for(system_name, parts[2]),
                    spec.message_type,
                    spec.category,
                    spec.core,
                )
            )
        else:
            renamed.append(spec)
    return renamed


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
    # Optional rover system selecting the /<system>/... topic namespace.
    parser.add_argument(
        "--system",
        default=DEFAULT_SYSTEM,
        help=f"Rover system to list topics for (default: {DEFAULT_SYSTEM}).",
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

    # Select the core topics, plus the image topics when requested, under
    # the requested system's namespace.
    topics = topics_for_profile(args.profile, args.system)

    # Print one topic name per line, in registration order.
    for spec in topics:
        print(spec.topic_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
