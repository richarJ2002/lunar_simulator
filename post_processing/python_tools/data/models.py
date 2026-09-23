"""!
@brief  Typed data models for post-processing: every array or record shape
        produced by bag ingestion (python_tools.bag) and diagnostic log
        parsing (python_tools.diagnostics), and consumed by metrics
        (python_tools.data.metrics) and reporting (python_tools.reporting).
        Defining these as dataclasses keeps calculations testable without a
        bag or generated HTML: a metric function or plot constructor takes
        one of these types and never touches rosbag2_py or Plotly directly.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import numpy as np
import numpy.typing as npt

# Six-wheel array ordering used throughout this project (AGENTS.md "C++
# Layout" / "Architecture"): front-left, front-right, centre-left,
# centre-right, rear-left, rear-right. Every six-element wheel array in this
# package follows this order; extractors that build one from a named source
# (e.g. sensor_msgs/JointState, which does not guarantee array order) shall
# resolve each element by name against this tuple rather than trusting
# message order.
WHEEL_ORDER: tuple[str, ...] = (
    "front_left",
    "front_right",
    "centre_left",
    "centre_right",
    "rear_left",
    "rear_right",
)

# Number of wheels Alpha's drivetrain has; used to validate six-wheel arrays
# at extractor boundaries per the plan's "required six-wheel cardinality"
# requirement.
WHEEL_COUNT = len(WHEEL_ORDER)


class ResetReason(Enum):
    """!
    @brief  Reason a time series was split into a new continuous segment.
            Used so plots and metrics never interpolate or compute rates
            across a discontinuity that is not genuine sensor motion.
    """

    # A later timestamp arrived strictly before an earlier one already seen
    # (this also covers a /clock reset, which manifests as exactly this).
    BACKWARD_TIME_JUMP = auto()
    # The gap since the previous sample exceeded --maximum-alignment-gap-s.
    MAXIMUM_GAP_EXCEEDED = auto()
    # An explicit /alpha/localisation/visual/reset event was observed.
    VISUAL_RESET_EVENT = auto()


@dataclass(frozen=True)
class SeriesSegment:
    """!
    @brief  One contiguous, monotonically increasing index range of a time
            series between two discontinuities.
    """

    # Index of the first sample in this segment (inclusive).
    start_index: int
    # Index one past the last sample in this segment (exclusive), so the
    # segment's samples are `series[start_index:end_index]`.
    end_index: int
    # Why the previous segment ended here (absent for the first segment,
    # which begins at the start of the recording rather than a discontinuity).
    reason: Optional[ResetReason]


@dataclass(frozen=True)
class RunMetadata:
    """!
    @brief  Everything post-processing knows about the captured test run as
            a whole, independent of any one topic's contents.
    """

    # Absolute path to the test run directory (e.g. test_runs/2026-09-22-...).
    test_run_dir: Path
    # Absolute path to the rosbag2 bag directory that was read.
    bag_path: Path
    # Gazebo world name used for the run (e.g. "lunar_surface").
    world: str
    # Rover system name used for the run (e.g. "alpha").
    system: str
    # Full shell-quoted launcher command line, if the recording manifest
    # captured one; None for a bag without a manifest (an old run).
    command_line: Optional[str]
    # ROS_DOMAIN_ID the run was recorded under.
    ros_domain_id: Optional[int]
    # GZ_PARTITION the run was recorded under.
    gz_partition: Optional[str]
    # "core" or "core+images", from the recording manifest; None if unknown.
    recording_profile: Optional[str]
    # Git revision the source tree was at when the run was recorded.
    source_revision: Optional[str]
    # Whether the worktree had uncommitted changes at recording time.
    dirty_worktree: Optional[bool]
    # rosbag2 storage identifier ("mcap" or "sqlite3").
    storage_identifier: str
    # Bag start time, elapsed-zero reference, in nanoseconds since epoch.
    start_time_ns: int
    # Bag end time in nanoseconds since epoch.
    end_time_ns: int


@dataclass(frozen=True)
class TopicHealth:
    """!
    @brief  Per-topic recording health, surfaced on every report page so a
            disabled, unpublished or dropped topic is visible rather than
            silently rendered as an empty plot.
    """

    # Fully qualified topic name, e.g. "/alpha/localisation/wheel/odometry".
    topic_name: str
    # Fully qualified ROS message type name, e.g. "nav_msgs/msg/Odometry".
    message_type: str
    # Number of messages successfully deserialized from this topic.
    message_count: int
    # Elapsed seconds (RunMetadata.start_time_ns-relative) of the first
    # message, or None if the topic produced no messages.
    first_stamp_s: Optional[float]
    # Elapsed seconds of the last message, or None if there were none.
    last_stamp_s: Optional[float]
    # Message count divided by the topic's own observed duration, or None
    # if fewer than two messages were recorded.
    effective_rate_hz: Optional[float]
    # Largest inter-message gap observed on this topic, in seconds.
    maximum_gap_s: Optional[float]
    # Number of messages on this topic that failed to deserialize.
    deserialize_failures: int


@dataclass(frozen=True)
class Vector3Series:
    """!
    @brief  A stamped series of 3-vectors (e.g. position, linear/angular
            velocity or acceleration) sharing one frame and physical unit.
    """

    # Elapsed simulation seconds since RunMetadata.start_time_ns, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps backing times_s, shape (N,), retained
    # for alignment and diagnostics per the plan's time contract.
    times_ns: npt.NDArray[np.int64]
    # The 3-vector samples themselves, shape (N, 3).
    values: npt.NDArray[np.float64]
    # Coordinate frame the vectors are expressed in (e.g. "alpha/base_link").
    frame_id: str
    # Physical unit of every component (e.g. "m", "m/s", "rad/s", "m/s^2").
    unit: str
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class QuaternionSeries:
    """!
    @brief  A stamped series of unit quaternions in (x, y, z, w) order,
            matching geometry_msgs/msg/Quaternion field order.
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Quaternion samples in (x, y, z, w) order, shape (N, 4).
    values: npt.NDArray[np.float64]
    # Coordinate frame the orientation is expressed relative to.
    frame_id: str
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class OdometrySeries:
    """!
    @brief  One decoded nav_msgs/msg/Odometry stream: pose in `frame_id`,
            twist in `child_frame_id` (the body frame), per REP-103/105 and
            this project's own convention (AGENTS.md "Local estimators use
            <system>/startup_fixed... twist is expressed in the child body
            frame").
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Parent frame every position/orientation sample is expressed in.
    frame_id: str
    # Body frame every twist sample is expressed in.
    child_frame_id: str
    # Position samples in `frame_id`, metres, shape (N, 3).
    position_m: npt.NDArray[np.float64]
    # Orientation samples in `frame_id`, (x, y, z, w), shape (N, 4).
    orientation_xyzw: npt.NDArray[np.float64]
    # Linear velocity samples in `child_frame_id`, m/s, shape (N, 3).
    linear_velocity_mps: npt.NDArray[np.float64]
    # Angular velocity samples in `child_frame_id`, rad/s, shape (N, 3).
    angular_velocity_radps: npt.NDArray[np.float64]
    # Row-major 6x6 pose covariance per sample, shape (N, 6, 6); may be all
    # zero if the source never populates it (callers must not assume a
    # populated covariance implies a measurement-derived one — see the
    # per-subsystem page notes on which covariances are fixed constants).
    pose_covariance: npt.NDArray[np.float64]
    # Row-major 6x6 twist covariance per sample, shape (N, 6, 6).
    twist_covariance: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class ImuSeries:
    """!
    @brief  One decoded sensor_msgs/msg/Imu stream. Orientation may be
            unset (NaN-filled) for a raw/noisy driver IMU that does not
            estimate attitude; callers shall check `has_orientation` rather
            than assuming every IMU stage carries it.
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Frame the IMU samples are expressed in (typically the sensor frame).
    frame_id: str
    # Whether `orientation_xyzw` carries a real estimate (message field
    # `orientation_covariance[0] != -1`, the sensor_msgs/Imu convention for
    # "orientation not provided").
    has_orientation: bool
    # Orientation samples, (x, y, z, w), shape (N, 4); NaN-filled when
    # `has_orientation` is False.
    orientation_xyzw: npt.NDArray[np.float64]
    # Body angular rate samples, rad/s, shape (N, 3).
    angular_velocity_radps: npt.NDArray[np.float64]
    # Body specific-force/acceleration samples, m/s^2, shape (N, 3). Raw and
    # noisy stages carry gravity in this field; the filtered stage does not
    # (see the inertial-odometry page notes on gravity removal).
    linear_acceleration_mps2: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class JointStateSeries:
    """!
    @brief  A decoded sensor_msgs/msg/JointState stream indexed by resolved
            joint name, never assumed message-array order (WheelOdometryNode
            resolves joints the same way — see findJoint.cc).
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Joint names present in every sample, in the order columns are stored.
    joint_names: tuple[str, ...]
    # Position per joint, rad, shape (N, len(joint_names)).
    position_rad: npt.NDArray[np.float64]
    # Velocity per joint, rad/s, shape (N, len(joint_names)).
    velocity_radps: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class WheelActuatorSeries:
    """!
    @brief  A decoded actuator_msgs/msg/Actuators stream for the six-wheel
            drive/steer command, in WHEEL_ORDER.
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Commanded drive-joint velocity per wheel, rad/s, shape (N, 6).
    velocity_radps: npt.NDArray[np.float64]
    # Commanded steer-joint position per wheel, rad, shape (N, 6).
    position_rad: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class WheelScalarSeries:
    """!
    @brief  A decoded std_msgs/msg/Float64MultiArray stream carrying one
            scalar per wheel in WHEEL_ORDER (slip ratios/observations).
            NaN marks "not observed this cycle" per-wheel, matching
            publishSlipObservation.cc's own convention; callers shall treat
            NaN as absence, not as a zero measurement.
    """

    # Elapsed simulation seconds, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # One scalar per wheel per sample, shape (N, 6); NaN where unobserved.
    values: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class TwistCommandSeries:
    """!
    @brief  A decoded geometry_msgs/msg/Twist stream (Alpha's higher-level
            velocity command topic). Twist messages carry no header/stamp,
            so times derive from the rosbag receive timestamp, per the
            plan's time contract for headerless topics.
    """

    # Elapsed simulation seconds (bag receive time), shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond bag receive timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # Commanded linear velocity, m/s, shape (N, 3).
    linear_mps: npt.NDArray[np.float64]
    # Commanded angular velocity, rad/s, shape (N, 3).
    angular_radps: npt.NDArray[np.float64]
    # Contiguous, discontinuity-free index ranges within this series.
    segments: tuple[SeriesSegment, ...]


@dataclass(frozen=True)
class VisualResetSeries:
    """!
    @brief  Timestamps of std_msgs/msg/Empty events on
            /alpha/localisation/visual/reset. These mark points no series
            may be interpolated across, per the plan's time contract.
    """

    # Elapsed simulation seconds of each reset event, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond bag receive timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]


@dataclass(frozen=True)
class PointCloudSnapshot:
    """!
    @brief  One bounded sensor_msgs/msg/PointCloud2 sample retained in full,
            out of a possibly much larger stream (Phase 2's "bounded number
            of point-cloud snapshots").
    """

    # Elapsed simulation seconds of this snapshot.
    time_s: float
    # Points in this cloud's own frame, metres, shape (width, 3). Width
    # equals the accepted PnP inlier count for an accepted frame, or the
    # full reconstructed correspondence count for a failed/unavailable
    # frame (publishPointCloud.cc / handleStereoCallBack.cc) — callers must
    # read `is_inlier_cloud` to label the count correctly rather than
    # always calling it "inliers".
    points_m: npt.NDArray[np.float64]
    # Frame the points are expressed in.
    frame_id: str
    # Whether `points_m`'s width is the accepted PnP inlier count (True) or
    # the full reconstructed correspondence count from a rejected frame
    # (False).
    is_inlier_cloud: bool


@dataclass(frozen=True)
class VisualPointCloudSeries:
    """!
    @brief  Per-frame point counts across the whole run plus a bounded set
            of retained full snapshots, so memory stays bounded even for a
            long high-rate run (Phase 2's bounded point-cloud requirement).
    """

    # Elapsed simulation seconds of every received cloud, shape (N,).
    times_s: npt.NDArray[np.float64]
    # Original nanosecond timestamps, shape (N,).
    times_ns: npt.NDArray[np.int64]
    # PointCloud2.width of every received cloud, shape (N,).
    point_counts: npt.NDArray[np.int64]
    # Evenly sampled full snapshots, bounded by --maximum-image-frames.
    snapshots: tuple[PointCloudSnapshot, ...]


@dataclass(frozen=True)
class ImageFrame:
    """!
    @brief  One decoded sensor_msgs/msg/Image sample retained for display.
    """

    # Elapsed simulation seconds of this frame.
    time_s: float
    # Decoded RGB pixel data, uint8, shape (height, width, 3). Always RGB
    # regardless of the source encoding — bgr8 sources are channel-swapped
    # at decode time so every consumer only ever handles one layout.
    rgb: npt.NDArray[np.uint8]
    # Original sensor_msgs/Image "encoding" field this frame was decoded
    # from (e.g. "rgb8", "bgr8"), retained for the report's provenance text.
    source_encoding: str


@dataclass(frozen=True)
class ImageFrameSeries:
    """!
    @brief  A bounded, evenly sampled set of decoded images from one image
            topic, plus the full-stream message count and any rejected
            (unsupported-encoding) frame count.
    """

    # Fully qualified source topic name (e.g. "/alpha/drivers/loccam/left").
    topic_name: str
    # Total number of Image messages seen on this topic, before sampling.
    total_message_count: int
    # Number of frames rejected for an unsupported encoding rather than
    # decoded; surfaced as a page warning per the plan.
    rejected_encoding_count: int
    # The evenly sampled, bounded, decoded frames actually retained.
    frames: tuple[ImageFrame, ...]


@dataclass(frozen=True)
class VisualDiagnosticRecord:
    """!
    @brief  One parsed "visual_diag" five-second periodic log record
            (src/localisation/visual_odometry/.../logPipelineDiagnostics.cc).
            These are periodic diagnostics, not per-frame measurements —
            each record summarizes roughly the preceding five seconds.
    """

    # Elapsed seconds since this log file's own first parsed record (from
    # the ROS log entry's own wall-clock timestamp, not a field inside the
    # message). NOT the same axis as any bag-derived plot's elapsed
    # simulation seconds -- RCLCPP_INFO stamps its own console/file output
    # in wall-clock time regardless of a node's use_sim_time setting, and no
    # reliable wall-clock<->simulated-time anchor exists in a
    # --use-sim-time recording to convert one into the other (see the
    # plan's 2026-09-22 audit). Display this on its own explicitly-labeled
    # axis; never overlay it on a bag-elapsed-time plot.
    log_relative_time_s: float
    # Cumulative frames received by the node at this tick.
    received: int
    # Cumulative frames accepted (passed quality gating) at this tick.
    accepted: int
    # Cumulative frames that failed processing at this tick.
    failed: int
    # Instantaneous reception rate, Hz.
    reception_rate_hz: float
    # Instantaneous accepted-frame rate, Hz.
    accepted_rate_hz: float
    # Age of the diagnostic snapshot itself, seconds.
    age_s: float
    # Total per-frame processing time, milliseconds.
    processing_ms: float
    # Stereo image conversion stage time, milliseconds.
    conversion_ms: float
    # Disparity computation stage time, milliseconds.
    disparity_ms: float
    # Feature detection stage time, milliseconds.
    detection_ms: float
    # Feature tracking stage time, milliseconds.
    tracking_ms: float
    # 3-D reconstruction stage time, milliseconds.
    reconstruction_ms: float
    # PnP solve stage time, milliseconds.
    pnp_ms: float
    # Features detected in the most recent frame.
    detected: int
    # Features successfully tracked in the most recent frame.
    tracked: int
    # Features with a valid stereo correspondence in the most recent frame.
    stereo_valid: int
    # 2D-3D correspondences formed in the most recent frame.
    correspondences: int
    # PnP RANSAC inliers in the most recent frame.
    inliers: int
    # Fraction of the feature occupancy grid with at least one inlier.
    occupancy: float
    # Fraction of inliers within the configured near/mid depth band.
    near_mid_ratio: float
    # Disparity 10th/50th/90th percentile, pixels.
    disparity_p10_px: float
    disparity_p50_px: float
    disparity_p90_px: float
    # Depth 10th/50th/90th percentile, metres.
    depth_p10_m: float
    depth_p50_m: float
    depth_p90_m: float
    # Reprojection RMS error of the accepted PnP solution, pixels.
    reprojection_rms_px: float
    # Condition number of the normal-equations matrix used for quality
    # gating (calculateVisualPoseQuality.cc).
    normal_condition: float
    # Interval since the previous accepted frame, seconds.
    accepted_interval_s: float
    # Cumulative consecutive-failure count at this tick.
    consecutive_failures: int


@dataclass(frozen=True)
class LocalisationDiagnosticRecord:
    """!
    @brief  One parsed "localisation_diag" five-second periodic log record
            (alpha_kalman_filter/.../logDiagnostics.cc), for one source
            ("imu", "visual" or "wheel"). The covariance/quaternion/bias
            fields are computed once per tick and repeated identically
            across all three sources' lines in the underlying log — callers
            shall treat them as one filter-wide snapshot rather than
            attributing them to the individual source.
    """

    # Elapsed seconds since this log file's own first parsed record. Same
    # caveat as VisualDiagnosticRecord.log_relative_time_s: NOT the same
    # axis as any bag-derived plot's elapsed simulation seconds.
    log_relative_time_s: float
    # Which measurement source this record's counters describe.
    source: str
    # Cumulative messages received from this source.
    received: int
    # Cumulative messages accepted (fused) from this source.
    accepted: int
    # Cumulative messages rejected for being too old.
    age_rejected: int
    # Cumulative messages rejected by the NIS gate.
    nis_rejected: int
    # Cumulative messages rejected for a numerical failure.
    numerical_rejected: int
    # Cumulative messages actually fused into the filter state.
    fused: int
    # Wall-clock publication rate observed for this source, Hz-equivalent
    # (the raw "publication" field from the log line).
    publication: float
    # Wall-clock admission rate observed for this source.
    admission: float
    # Tick window start time, seconds (estimator clock).
    start: float
    # Tick window end time, seconds (estimator clock).
    end: float
    # Estimator epoch (filter's own internal clock) at this tick, seconds.
    estimator_epoch: float
    # Most recent accepted measurement's normalized innovation squared.
    nis: float
    # Norm of the most recent state correction applied for this source.
    correction_norm: float
    # Trace of the filter's error-state covariance at this tick.
    covariance_trace: float
    # Minimum eigenvalue of the filter's error-state covariance.
    covariance_min_eigenvalue: float
    # Minimum/maximum covariance diagonal entries observed at this tick.
    covariance_diagonal_min: float
    covariance_diagonal_max: float
    # Norm of the filter's nominal-state quaternion (should track 1.0).
    quaternion_norm: float
    # Estimated accelerometer bias, body frame, m/s^2.
    accel_bias_body_mps2: tuple[float, float, float]
    # Estimated gyroscope bias, body frame, rad/s.
    gyro_bias_body_radps: tuple[float, float, float]


@dataclass(frozen=True)
class DiagnosticLog:
    """!
    @brief  All periodic diagnostic records parsed from one node's ROS log
            file for one test run.
    """

    # Path of the log file the records were parsed from.
    log_path: Path
    # Every parsed visual_diag record, in ascending time order.
    visual_records: tuple[VisualDiagnosticRecord, ...] = field(
        default_factory=tuple
    )
    # Every parsed localisation_diag record, in ascending time order.
    localisation_records: tuple[LocalisationDiagnosticRecord, ...] = field(
        default_factory=tuple
    )
    # Number of log lines that matched a diagnostic tag but failed to parse,
    # surfaced as a report warning rather than silently dropped.
    unparsed_line_count: int = 0


@dataclass(frozen=True)
class ParameterSnapshot:
    """!
    @brief  The subset of one node's snapshotted `ros__parameters` this
            report needs, read from the run's own `parameters/` copy (never
            the live workspace tree) so the report reflects what actually
            ran.
    """

    # Path of the YAML file this snapshot was read from.
    source_path: Path
    # Registered ROS node name this snapshot applies to (the YAML top-level
    # key, which AGENTS.md notes is not always the directory name).
    node_name: str
    # The node's ros__parameters mapping, as loaded from YAML.
    parameters: dict[str, object]


@dataclass(frozen=True)
class BagIngestResult:
    """!
    @brief  Everything one single pass over the run's bag produced: bag-level
            timing, per-topic health, and every extracted typed series,
            keyed by the topic name it came from. `python_tools.bag.reader`
            is the only module that constructs this; everything downstream
            only reads it.
    """

    # Elapsed-zero reference: the earliest valid captured timestamp across
    # every registered topic in the bag, nanoseconds since epoch.
    start_time_ns: int
    # The latest valid captured timestamp across every registered topic.
    end_time_ns: int
    # rosbag2 storage identifier the bag itself reports (e.g. "mcap").
    storage_identifier: str
    # Per-topic recording health, including topics with zero messages.
    topic_health: dict[str, TopicHealth]
    # Human-readable issues encountered while reading (an unresolved
    # message type, a malformed point cloud, ...), surfaced as report
    # warnings rather than raised.
    warnings: tuple[str, ...]
    # Extracted series, keyed by source topic name.
    odometry: dict[str, OdometrySeries]
    imu: dict[str, ImuSeries]
    joint_states: dict[str, JointStateSeries]
    wheel_actuators: dict[str, WheelActuatorSeries]
    wheel_scalars: dict[str, WheelScalarSeries]
    twist_commands: dict[str, TwistCommandSeries]
    visual_reset: Optional[VisualResetSeries]
    point_cloud: Optional[VisualPointCloudSeries]
    images: dict[str, ImageFrameSeries]


@dataclass(frozen=True)
class ReportPage:
    """!
    @brief  One fully rendered report page, ready to be written to disk by
            the caller. Kept separate from actually writing the file so a
            page's HTML can be unit-tested without touching the filesystem.
    """

    # Output filename relative to the report's output directory, e.g.
    # "kalman_filter.html".
    filename: str
    # Page title, used both in <title> and the navigation bar.
    title: str
    # The complete, self-contained HTML document for this page.
    html: str
    # Short key/value headline stats for this page's index-page summary
    # card (e.g. {"Position RMSE": "0.12 m", "Duration": "58.3 s"}).
    summary_stats: dict[str, str]
    # Warnings this page's generation produced, echoed on the index page.
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class ReportContext:
    """!
    @brief  Everything one subsystem's `generate_*_report()` function needs:
            the run's identity, the bag ingest result, parsed diagnostics,
            snapshotted parameters, and the resolved CLI options controlling
            report generation. Built once by post_processing.py (or by a
            standalone subsystem script) and passed by reference into every
            report function -- nothing downstream re-reads the bag or logs.
    """

    # The run's identity and bag-level timing.
    run_metadata: RunMetadata
    # The single-pass bag ingest result (series + health + warnings).
    bag: BagIngestResult
    # Parsed periodic diagnostic log records for this run.
    diagnostics: DiagnosticLog
    # Snapshotted node parameters, keyed by registered node name.
    parameters: dict[str, ParameterSnapshot]
    # Directory report pages and shared assets are written into.
    output_dir: Path
    # Largest inter-sample gap, in seconds, still considered one segment
    # and still eligible for ground-truth interpolation.
    maximum_alignment_gap_s: float
    # Largest number of image/point-cloud frames retained per topic.
    maximum_image_frames: int
