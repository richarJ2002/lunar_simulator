"""!
@brief  Typed, boundary-validating extractors that turn one topic's stream
        of deserialized ROS messages into the NumPy-array dataclasses in
        python_tools.data.models. Each `*Collector` accumulates raw field
        values message-by-message during python_tools.bag.reader's single
        bag pass, then `finalize()`s into the typed series once the run's
        elapsed-time reference is known. Every collector except
        `ImageCollector`/`PointCloudCollector` never retains a ROS message
        object after its own fields have been copied out. Those two decode
        a bounded, evenly-spaced sample via `_BoundedEvenSampler` rather
        than decoding and retaining every frame during the single bag
        pass (see the plan's 2026-09-22 audit, Defect 2): each holds at
        most `2 * maximum_frames` decoded frames plus one undecoded raw
        message reference (the current "last eligible" candidate, replaced
        on every later message and decoded at most once, in `finalize()`)
        -- never the full stream. Pure Python/NumPy only -- no rosbag2_py
        or ROS node access here, so these are unit-testable against
        synthetic messages without a bag.
"""

from __future__ import annotations

import argparse
from typing import Callable, Optional, Sequence

import numpy as np
import numpy.typing as npt

from python_tools.bag.timestamps import (
    ORIENTATION_UNAVAILABLE_COVARIANCE_MARKER,
    elapsed_seconds,
    filter_finite_timestamps,
    header_stamp_to_ns,
    is_valid_timestamp_ns,
    segment_series,
)
from python_tools.data.models import (
    WHEEL_COUNT,
    WHEEL_ORDER,
    ImageFrame,
    ImageFrameSeries,
    ImuSeries,
    JointStateSeries,
    OdometrySeries,
    PointCloudSnapshot,
    TwistCommandSeries,
    VisualPointCloudSeries,
    VisualResetSeries,
    WheelActuatorSeries,
    WheelScalarSeries,
)

# sensor_msgs/msg/Image encodings this tool can decode to RGB without
# cv_bridge. Anything else is rejected with a page warning per the plan.
SUPPORTED_IMAGE_ENCODINGS = ("rgb8", "bgr8", "mono8")

# Bytes per pixel for each supported encoding, used to validate `step`
# against `width` before indexing into the raw image buffer.
_BYTES_PER_PIXEL = {"rgb8": 3, "bgr8": 3, "mono8": 1}


def _filter_and_mask(
    stamps_ns: Sequence[int], *parallel_arrays: npt.NDArray
) -> tuple[npt.NDArray[np.int64], tuple[npt.NDArray, ...]]:
    """!
    @brief   Converts a collector's raw per-message timestamp list to a
             sorted-by-validity int64 array and applies the same
             finite-timestamp mask to every parallel per-message array, per
             the plan's "reject non-finite timestamps" requirement.

    @param   stamps_ns
             Raw per-message timestamps, one per collected sample.
    @param   parallel_arrays
             Any number of arrays whose first axis is one entry per sample,
             already in the same order as `stamps_ns`.

    @return  The validated timestamp array and a tuple of the same
             parallel arrays, each filtered to the valid-timestamp subset.
    """
    # Build the raw timestamp array first so the validity mask can be
    # computed once and reused for every parallel array.
    times_ns = np.array(stamps_ns, dtype=np.int64)
    valid_mask = filter_finite_timestamps(times_ns)
    # Apply the same mask to the timestamps and every parallel array so
    # every returned array stays aligned index-for-index.
    filtered_times = times_ns[valid_mask]
    filtered_arrays = tuple(array[valid_mask] for array in parallel_arrays)
    return filtered_times, filtered_arrays


class OdometryCollector:
    """!
    @brief  Accumulates nav_msgs/msg/Odometry samples from one topic during
            a single bag pass.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        # Per-message header stamp, nanoseconds since epoch.
        self._stamps_ns: list[int] = []
        # Per-message position, metres, in the message's own frame_id.
        self._positions: list[tuple[float, float, float]] = []
        # Per-message orientation, (x, y, z, w).
        self._orientations: list[tuple[float, float, float, float]] = []
        # Per-message body-frame linear velocity, m/s.
        self._linear_velocities: list[tuple[float, float, float]] = []
        # Per-message body-frame angular velocity, rad/s.
        self._angular_velocities: list[tuple[float, float, float]] = []
        # Per-message flattened 36-entry pose/twist covariance rows.
        self._pose_covariances: list[npt.NDArray[np.float64]] = []
        self._twist_covariances: list[npt.NDArray[np.float64]] = []
        # Frame IDs are latched from the first message; every producer in
        # this project publishes a fixed frame_id/child_frame_id per topic.
        self._frame_id: Optional[str] = None
        self._child_frame_id: Optional[str] = None

    def append(self, msg: object) -> None:
        """!
        @brief   Records one deserialized nav_msgs/msg/Odometry message.

        @param   msg
                 The deserialized message.

        @return  None
        """
        # Combine the header stamp into one nanosecond timestamp.
        self._stamps_ns.append(
            header_stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        )
        # Copy the pose position/orientation out of the message.
        position = msg.pose.pose.position
        self._positions.append((position.x, position.y, position.z))
        orientation = msg.pose.pose.orientation
        self._orientations.append(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        # Copy the body-frame twist out of the message.
        linear = msg.twist.twist.linear
        self._linear_velocities.append((linear.x, linear.y, linear.z))
        angular = msg.twist.twist.angular
        self._angular_velocities.append((angular.x, angular.y, angular.z))
        # Reshape each flat 36-entry ROS covariance array to row-major 6x6.
        self._pose_covariances.append(
            np.asarray(msg.pose.covariance, dtype=np.float64).reshape(6, 6)
        )
        self._twist_covariances.append(
            np.asarray(msg.twist.covariance, dtype=np.float64).reshape(6, 6)
        )
        # Latch the frame IDs from the first message only.
        if self._frame_id is None:
            self._frame_id = msg.header.frame_id
            self._child_frame_id = msg.child_frame_id

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[OdometrySeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `OdometrySeries`, or `None` if no valid
                 samples were collected.
        """
        # An empty collector produces no series at all.
        if not self._stamps_ns:
            return None
        # Stack the per-message tuples into fixed-shape NumPy arrays, then
        # drop any sample whose timestamp is invalid, keeping every
        # parallel array aligned.
        times_ns, (positions, orientations, linear, angular, pose_cov, twist_cov) = (
            _filter_and_mask(
                self._stamps_ns,
                np.array(self._positions, dtype=np.float64),
                np.array(self._orientations, dtype=np.float64),
                np.array(self._linear_velocities, dtype=np.float64),
                np.array(self._angular_velocities, dtype=np.float64),
                np.array(self._pose_covariances, dtype=np.float64),
                np.array(self._twist_covariances, dtype=np.float64),
            )
        )
        # No samples survived timestamp validation.
        if times_ns.size == 0:
            return None
        # Convert to elapsed seconds and split into discontinuity-free
        # segments per the time contract.
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        return OdometrySeries(
            times_s=times_s,
            times_ns=times_ns,
            frame_id=self._frame_id or "",
            child_frame_id=self._child_frame_id or "",
            position_m=positions,
            orientation_xyzw=orientations,
            linear_velocity_mps=linear,
            angular_velocity_radps=angular,
            pose_covariance=pose_cov,
            twist_covariance=twist_cov,
            segments=segments,
        )


class ImuCollector:
    """!
    @brief  Accumulates sensor_msgs/msg/Imu samples from one topic during a
            single bag pass.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []
        self._orientations: list[tuple[float, float, float, float]] = []
        self._angular_velocities: list[tuple[float, float, float]] = []
        self._linear_accelerations: list[tuple[float, float, float]] = []
        self._frame_id: Optional[str] = None
        # Latched from the first message's orientation_covariance[0]; see
        # ImuSeries.has_orientation for the sensor_msgs/Imu convention this
        # implements.
        self._has_orientation: Optional[bool] = None

    def append(self, msg: object) -> None:
        """!
        @brief   Records one deserialized sensor_msgs/msg/Imu message.

        @param   msg
                 The deserialized message.

        @return  None
        """
        self._stamps_ns.append(
            header_stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        )
        orientation = msg.orientation
        self._orientations.append(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        angular = msg.angular_velocity
        self._angular_velocities.append((angular.x, angular.y, angular.z))
        linear = msg.linear_acceleration
        self._linear_accelerations.append((linear.x, linear.y, linear.z))
        if self._frame_id is None:
            self._frame_id = msg.header.frame_id
            # sensor_msgs/Imu's documented convention: orientation_covariance
            # [0] == -1 means orientation was not estimated by this sensor.
            self._has_orientation = (
                msg.orientation_covariance[0]
                != ORIENTATION_UNAVAILABLE_COVARIANCE_MARKER
            )

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[ImuSeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `ImuSeries`, or `None` if no valid samples
                 were collected.
        """
        if not self._stamps_ns:
            return None
        times_ns, (orientations, angular, linear) = _filter_and_mask(
            self._stamps_ns,
            np.array(self._orientations, dtype=np.float64),
            np.array(self._angular_velocities, dtype=np.float64),
            np.array(self._linear_accelerations, dtype=np.float64),
        )
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        # A raw/noisy IMU with no orientation estimate carries NaN in that
        # field instead of a misleadingly precise identity quaternion.
        if not self._has_orientation:
            orientations = np.full_like(orientations, np.nan)
        return ImuSeries(
            times_s=times_s,
            times_ns=times_ns,
            frame_id=self._frame_id or "",
            has_orientation=bool(self._has_orientation),
            orientation_xyzw=orientations,
            angular_velocity_radps=angular,
            linear_acceleration_mps2=linear,
            segments=segments,
        )


class JointStateCollector:
    """!
    @brief  Accumulates sensor_msgs/msg/JointState samples, resolving each
            message's joint values by name against a canonical column
            order rather than trusting message array order (matching
            WheelOdometryNode's own findJoint() convention). A message
            whose joint-name set does not exactly match the canonical set
            (missing or duplicate names) is rejected and counted rather
            than corrupting the series.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []
        self._positions: list[list[float]] = []
        self._velocities: list[list[float]] = []
        # The canonical joint-name column order, latched from the first
        # accepted message.
        self._joint_names: Optional[tuple[str, ...]] = None
        # Messages rejected for a missing/duplicate joint name, surfaced as
        # extraction-failure health information by the caller.
        self.rejected_count = 0

    def append(self, msg: object) -> bool:
        """!
        @brief   Records one deserialized sensor_msgs/msg/JointState
                 message, resolving its values by joint name.

        @param   msg
                 The deserialized message.

        @return  `True` if the message's joint-name set matched the
                 canonical set and was recorded; `False` if it was
                 rejected as malformed.

        @throws  None
        """
        names = list(msg.name)
        # Reject a message with a duplicate joint name outright: no
        # consistent column mapping is possible.
        if len(set(names)) != len(names):
            self.rejected_count += 1
            return False
        # The first accepted message fixes the canonical column order.
        if self._joint_names is None:
            self._joint_names = tuple(names)
        # Every later message must name exactly the same joint set,
        # regardless of array order.
        elif set(names) != set(self._joint_names):
            self.rejected_count += 1
            return False
        # Build a name -> value lookup for this message, then resolve every
        # canonical-order joint from it so column order is always stable.
        position_by_name = dict(zip(names, msg.position))
        velocity_by_name = dict(zip(names, msg.velocity))
        self._stamps_ns.append(
            header_stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        )
        self._positions.append(
            [position_by_name[name] for name in self._joint_names]
        )
        self._velocities.append(
            [velocity_by_name[name] for name in self._joint_names]
        )
        return True

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[JointStateSeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `JointStateSeries`, or `None` if no valid
                 samples were collected.
        """
        if not self._stamps_ns or self._joint_names is None:
            return None
        times_ns, (positions, velocities) = _filter_and_mask(
            self._stamps_ns,
            np.array(self._positions, dtype=np.float64),
            np.array(self._velocities, dtype=np.float64),
        )
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        return JointStateSeries(
            times_s=times_s,
            times_ns=times_ns,
            joint_names=self._joint_names,
            position_rad=positions,
            velocity_radps=velocities,
            segments=segments,
        )


class WheelActuatorCollector:
    """!
    @brief  Accumulates actuator_msgs/msg/Actuators samples. This
            project's convention fixes the six-element array order to
            WHEEL_ORDER at the publisher (README "Alpha wheel command"),
            so no by-name resolution is needed here, only cardinality
            validation.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []
        self._velocities: list[list[float]] = []
        self._positions: list[list[float]] = []
        # Messages rejected for not carrying exactly WHEEL_COUNT elements.
        self.rejected_count = 0

    def append(self, msg: object) -> bool:
        """!
        @brief   Records one deserialized actuator_msgs/msg/Actuators
                 message.

        @param   msg
                 The deserialized message.

        @return  `True` if the message carried the required six-wheel
                 cardinality; `False` if it was rejected.
        """
        # Both arrays must carry exactly one value per wheel.
        if len(msg.velocity) != WHEEL_COUNT or len(msg.position) != WHEEL_COUNT:
            self.rejected_count += 1
            return False
        self._stamps_ns.append(header_stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec))
        self._velocities.append(list(msg.velocity))
        self._positions.append(list(msg.position))
        return True

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[WheelActuatorSeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `WheelActuatorSeries`, or `None` if no
                 valid samples were collected.
        """
        if not self._stamps_ns:
            return None
        times_ns, (velocities, positions) = _filter_and_mask(
            self._stamps_ns,
            np.array(self._velocities, dtype=np.float64),
            np.array(self._positions, dtype=np.float64),
        )
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        return WheelActuatorSeries(
            times_s=times_s,
            times_ns=times_ns,
            velocity_radps=velocities,
            position_rad=positions,
            segments=segments,
        )


class WheelScalarCollector:
    """!
    @brief  Accumulates std_msgs/msg/Float64MultiArray six-wheel scalar
            samples (slip ratios/observations). Headerless per the message
            type, so the caller supplies the bag receive timestamp per the
            plan's time contract for headerless topics.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []
        self._values: list[list[float]] = []
        # Messages rejected for not carrying exactly WHEEL_COUNT elements.
        self.rejected_count = 0

    def append(self, msg: object, receive_time_ns: int) -> bool:
        """!
        @brief   Records one deserialized std_msgs/msg/Float64MultiArray
                 message.

        @param   msg
                 The deserialized message.
        @param   receive_time_ns
                 The bag's own receive timestamp for this message.

        @return  `True` if the message carried the required six-wheel
                 cardinality; `False` if it was rejected.
        """
        if len(msg.data) != WHEEL_COUNT:
            self.rejected_count += 1
            return False
        self._stamps_ns.append(receive_time_ns)
        self._values.append(list(msg.data))
        return True

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[WheelScalarSeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `WheelScalarSeries`, or `None` if no valid
                 samples were collected.
        """
        if not self._stamps_ns:
            return None
        times_ns, (values,) = _filter_and_mask(
            self._stamps_ns, np.array(self._values, dtype=np.float64)
        )
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        return WheelScalarSeries(
            times_s=times_s, times_ns=times_ns, values=values, segments=segments
        )


class TwistCommandCollector:
    """!
    @brief  Accumulates headerless geometry_msgs/msg/Twist command
            samples, timestamped by bag receive time per the plan's time
            contract.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []
        self._linear: list[tuple[float, float, float]] = []
        self._angular: list[tuple[float, float, float]] = []

    def append(self, msg: object, receive_time_ns: int) -> None:
        """!
        @brief   Records one deserialized geometry_msgs/msg/Twist message.

        @param   msg
                 The deserialized message.
        @param   receive_time_ns
                 The bag's own receive timestamp for this message.

        @return  None
        """
        self._stamps_ns.append(receive_time_ns)
        linear = msg.linear
        self._linear.append((linear.x, linear.y, linear.z))
        angular = msg.angular
        self._angular.append((angular.x, angular.y, angular.z))

    def finalize(
        self, start_time_ns: int, maximum_gap_s: float
    ) -> Optional[TwistCommandSeries]:
        """!
        @brief   Builds the final typed series from every collected sample.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.
        @param   maximum_gap_s
                 Largest inter-sample gap still considered one segment.

        @return  The assembled `TwistCommandSeries`, or `None` if no valid
                 samples were collected.
        """
        if not self._stamps_ns:
            return None
        times_ns, (linear, angular) = _filter_and_mask(
            self._stamps_ns,
            np.array(self._linear, dtype=np.float64),
            np.array(self._angular, dtype=np.float64),
        )
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)
        segments = segment_series(times_ns, maximum_gap_s)
        return TwistCommandSeries(
            times_s=times_s,
            times_ns=times_ns,
            linear_mps=linear,
            angular_radps=angular,
            segments=segments,
        )


class VisualResetCollector:
    """!
    @brief  Accumulates std_msgs/msg/Empty reset-event timestamps from
            /alpha/localisation/visual/reset.
    """

    def __init__(self) -> None:
        """!
        @brief  Initializes an empty collector.
        """
        self._stamps_ns: list[int] = []

    def append(self, receive_time_ns: int) -> None:
        """!
        @brief   Records one reset event.

        @param   receive_time_ns
                 The bag's own receive timestamp for this event.

        @return  None
        """
        self._stamps_ns.append(receive_time_ns)

    def finalize(self, start_time_ns: int) -> VisualResetSeries:
        """!
        @brief   Builds the final typed series from every collected event.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.

        @return  The assembled `VisualResetSeries` (possibly empty).
        """
        times_ns = np.array(self._stamps_ns, dtype=np.int64)
        times_ns = times_ns[filter_finite_timestamps(times_ns)]
        times_s = elapsed_seconds(times_ns, start_time_ns)
        return VisualResetSeries(times_s=times_s, times_ns=times_ns)


class _BoundedEvenSampler:
    """!
    @brief  Maintains an evenly-spaced, size-bounded, deterministic sample
            of a single-pass stream of unknown total length, per the
            plan's bounded-memory requirement for high-volume streams
            (Defect 2 in the plan's 2026-09-22 audit: the previous
            implementation decoded and retained every frame during
            ingestion, only discarding down to the bound at finalize()).

            Classic "keep every Nth, then thin and double the step when
            the buffer overflows" streaming decimation: an offered item is
            decoded at all only if it currently lands on the step
            schedule, so a discarded item is never decoded, and the
            buffer never exceeds twice the configured bound. Covers the
            whole stream (not just its beginning) because the step
            schedule keeps advancing for as long as items are offered.

            One-slot contract (2026-09-23 correction: the 2026-09-22
            correction pass's own `finalize()` silently dropped the true
            last item when `maximum_count == 1`, because a single slot
            cannot hold two distinct endpoints -- `_evenly_sampled_indices`
            was never asked to prove otherwise for count 1, so its
            `np.linspace(..., num=1)` behavior of returning only the start
            went unnoticed by the tests that only exercised counts >= 3):
            - `maximum_count == 0`: nothing is ever retained or decoded.
            - `maximum_count == 1`: the true final eligible item is kept
              (never the first) -- for a single retained sample, "what the
              run ended with" is more useful than "what it started with",
              and the run's start is already visible from message/health
              counts independent of which frames were retained.
            - `maximum_count >= 2`: both the true first and true final
              eligible items are kept, with the remaining slots sampled
              deterministically across the complete stream.
    """

    def __init__(self, maximum_count: int) -> None:
        """!
        @brief   Initializes an empty sampler.

        @param   maximum_count
                 The largest number of items the final sample may contain.
        """
        self._maximum_count = max(0, maximum_count)
        # (stream-order index, item) pairs currently retained.
        self._buffer: list[tuple[int, object]] = []
        self._step = 1
        self._offered_count = 0

    def offer(self, factory: Callable[[], object]) -> None:
        """!
        @brief   Considers one more stream item for retention.

        @param   factory
                 A zero-argument callable that decodes/constructs the
                 item; invoked only if this item lands on the current step
                 schedule, so a discarded item is never decoded.

        @return  None
        """
        index = self._offered_count
        self._offered_count += 1
        if self._maximum_count == 0:
            return
        if index % self._step == 0:
            self._buffer.append((index, factory()))
            if len(self._buffer) > 2 * self._maximum_count:
                self._thin()

    def _thin(self) -> None:
        """!
        @brief   Halves the buffer (keeping every other retained entry)
                 and doubles the step, bounding memory as the stream
                 grows without ever re-inspecting a discarded item.

        @return  None
        """
        self._buffer = self._buffer[::2]
        self._step *= 2

    def finalize(self, last_factory: Optional[Callable[[], object]] = None) -> list:
        """!
        @brief   Produces the final, evenly-spaced, bounded sample, per
                 this class's one-slot contract (see the class docstring):
                 nothing for `maximum_count == 0`; only the true final
                 eligible item for `maximum_count == 1`; both true
                 endpoints plus deterministic in-between sampling for
                 `maximum_count >= 2`.

        @param   last_factory
                 A zero-argument callable that decodes/constructs the true
                 last offered item, used only if that item does not
                 already land in the buffer (so it is never decoded
                 twice); `None` if no item was ever offered or the caller
                 has no such item to add.

        @return  Up to `maximum_count` items, in stream order. For
                 `maximum_count >= 2`, includes the first offered item and
                 the true last offered item when `last_factory` is given,
                 so neither end of the run is silently dropped by the step
                 schedule. For `maximum_count == 1`, only the true last
                 offered item is returned (see the class docstring for why
                 a single slot cannot honor both endpoints).
        """
        if self._maximum_count == 0 or self._offered_count == 0:
            return []
        if last_factory is not None and (
            not self._buffer or self._buffer[-1][0] != self._offered_count - 1
        ):
            # Appending the true last item here, then thinning if that
            # overflows the buffer, always leaves it at self._buffer[-1]:
            # this append only ever runs when the buffer was already at
            # its steady-state cap of `2 * maximum_count`, so the
            # post-append length is always the odd value
            # `2 * maximum_count + 1`, and `buffer[::2]` always retains an
            # odd-length list's own last position.
            self._buffer.append((self._offered_count - 1, last_factory()))
            if len(self._buffer) > 2 * self._maximum_count:
                self._thin()
        if self._maximum_count == 1:
            # A single slot cannot hold two distinct endpoints; the true
            # final item (guaranteed to be self._buffer[-1] by the append
            # above whenever last_factory was supplied) is the more useful
            # one to keep for a single retained sample.
            return [self._buffer[-1][1]]
        if len(self._buffer) <= self._maximum_count:
            return [item for _, item in self._buffer]
        positions = _evenly_sampled_indices(len(self._buffer), self._maximum_count)
        return [self._buffer[position][1] for position in positions]


class PointCloudCollector:
    """!
    @brief  Tracks every sensor_msgs/msg/PointCloud2 message's point count
            across the whole run (cheap: one int per message), while
            decoding and retaining only a `_BoundedEvenSampler`-bounded set
            of full snapshots, per the plan's bounded-memory requirement
            for high-volume streams.
    """

    def __init__(self, maximum_snapshots: int) -> None:
        """!
        @brief   Initializes an empty collector.

        @param   maximum_snapshots
                 The largest number of full point clouds to retain.
        """
        self._stamps_ns: list[int] = []
        self._point_counts: list[int] = []
        self._sampler = _BoundedEvenSampler(maximum_snapshots)
        # The most recently seen valid-timestamp message, kept undecoded
        # (cheap: one message reference, replaced on every later message)
        # so the stream's true last eligible snapshot can still be decoded
        # in finalize() without having decoded every message along the way.
        self._last_eligible: Optional[tuple[int, object]] = None

    def append(self, msg: object, receive_time_ns: int) -> None:
        """!
        @brief   Records one deserialized sensor_msgs/msg/PointCloud2
                 message.

        @param   msg
                 The deserialized message.
        @param   receive_time_ns
                 The bag's own receive timestamp for this message.

        @return  None
        """
        self._stamps_ns.append(receive_time_ns)
        self._point_counts.append(int(msg.width))
        if not is_valid_timestamp_ns(receive_time_ns):
            return
        self._last_eligible = (receive_time_ns, msg)
        self._sampler.offer(lambda: (receive_time_ns, _decode_point_cloud_snapshot(msg)))

    def finalize(
        self, start_time_ns: int
    ) -> Optional[VisualPointCloudSeries]:
        """!
        @brief   Builds the final typed series, with a bounded, evenly
                 sampled set of full snapshots.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.

        @return  The assembled `VisualPointCloudSeries`, or `None` if no
                 messages were collected.
        """
        if not self._stamps_ns:
            return None
        times_ns = np.array(self._stamps_ns, dtype=np.int64)
        valid_mask = filter_finite_timestamps(times_ns)
        times_ns = times_ns[valid_mask]
        point_counts = np.array(self._point_counts, dtype=np.int64)[valid_mask]
        if times_ns.size == 0:
            return None
        times_s = elapsed_seconds(times_ns, start_time_ns)

        def _last_factory() -> tuple[int, PointCloudSnapshot]:
            stamp_ns, msg = self._last_eligible
            return stamp_ns, _decode_point_cloud_snapshot(msg)

        sampled = self._sampler.finalize(
            _last_factory if self._last_eligible is not None else None
        )
        snapshots = tuple(
            PointCloudSnapshot(
                time_s=float(elapsed_seconds(np.array([stamp_ns], dtype=np.int64), start_time_ns)[0]),
                points_m=snapshot.points_m,
                frame_id=snapshot.frame_id,
                is_inlier_cloud=snapshot.is_inlier_cloud,
            )
            for stamp_ns, snapshot in sampled
        )
        return VisualPointCloudSeries(
            times_s=times_s,
            times_ns=times_ns,
            point_counts=point_counts,
            snapshots=snapshots,
        )


class ImageCollector:
    """!
    @brief  Tracks every sensor_msgs/msg/Image message's presence (cheap
            counters only) while decoding and retaining only a
            `_BoundedEvenSampler`-bounded, evenly sampled set of frames,
            per the plan's bounded-memory requirement.
    """

    def __init__(self, topic_name: str, maximum_frames: int) -> None:
        """!
        @brief   Initializes an empty collector.

        @param   topic_name
                 The source topic name, retained for the report page.
        @param   maximum_frames
                 The largest number of decoded frames to retain.
        """
        self._topic_name = topic_name
        self._sampler = _BoundedEvenSampler(maximum_frames)
        self.total_message_count = 0
        self.rejected_encoding_count = 0
        # The most recently seen decodable, valid-timestamp message, kept
        # undecoded (cheap: one message reference, replaced on every later
        # message) so the stream's true last eligible frame can still be
        # decoded in finalize() without having decoded every message along
        # the way.
        self._last_eligible: Optional[tuple[int, object]] = None

    def append(self, msg: object) -> None:
        """!
        @brief   Records one deserialized sensor_msgs/msg/Image message.

        @param   msg
                 The deserialized message.

        @return  None
        """
        self.total_message_count += 1
        # Reject an unsupported encoding rather than guessing a layout;
        # the rest of the page still renders with a warning about it.
        if msg.encoding not in SUPPORTED_IMAGE_ENCODINGS:
            self.rejected_encoding_count += 1
            return
        stamp_ns = header_stamp_to_ns(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not is_valid_timestamp_ns(stamp_ns):
            return
        self._last_eligible = (stamp_ns, msg)
        self._sampler.offer(
            lambda: (stamp_ns, _decode_image_to_rgb(msg), msg.encoding)
        )

    def finalize(self, start_time_ns: int) -> ImageFrameSeries:
        """!
        @brief   Builds the final typed series, with a bounded, evenly
                 sampled set of decoded frames.

        @param   start_time_ns
                 The run's elapsed-zero reference timestamp.

        @return  The assembled `ImageFrameSeries` (possibly with zero
                 retained frames if every message had an unsupported
                 encoding or an invalid timestamp).
        """
        def _last_factory() -> tuple[int, npt.NDArray[np.uint8], str]:
            stamp_ns, msg = self._last_eligible
            return stamp_ns, _decode_image_to_rgb(msg), msg.encoding

        sampled = self._sampler.finalize(
            _last_factory if self._last_eligible is not None else None
        )
        frames = tuple(
            ImageFrame(
                time_s=float(elapsed_seconds(np.array([stamp_ns], dtype=np.int64), start_time_ns)[0]),
                rgb=rgb,
                source_encoding=encoding,
            )
            for stamp_ns, rgb, encoding in sampled
        )
        return ImageFrameSeries(
            topic_name=self._topic_name,
            total_message_count=self.total_message_count,
            rejected_encoding_count=self.rejected_encoding_count,
            frames=frames,
        )


def _evenly_sampled_indices(total_count: int, maximum_count: int) -> list[int]:
    """!
    @brief   Picks up to `maximum_count` indices, evenly spaced across
             `[0, total_count)`. For `maximum_count >= 2`, both the first
             and last index are always included. `_BoundedEvenSampler`
             (this module's only caller) never invokes this with
             `maximum_count < 2` -- it handles 0 and 1 itself, via its own
             one-slot contract -- because a single requested index cannot
             represent both a series' first and last sample at once:
             `np.linspace(0, total_count - 1, num=1)` returns only index 0,
             not both endpoints, a mismatch with "always including the
             first and last" that a caller expecting `maximum_count == 1`
             to honor both endpoints could otherwise miss (see the plan's
             2026-09-23 correction).

    @param   total_count
             The number of items available to sample from.
    @param   maximum_count
             The largest number of indices to return.

    @return  A sorted list of unique indices, length
             `min(total_count, maximum_count)`.
    """
    # Nothing to sample, or the bound allows keeping everything.
    if total_count == 0 or maximum_count <= 0:
        return []
    if maximum_count >= total_count:
        return list(range(total_count))
    # Evenly spaced floating-point positions across the valid index range,
    # rounded to the nearest integer index and de-duplicated.
    positions = np.linspace(0, total_count - 1, num=maximum_count)
    indices = sorted(set(int(round(position)) for position in positions))
    return indices


def _decode_xyz_point_cloud(msg: object) -> npt.NDArray[np.float64]:
    """!
    @brief   Decodes a packed-float32 XYZ sensor_msgs/msg/PointCloud2 (the
             only layout publishPointCloud.cc emits) into an (N, 3) array.

    @param   msg
             The deserialized PointCloud2 message.

    @return  Points in the cloud's own frame, metres, shape (width, 3).

    @throws  ValueError
             If the message's fields do not match the expected packed
             float32 x/y/z layout this project's visual odometry emits.
    """
    field_names = tuple(field.name for field in msg.fields)
    # Guard against a future/foreign point cloud layout rather than
    # silently misreading its bytes as XYZ floats.
    if field_names != ("x", "y", "z"):
        raise ValueError(f"unsupported PointCloud2 field layout: {field_names}")
    # Reinterpret the raw byte buffer as (N, 3) float32, honoring point_step
    # in case of future padding, then upcast to float64 for downstream math.
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    point_count = int(msg.width)
    if point_count == 0:
        return np.empty((0, 3), dtype=np.float64)
    stride = int(msg.point_step)
    points = np.empty((point_count, 3), dtype=np.float64)
    for index in range(point_count):
        offset = index * stride
        points[index] = np.frombuffer(
            raw[offset : offset + 12].tobytes(), dtype=np.float32
        )
    return points


def _decode_point_cloud_snapshot(msg: object) -> PointCloudSnapshot:
    """!
    @brief   Decodes one sensor_msgs/msg/PointCloud2 message into a
             `PointCloudSnapshot`, called only for a message
             `PointCloudCollector`'s `_BoundedEvenSampler` has decided to
             retain (see this module's docstring).

    @param   msg
             The deserialized PointCloud2 message.

    @return  The decoded snapshot, with a placeholder `time_s` of `nan`;
             the caller patches this once the run's elapsed-time reference
             is known.
    """
    return PointCloudSnapshot(
        time_s=float("nan"),
        points_m=_decode_xyz_point_cloud(msg),
        frame_id=msg.header.frame_id,
        is_inlier_cloud=True,
    )


def _decode_image_to_rgb(msg: object) -> npt.NDArray[np.uint8]:
    """!
    @brief   Decodes a sensor_msgs/msg/Image with a supported encoding
             into an (H, W, 3) RGB array, honoring `step` rather than
             assuming no row padding.

    @param   msg
             The deserialized Image message, whose `encoding` has already
             been checked against SUPPORTED_IMAGE_ENCODINGS.

    @return  Decoded RGB pixel data, shape (height, width, 3).
    """
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    bytes_per_pixel = _BYTES_PER_PIXEL[msg.encoding]
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, step)
    # Slice off any row padding beyond the pixel data before reshaping.
    row_pixel_bytes = raw[:height, : width * bytes_per_pixel]
    if msg.encoding == "mono8":
        # Broadcast the single channel to RGB so every consumer only ever
        # handles one (H, W, 3) layout.
        gray = row_pixel_bytes.reshape(height, width)
        return np.repeat(gray[:, :, np.newaxis], 3, axis=2)
    channels = row_pixel_bytes.reshape(height, width, 3)
    if msg.encoding == "bgr8":
        # Swap channel order so the caller always receives RGB.
        return channels[:, :, ::-1].copy()
    return channels.copy()


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone image-decode self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Decode a raw sensor_msgs/Image byte dump (as produced by "
            "`ros2 topic echo --field data --raw`) is not supported; this "
            "standalone mode only reports the module's supported encodings."
        )
    )
    # No arguments are needed for the informational self-check; the parser
    # still follows the shared pattern so -h documents the module.
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the image encodings this module can
             decode, for quick manual reference.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    parser.parse_args(argv)
    print("Supported sensor_msgs/Image encodings:")
    for encoding in SUPPORTED_IMAGE_ENCODINGS:
        print(f"  {encoding}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
