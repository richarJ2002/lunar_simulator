"""!
@brief  Tests for recorded /alpha/diagnostics handling: the
        DiagnosticArrayCollector's flattening of synthetic DiagnosticArray
        messages, and bag_diagnostics' conversion of those statuses into
        bag-elapsed visual and estimator records (including the join of each
        per-source status with its tick's filter-wide status).
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.data.extractors import DiagnosticArrayCollector
from python_tools.data.models import DiagnosticTimeBasis
from python_tools.diagnostics import bag_diagnostics

# Every key the visual_odometry status publishes, with plausible values.
_VISUAL_VALUES = {
    "received": "40", "accepted": "38", "failed": "2",
    "reception_rate_hz": "3.9", "accepted_rate_hz": "3.7", "age_s": "0.05",
    "processing_ms": "150.0", "conversion_ms": "2.0", "disparity_ms": "60.0",
    "detection_ms": "40.0", "tracking_ms": "45.0", "reconstruction_ms": "0.5",
    "pnp_ms": "1.5", "detected": "600", "tracked": "480", "stereo_valid": "300",
    "correspondences": "250", "inliers": "180", "occupancy": "0.5",
    "near_mid_ratio": "0.4", "disparity_p10_px": "5.0", "disparity_p50_px": "10.0",
    "disparity_p90_px": "20.0", "depth_p10_m": "6.0", "depth_p50_m": "12.0",
    "depth_p90_m": "24.0", "reprojection_rms_px": "0.3",
    "normal_condition": "1.5e+03", "accepted_interval_s": "0.25",
    "consecutive_failures": "0",
}

# Every key a continuous_ekf/<source> status publishes.
_SOURCE_VALUES = {
    "received": "500", "accepted": "480", "age_rejected": "20",
    "nis_rejected": "1", "numerical_rejected": "0", "fused": "479",
    "publication": "30.10", "admission": "30.12", "start": "30.12",
    "end": "30.13", "estimator_epoch": "30.12", "nis": "1.2",
    "correction_norm": "0.004", "pre_init": "10", "negative_age": "10",
    "too_old": "0", "state_gap": "0", "rollback_failed": "0",
    "predict_failed": "0",
}

# Every key the filter-wide continuous_ekf status publishes.
_FILTER_VALUES = {
    "covariance_trace": "0.02", "covariance_min_eigenvalue": "1e-09",
    "covariance_diagonal_min": "1e-09", "covariance_diagonal_max": "0.01",
    "quaternion_norm": "1.0", "accel_bias_x_mps2": "0.01",
    "accel_bias_y_mps2": "-0.02", "accel_bias_z_mps2": "0.03",
    "gyro_bias_x_radps": "0.001", "gyro_bias_y_radps": "-0.002",
    "gyro_bias_z_radps": "0.003",
}


def _status(name: str, values: dict[str, str], level: bytes = b"\x00") -> SimpleNamespace:
    """!
    @brief   Builds a duck-typed DiagnosticStatus like rclpy deserializes.

    @param   name
             Status name.
    @param   values
             Key/value text.
    @param   level
             Level as rclpy's length-1 bytes.

    @return  The synthetic status.
    """
    return SimpleNamespace(
        name=name,
        level=level,
        message="summary",
        values=[SimpleNamespace(key=key, value=value) for key, value in values.items()],
    )


def _array(stamp_s: int, statuses: list[SimpleNamespace]) -> SimpleNamespace:
    """!
    @brief   Builds a duck-typed DiagnosticArray stamped at whole seconds.

    @param   stamp_s
             Header stamp, whole seconds.
    @param   statuses
             The array's statuses.

    @return  The synthetic array.
    """
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_s, nanosec=0)),
        status=statuses,
    )


def _collect(arrays: list[SimpleNamespace], start_s: int = 100):
    """!
    @brief   Runs the collector over synthetic arrays.

    @param   arrays
             Arrays in recording order.
    @param   start_s
             The run's elapsed-zero reference, whole seconds.

    @return  The finalized DiagnosticStatusSeries.
    """
    collector = DiagnosticArrayCollector("/alpha/diagnostics")
    for array in arrays:
        collector.append(array)
    return collector.finalize(start_s * 1_000_000_000)


class DiagnosticArrayCollectorTests(unittest.TestCase):
    """!
    @brief  Tests for flattening DiagnosticArray messages.
    """

    def test_flattens_sorts_and_decodes_levels(self) -> None:
        """!
        @brief  Statuses share their array's stamp, are sorted by time even
                when arrays arrive out of order, and byte levels decode.
        """
        series = _collect(
            [
                _array(103, [_status("b", {"k": "2"}, b"\x02")]),
                _array(101, [_status("a", {"k": "1"}), _status("c", {"k": "3"}, b"\x01")]),
            ]
        )
        self.assertEqual([sample.name for sample in series.samples], ["a", "c", "b"])
        self.assertEqual([sample.time_s for sample in series.samples], [1.0, 1.0, 3.0])
        self.assertEqual([sample.level for sample in series.samples], [0, 1, 2])
        self.assertEqual(series.samples[0].values, {"k": "1"})

    def test_drops_unstamped_arrays(self) -> None:
        """!
        @brief  A never-stamped array cannot be placed in time and is
                dropped.
        """
        series = _collect([_array(0, [_status("a", {})]), _array(102, [_status("b", {})])])
        self.assertEqual([sample.name for sample in series.samples], ["b"])


class BagDiagnosticConversionTests(unittest.TestCase):
    """!
    @brief  Tests for converting recorded statuses into records.
    """

    def test_builds_records_on_the_bag_elapsed_axis(self) -> None:
        """!
        @brief  Visual and per-source records are built, joined with their
                tick's filter snapshot, and marked bag-elapsed.
        """
        series = _collect(
            [
                _array(
                    130,
                    [
                        _status("visual_odometry", _VISUAL_VALUES),
                        _status("continuous_ekf", _FILTER_VALUES),
                        _status("continuous_ekf/wheel", _SOURCE_VALUES),
                        _status("wheel_odometry", {"ready": "true"}),
                    ],
                )
            ]
        )
        log = bag_diagnostics.diagnostic_log_from_bag(series, Path("/bag"))
        self.assertIs(log.time_basis, DiagnosticTimeBasis.BAG_ELAPSED)
        self.assertEqual(log.unparsed_line_count, 0)
        self.assertEqual(len(log.visual_records), 1)
        self.assertAlmostEqual(log.visual_records[0].time_s, 30.0)
        self.assertEqual(log.visual_records[0].inliers, 180)
        self.assertAlmostEqual(log.visual_records[0].normal_condition, 1500.0)
        record = log.localisation_records[0]
        self.assertEqual(record.source, "wheel")
        self.assertEqual(record.age_rejected, 20)
        self.assertEqual(record.age_rejection_breakdown.negative_age, 10)
        self.assertAlmostEqual(record.gyro_bias_body_radps[2], 0.003)

    def test_missing_key_or_filter_snapshot_counts_as_malformed(self) -> None:
        """!
        @brief  A status missing a required key, or a source status whose
                tick has no filter-wide status, is counted, not guessed.
        """
        incomplete_visual = dict(_VISUAL_VALUES)
        del incomplete_visual["inliers"]
        series = _collect(
            [
                _array(110, [_status("visual_odometry", incomplete_visual)]),
                _array(111, [_status("continuous_ekf/imu", _SOURCE_VALUES)]),
            ]
        )
        log = bag_diagnostics.diagnostic_log_from_bag(series, Path("/bag"))
        self.assertEqual(log.unparsed_line_count, 2)
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(len(log.localisation_records), 0)

    def test_inertial_calibration_time_uses_the_completing_stamp(self) -> None:
        """!
        @brief  The first calibrated status gives the completing sample's
                stamp on the bag-elapsed axis; uncalibrated ticks are
                skipped and a run that never calibrates reports None.
        """
        series = _collect(
            [
                _array(104, [_status("inertial_odometry", {"calibrated": "false"})]),
                _array(
                    106,
                    [
                        _status(
                            "inertial_odometry",
                            {"calibrated": "true", "calibration_complete_stamp_s": "105.5"},
                        )
                    ],
                ),
            ]
        )
        self.assertAlmostEqual(
            bag_diagnostics.inertial_calibration_complete_elapsed_s(series, 100_000_000_000), 5.5
        )
        uncalibrated = _collect([_array(104, [_status("inertial_odometry", {"calibrated": "false"})])])
        self.assertIsNone(
            bag_diagnostics.inertial_calibration_complete_elapsed_s(uncalibrated, 100_000_000_000)
        )

    def test_system_state_transitions_skip_heartbeats(self) -> None:
        """!
        @brief  Only state changes are reported; repeated heartbeats of the
                same state are skipped.
        """
        series = _collect(
            [
                _array(101, [_status("system_state", {"state": "INITIALISING"})]),
                _array(102, [_status("system_state", {"state": "INITIALISING"})]),
                _array(112, [_status("system_state", {"state": "READY"})]),
                _array(113, [_status("system_state", {"state": "READY"})]),
                _array(150, [_status("system_state", {"state": "HOLD"})]),
            ]
        )
        transitions = bag_diagnostics.system_state_transitions(series)
        self.assertEqual(
            [(time_s, state) for time_s, state, _ in transitions],
            [(1.0, "INITIALISING"), (12.0, "READY"), (50.0, "HOLD")],
        )

    def test_latest_status_values_prefers_the_newest(self) -> None:
        """!
        @brief  The latest status of a name wins; an absent name gives {}.
        """
        series = _collect(
            [
                _array(101, [_status("alpha_driver_node", {"commands_blocked": "1"})]),
                _array(105, [_status("alpha_driver_node", {"commands_blocked": "4"})]),
            ]
        )
        self.assertEqual(
            bag_diagnostics.latest_status_values(series, "alpha_driver_node"),
            {"commands_blocked": "4"},
        )
        self.assertEqual(bag_diagnostics.latest_status_values(series, "missing"), {})

    def test_axis_title_follows_time_basis(self) -> None:
        """!
        @brief  Recorded diagnostics plot on the elapsed-simulation axis;
                legacy logs keep their explicit caveat title.
        """
        from python_tools.data.models import DiagnosticLog

        bag_log = bag_diagnostics.diagnostic_log_from_bag(_collect([]), Path("/bag"))
        legacy_log = DiagnosticLog(log_path=Path("/log"))
        self.assertEqual(
            bag_diagnostics.diagnostic_time_axis_title(bag_log),
            bag_diagnostics.BAG_DIAGNOSTIC_TIME_AXIS_TITLE,
        )
        self.assertIn("not bag-elapsed", bag_diagnostics.diagnostic_time_axis_title(legacy_log))


if __name__ == "__main__":
    unittest.main()
