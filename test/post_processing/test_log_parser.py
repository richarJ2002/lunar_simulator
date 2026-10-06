"""Tests for diagnostics log parsing.

Contents:
    ParseVisualDiagTests: visual line parsing.
    ParseLocalisationDiagTests: localisation parsing.
    LogRelativeTimingTests: log-relative timing.
    CalibrationMarkerTests: calibration marker lookup.

Uses representative lines captured from a real run so
regexes are checked against real node output. Times are
elapsed seconds since the log file's own first record.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.diagnostics import log_parser

# Representative lines captured from a real run's log, used
# verbatim so regexes meet real node output. Bracketed stamps
# (~1.79e9) are wall-clock epoch seconds, not simulated time.
_REAL_VISUAL_DIAG_LINE = (
    "[INFO] [1790072206.200122095] [visual_odometry]: "
    "visual_diag received=6 accepted=6 failed=0 "
    "reception_rate_hz=1.200 accepted_rate_hz=1.200 "
    "age_s=0.215000 processing_ms=103.432 "
    "conversion_ms=1.703 disparity_ms=33.508 "
    "detection_ms=32.350 tracking_ms=32.880 "
    "reconstruction_ms=0.012 pnp_ms=0.241 "
    "detected=240 tracked=196 stereo_valid=126 "
    "correspondences=114 inliers=114 "
    "occupancy=0.250 near_mid_ratio=0.632 "
    "disparity_p10_p50_p90=[9.269,20.906,33.750] "
    "depth_p10_p50_p90_m=[3.558,5.742,12.947] "
    "reprojection_rms_px=0.001 normal_condition=5.000e+02 "
    "accepted_interval_s=0.400000 consecutive_failures=0"
)
_REAL_LOCALISATION_DIAG_LINE = (
    "[INFO] [1790072206.208832529] [continuous_ekf]: "
    "localisation_diag source=imu received=211 accepted=211 "
    "age_rejected=0 nis_rejected=0 numerical_rejected=0 "
    "fused=112 publication=7.960000 admission=7.960000 "
    "start=7.960000 end=7.960000 estimator_epoch=7.960000 "
    "nis=0.000000 correction_norm=0.000000 "
    "covariance_trace=0.329332 "
    "covariance_min_eigenvalue=2.334989e-05 "
    "covariance_diagonal_range=[7.797882e-04,8.578001e-02] "
    "quaternion_norm=1.000000000000 "
    "accel_bias_body_mps2=[0.004082,-0.007037,0.028006] "
    "gyro_bias_body_radps=[0.002169,-0.000285,0.001520]"
)
_REAL_CALIBRATION_LINE = (
    "[INFO] [1790072201.177104622] [inertial_odometry]: "
    "IMU calibration requires a stationary rover for "
    "100 samples; filtered samples are temporally correlated"
)
_REAL_CALIBRATION_COMPLETE_LINE = (
    "[INFO] [1790072201.200000000] [inertial_odometry]: "
    "IMU stationary calibration complete (100 samples)"
)


class ParseVisualDiagTests(unittest.TestCase):
    """Tests for visual_diag parsing."""

    def _write_and_parse(self, lines: list[str]):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text("\n".join(lines) + "\n")
            return log_parser.parse_log_file(log_path)

    def test_parses_real_line_fields(self) -> None:
        """Every field of a real line is extracted."""
        log = self._write_and_parse([_REAL_VISUAL_DIAG_LINE])
        self.assertEqual(len(log.visual_records), 1)
        record = log.visual_records[0]
        self.assertEqual(record.received, 6)
        self.assertEqual(record.inliers, 114)
        self.assertAlmostEqual(record.reprojection_rms_px, 0.001)
        self.assertAlmostEqual(record.normal_condition, 500.0)
        self.assertAlmostEqual(record.disparity_p50_px, 20.906)
        self.assertEqual(record.consecutive_failures, 0)
        self.assertEqual(log.unparsed_line_count, 0)

    def test_non_diagnostic_lines_are_ignored(self) -> None:
        """Ordinary lines parse to zero records."""
        log = self._write_and_parse(
            ["[INFO] [1.0] [some_node]: ordinary startup message"]
        )
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(len(log.localisation_records), 0)
        self.assertEqual(log.unparsed_line_count, 0)

    def test_compact_lines_are_read_but_carry_no_records(
        self,
    ) -> None:
        """Timeless tagged lines count as unparsed."""
        log = self._write_and_parse(
            [
                "[INFO] [continuous_ekf]: EKF fused v9 w245 rej 0",
                "[INFO] [visual_odometry]: "
                + _REAL_VISUAL_DIAG_LINE.split(": ", 1)[1],
            ]
        )
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(log.unparsed_line_count, 1)

    def test_malformed_tagged_line_counts_as_unparsed(self) -> None:
        """Malformed tagged line counts as unparsed."""
        log = self._write_and_parse(
            ["[INFO] [1.0] [visual_odometry]: "
             "visual_diag some_future_field=1"]
        )
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(log.unparsed_line_count, 1)


class ParseLocalisationDiagTests(unittest.TestCase):
    """Tests for localisation_diag parsing."""

    def _write_and_parse(self, lines: list[str]):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text("\n".join(lines) + "\n")
            return log_parser.parse_log_file(log_path)

    def test_parses_real_line_fields(self) -> None:
        """Every field including biases is extracted."""
        log = self._write_and_parse([_REAL_LOCALISATION_DIAG_LINE])
        self.assertEqual(len(log.localisation_records), 1)
        record = log.localisation_records[0]
        self.assertEqual(record.source, "imu")
        self.assertEqual(record.received, 211)
        self.assertEqual(record.fused, 112)
        self.assertAlmostEqual(
            record.quaternion_norm, 1.0, places=9
        )
        self.assertAlmostEqual(
            record.accel_bias_body_mps2[1], -0.007037
        )
        self.assertAlmostEqual(
            record.gyro_bias_body_radps[2], 0.001520
        )

    def test_legacy_line_has_no_age_rejection_breakdown(
        self,
    ) -> None:
        """Legacy line reports breakdown as absent."""
        log = self._write_and_parse([_REAL_LOCALISATION_DIAG_LINE])
        self.assertIsNone(
            log.localisation_records[0].age_rejection_breakdown
        )

    def test_parses_age_rejection_breakdown(self) -> None:
        """Six age-rejection counters parse in order."""
        line = (
            _REAL_LOCALISATION_DIAG_LINE
            + " pre_init=100 negative_age=250 too_old=40"
            + " state_gap=5 rollback_failed=2 predict_failed=0"
        )
        log = self._write_and_parse([line])
        self.assertEqual(log.unparsed_line_count, 0)
        breakdown = (
            log.localisation_records[0].age_rejection_breakdown
        )
        self.assertIsNotNone(breakdown)
        self.assertEqual(
            (
                breakdown.pre_init,
                breakdown.negative_age,
                breakdown.too_old,
                breakdown.state_gap,
                breakdown.rollback_failed,
                breakdown.predict_failed,
            ),
            (100, 250, 40, 5, 2, 0),
        )

    def test_partial_age_rejection_breakdown_is_unparsed(
        self,
    ) -> None:
        """Truncated breakdown counts as unparsed."""
        line = (
            _REAL_LOCALISATION_DIAG_LINE
            + " pre_init=100 negative_age=250"
        )
        log = self._write_and_parse([line])
        self.assertEqual(len(log.localisation_records), 0)
        self.assertEqual(log.unparsed_line_count, 1)

    def test_multiple_sources_all_parsed(self) -> None:
        """Consecutive per-source lines all parse."""
        lines = [
            _REAL_LOCALISATION_DIAG_LINE,
            _REAL_LOCALISATION_DIAG_LINE.replace(
                "source=imu", "source=visual"
            ),
            _REAL_LOCALISATION_DIAG_LINE.replace(
                "source=imu", "source=wheel"
            ),
        ]
        log = self._write_and_parse(lines)
        self.assertEqual(
            [record.source for record in log.localisation_records],
            ["imu", "visual", "wheel"],
        )


class LogRelativeTimingTests(unittest.TestCase):
    """Tests for log-relative time_s."""

    def test_first_record_is_zero_and_second_reflects_the_gap(
        self,
    ) -> None:
        """First record is 0, second keeps spacing."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            payload = _REAL_VISUAL_DIAG_LINE.split(": ", 1)[1]
            first = (
                "[INFO] [10.000000000] [visual_odometry]: "
                f"{payload}"
            )
            second = (
                "[INFO] [15.000000000] [visual_odometry]: "
                f"{payload}"
            )
            log_path.write_text(first + "\n" + second + "\n")
            log = log_parser.parse_log_file(log_path)
            self.assertAlmostEqual(
                log.visual_records[0].time_s, 0.0, places=6
            )
            self.assertAlmostEqual(
                log.visual_records[1].time_s, 5.0, places=6
            )

    def test_realistic_epoch_timestamps_stay_bounded(self) -> None:
        """Epoch-scale stamps never leak into time_s."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(
                _REAL_VISUAL_DIAG_LINE
                + "\n"
                + _REAL_LOCALISATION_DIAG_LINE
                + "\n"
            )
            log = log_parser.parse_log_file(log_path)
            for record in (
                *log.visual_records,
                *log.localisation_records,
            ):
                self.assertLess(
                    abs(record.time_s),
                    60.0,
                    "time_s must stay within the log span, "
                    "never near a raw epoch timestamp",
                )

    def test_parse_log_file_takes_no_bag_derived_offset(
        self,
    ) -> None:
        """parse_log_file takes only the log path."""
        import inspect

        parameters = inspect.signature(
            log_parser.parse_log_file
        ).parameters
        self.assertEqual(list(parameters), ["log_path"])


class CalibrationMarkerTests(unittest.TestCase):
    """Tests for calibration-complete lookup."""

    def test_finds_the_marker(self) -> None:
        """Completion line returns log-relative time."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(
                _REAL_CALIBRATION_LINE
                + "\n"
                + _REAL_CALIBRATION_COMPLETE_LINE
                + "\n"
            )
            result = (
                log_parser.find_inertial_calibration_complete_time_s(
                    log_path
                )
            )
            self.assertIsNotNone(result)
            # Expected value derives from the fixture
            # literals to avoid a transcription error.
            first_wall_clock_s = 1790072201.177104622
            complete_wall_clock_s = 1790072201.200000000
            self.assertAlmostEqual(
                result,
                complete_wall_clock_s - first_wall_clock_s,
                places=6,
            )

    def test_finds_the_shortened_marker(self) -> None:
        """Short spelling of the marker is recognised."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(
                "[INFO] [10.0] [inertial_odometry]: "
                "IMU calibrating: 100 samples\n"
                "[INFO] [12.5] [inertial_odometry]: "
                "IMU calibration complete (100 samples)\n"
            )
            result = (
                log_parser.find_inertial_calibration_complete_time_s(
                    log_path
                )
            )
            self.assertAlmostEqual(result, 2.5)

    def test_timeless_compact_lines_give_no_time(self) -> None:
        """Timeless logs give no marker time."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(
                "[INFO] [inertial_odometry]: "
                "IMU calibration complete (100 samples)\n"
            )
            self.assertIsNone(
                log_parser.find_inertial_calibration_complete_time_s(
                    log_path
                )
            )

    def test_returns_none_when_absent(self) -> None:
        """Missing marker returns None, not an error."""
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(_REAL_CALIBRATION_LINE + "\n")
            result = (
                log_parser.find_inertial_calibration_complete_time_s(
                    log_path
                )
            )
            self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
