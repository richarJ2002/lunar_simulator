"""!
@brief  Tests for python_tools.diagnostics.log_parser against
        representative current log lines (captured from a real run's
        alpha_node log), plus boundary cases: an unparsed tagged line, a
        missing calibration marker, and log-relative timing (elapsed
        seconds since a log file's own first record -- see
        log_parser's module docstring for why this is deliberately not
        aligned to any bag's elapsed-simulated-time axis).
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.diagnostics import log_parser

# Representative lines captured from a real run's alpha_node log
# (test_runs/2026-09-22-12-16-30), used verbatim so the regexes are
# checked against real node output, not a hand-written approximation.
# Their bracketed timestamps (~1.79e9) are real wall-clock epoch seconds,
# not simulated time -- exactly the scale of value that a previous, now
# removed, bag-offset-based alignment attempt mishandled (see Defect 1 in
# the plan's 2026-09-22 audit).
_REAL_VISUAL_DIAG_LINE = (
    "[INFO] [1790072206.200122095] [visual_odometry]: visual_diag received=6 "
    "accepted=6 failed=0 reception_rate_hz=1.200 accepted_rate_hz=1.200 "
    "age_s=0.215000 processing_ms=103.432 conversion_ms=1.703 disparity_ms=33.508 "
    "detection_ms=32.350 tracking_ms=32.880 reconstruction_ms=0.012 pnp_ms=0.241 "
    "detected=240 tracked=196 stereo_valid=126 correspondences=114 inliers=114 "
    "occupancy=0.250 near_mid_ratio=0.632 "
    "disparity_p10_p50_p90=[9.269,20.906,33.750] "
    "depth_p10_p50_p90_m=[3.558,5.742,12.947] reprojection_rms_px=0.001 "
    "normal_condition=5.000e+02 accepted_interval_s=0.400000 consecutive_failures=0"
)
_REAL_LOCALISATION_DIAG_LINE = (
    "[INFO] [1790072206.208832529] [continuous_ekf]: localisation_diag "
    "source=imu received=211 accepted=211 age_rejected=0 nis_rejected=0 "
    "numerical_rejected=0 fused=112 publication=7.960000 admission=7.960000 "
    "start=7.960000 end=7.960000 estimator_epoch=7.960000 nis=0.000000 "
    "correction_norm=0.000000 covariance_trace=0.329332 "
    "covariance_min_eigenvalue=2.334989e-05 "
    "covariance_diagonal_range=[7.797882e-04,8.578001e-02] "
    "quaternion_norm=1.000000000000 "
    "accel_bias_body_mps2=[0.004082,-0.007037,0.028006] "
    "gyro_bias_body_radps=[0.002169,-0.000285,0.001520]"
)
_REAL_CALIBRATION_LINE = (
    "[INFO] [1790072201.177104622] [inertial_odometry]: IMU calibration "
    "requires a stationary rover for 100 samples; filtered samples are "
    "temporally correlated"
)
_REAL_CALIBRATION_COMPLETE_LINE = (
    "[INFO] [1790072201.200000000] [inertial_odometry]: IMU stationary "
    "calibration complete (100 samples)"
)


class ParseVisualDiagTests(unittest.TestCase):
    """!
    @brief  Tests for visual_diag parsing.
    """

    def _write_and_parse(self, lines: list[str]):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text("\n".join(lines) + "\n")
            return log_parser.parse_log_file(log_path)

    def test_parses_real_line_fields(self) -> None:
        """!
        @brief  Every field of a real visual_diag line is extracted
                correctly.
        """
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
        """!
        @brief  An ordinary log line (no diagnostic tag) parses to zero
                records and zero unparsed-line count.
        """
        log = self._write_and_parse(["[INFO] [1.0] [some_node]: ordinary startup message"])
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(len(log.localisation_records), 0)
        self.assertEqual(log.unparsed_line_count, 0)

    def test_malformed_tagged_line_counts_as_unparsed(self) -> None:
        """!
        @brief  A line that starts with the visual_diag tag but does not
                match the expected field format is counted as unparsed
                rather than silently dropped or crashing.
        """
        log = self._write_and_parse(
            ["[INFO] [1.0] [visual_odometry]: visual_diag some_future_field=1"]
        )
        self.assertEqual(len(log.visual_records), 0)
        self.assertEqual(log.unparsed_line_count, 1)


class ParseLocalisationDiagTests(unittest.TestCase):
    """!
    @brief  Tests for localisation_diag parsing.
    """

    def _write_and_parse(self, lines: list[str]):
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text("\n".join(lines) + "\n")
            return log_parser.parse_log_file(log_path)

    def test_parses_real_line_fields(self) -> None:
        """!
        @brief  Every field of a real localisation_diag line is extracted
                correctly, including the bias vectors.
        """
        log = self._write_and_parse([_REAL_LOCALISATION_DIAG_LINE])
        self.assertEqual(len(log.localisation_records), 1)
        record = log.localisation_records[0]
        self.assertEqual(record.source, "imu")
        self.assertEqual(record.received, 211)
        self.assertEqual(record.fused, 112)
        self.assertAlmostEqual(record.quaternion_norm, 1.0, places=9)
        self.assertAlmostEqual(record.accel_bias_body_mps2[1], -0.007037)
        self.assertAlmostEqual(record.gyro_bias_body_radps[2], 0.001520)

    def test_multiple_sources_all_parsed(self) -> None:
        """!
        @brief  Three consecutive per-source lines all parse, distinct by
                source.
        """
        lines = [
            _REAL_LOCALISATION_DIAG_LINE,
            _REAL_LOCALISATION_DIAG_LINE.replace("source=imu", "source=visual"),
            _REAL_LOCALISATION_DIAG_LINE.replace("source=imu", "source=wheel"),
        ]
        log = self._write_and_parse(lines)
        self.assertEqual(
            [record.source for record in log.localisation_records],
            ["imu", "visual", "wheel"],
        )


class LogRelativeTimingTests(unittest.TestCase):
    """!
    @brief  Tests for log_relative_time_s: elapsed seconds since a log
            file's own first parsed record. This is the derivation-and-
            application regression coverage for Defect 1 in the plan's
            2026-09-22 audit: `parse_log_file` no longer accepts, derives,
            or applies any bag-based wall-clock<->simulated-time offset at
            all, so a case that used to silently produce billion-second
            elapsed values (a real bag's simulated receive time minus a
            real log's wall-clock timestamp) cannot recur -- there is no
            longer any code path that combines those two quantities.
    """

    def test_first_record_is_zero_and_second_reflects_the_wall_clock_gap(self) -> None:
        """!
        @brief  The first record's log_relative_time_s is 0, and a second
                record five wall-clock seconds later reads 5.0 -- ordering
                and spacing are preserved from the raw log.
        """
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            first = f"[INFO] [10.000000000] [visual_odometry]: {_REAL_VISUAL_DIAG_LINE.split(': ', 1)[1]}"
            second = f"[INFO] [15.000000000] [visual_odometry]: {_REAL_VISUAL_DIAG_LINE.split(': ', 1)[1]}"
            log_path.write_text(first + "\n" + second + "\n")
            log = log_parser.parse_log_file(log_path)
            self.assertAlmostEqual(log.visual_records[0].log_relative_time_s, 0.0, places=6)
            self.assertAlmostEqual(log.visual_records[1].log_relative_time_s, 5.0, places=6)

    def test_realistic_epoch_scale_timestamps_never_produce_a_billion_second_value(self) -> None:
        """!
        @brief  Regression for Defect 1: even though the real log lines'
                bracketed timestamps are ~1.79e9 (real wall-clock epoch
                seconds -- the exact scale that leaked into a real
                generated report's Plotly x-axis before this fix),
                log_relative_time_s for every record stays small and
                bounded by the log's own span, not anywhere near the raw
                epoch value.
        """
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(_REAL_VISUAL_DIAG_LINE + "\n" + _REAL_LOCALISATION_DIAG_LINE + "\n")
            log = log_parser.parse_log_file(log_path)
            for record in (*log.visual_records, *log.localisation_records):
                self.assertLess(
                    abs(record.log_relative_time_s),
                    60.0,
                    "log_relative_time_s must stay within the log's own short "
                    "span, never anywhere near a raw epoch timestamp",
                )

    def test_parse_log_file_takes_no_bag_derived_offset(self) -> None:
        """!
        @brief  `parse_log_file` no longer accepts a bag start time or a
                wall-to-sim offset -- the invalid API surface from Defect 1
                cannot be reintroduced by a caller.
        """
        import inspect

        parameters = inspect.signature(log_parser.parse_log_file).parameters
        self.assertEqual(list(parameters), ["log_path"])


class CalibrationMarkerTests(unittest.TestCase):
    """!
    @brief  Tests for `find_inertial_calibration_complete_time_s`.
    """

    def test_finds_the_marker(self) -> None:
        """!
        @brief  The one-time calibration-complete line is found and its
                log-relative time returned.
        """
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(_REAL_CALIBRATION_LINE + "\n" + _REAL_CALIBRATION_COMPLETE_LINE + "\n")
            result = log_parser.find_inertial_calibration_complete_time_s(log_path)
            self.assertIsNotNone(result)
            # The complete line's own bracketed timestamp minus the first
            # line's (this log-relative axis's own zero point), derived
            # from the fixture literals rather than hand-computed to avoid
            # a transcription error in the expected value.
            first_wall_clock_s = 1790072201.177104622
            complete_wall_clock_s = 1790072201.200000000
            self.assertAlmostEqual(
                result, complete_wall_clock_s - first_wall_clock_s, places=6
            )

    def test_returns_none_when_absent(self) -> None:
        """!
        @brief  A log with no completion marker (e.g. calibration never
                finished) returns `None`, not an error.
        """
        with tempfile.TemporaryDirectory() as tmp:
            log_path = Path(tmp) / "alpha_node_1_1.log"
            log_path.write_text(_REAL_CALIBRATION_LINE + "\n")
            result = log_parser.find_inertial_calibration_complete_time_s(log_path)
            self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
