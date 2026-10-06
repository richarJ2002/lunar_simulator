"""Tests for report context helpers.

Contents:
    BuildCommonParserTests: shared flag defaults.
    LoadParameterSnapshotsTests: missing file tolerance.
    BuildReportContextValidationTests: validation chain.
    EntryPointCliErrorHandlingTests: invalid run handling.

Entry points report exit code 1 for an invalid run
rather than an uncaught traceback.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools import context
from python_tools.bag.reader import BagValidationError


class BuildCommonParserTests(unittest.TestCase):
    """Tests for build_common_parser flag defaults."""

    def test_defaults(self) -> None:
        """Optional flags carry documented defaults."""
        parser = context.build_common_parser()
        # add_help=False parents cannot parse alone, so wrap
        # the way every real entry point does.
        import argparse

        child = argparse.ArgumentParser(parents=[parser])
        args = child.parse_args(["--test-run", "test_runs/example"])
        self.assertEqual(args.test_run, Path("test_runs/example"))
        self.assertIsNone(args.output_dir)
        self.assertIsNone(args.bag)
        from python_tools.bag.reader import (
            DEFAULT_MAXIMUM_ALIGNMENT_GAP_S,
            DEFAULT_MAXIMUM_IMAGE_FRAMES,
        )

        self.assertEqual(
            args.maximum_alignment_gap_s,
            DEFAULT_MAXIMUM_ALIGNMENT_GAP_S,
        )
        self.assertEqual(
            args.maximum_image_frames,
            DEFAULT_MAXIMUM_IMAGE_FRAMES,
        )

    def test_missing_required_test_run_exits_nonzero(self) -> None:
        """Missing --test-run is an argparse error."""
        import argparse

        child = argparse.ArgumentParser(
            parents=[context.build_common_parser()]
        )
        with self.assertRaises(SystemExit) as raised:
            child.parse_args([])
        self.assertNotEqual(raised.exception.code, 0)


class LoadParameterSnapshotsTests(unittest.TestCase):
    """Tests for load_parameter_snapshots tolerance."""

    def test_missing_test_run_yields_empty_snapshots(self) -> None:
        """No parameter files yields empty snapshots."""
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(
                context.load_parameter_snapshots(Path(tmp)), {}
            )

    def test_loads_a_present_file(self) -> None:
        """Well-formed file loads under its node name."""
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            # Second entry is the Kalman filter file.
            relative_path, node_name = context.PARAMETER_FILES[1]
            file_path = test_run_dir / relative_path
            file_path.parent.mkdir(parents=True)
            file_path.write_text(
                f"{node_name}:\n"
                "  ros__parameters:\n"
                "    prediction_rate_hz: 100.0\n"
            )
            snapshots = context.load_parameter_snapshots(
                test_run_dir
            )
            self.assertIn(node_name, snapshots)
            self.assertEqual(
                snapshots[node_name].parameters[
                    "prediction_rate_hz"
                ],
                100.0,
            )

    def test_malformed_file_is_skipped_not_raised(self) -> None:
        """File without ros__parameters is skipped."""
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            relative_path, node_name = context.PARAMETER_FILES[1]
            file_path = test_run_dir / relative_path
            file_path.parent.mkdir(parents=True)
            file_path.write_text("not_the_expected_node: {}\n")
            snapshots = context.load_parameter_snapshots(
                test_run_dir
            )
            self.assertNotIn(node_name, snapshots)


class BuildReportContextValidationTests(unittest.TestCase):
    """Tests for build_report_context validation."""

    def test_output_dir_as_existing_file_raises(self) -> None:
        """Output path as a file raises an error."""
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp) / "run"
            (test_run_dir / "parameters").mkdir(parents=True)
            (test_run_dir / "ros" / "bags" / "localisation").mkdir(
                parents=True
            )
            metadata = (
                test_run_dir / "ros" / "bags" / "localisation"
                / "metadata.yaml"
            )
            metadata.write_text("")
            output_dir_as_file = Path(tmp) / "not_a_directory"
            output_dir_as_file.write_text("")
            with self.assertRaises(BagValidationError):
                context.build_report_context(
                    test_run_dir,
                    output_dir_as_file,
                    None,
                    1.0,
                    24,
                )


class EntryPointCliErrorHandlingTests(unittest.TestCase):
    """Tests for entry-point invalid-run handling."""

    def test_every_entry_point_handles_invalid_test_run(
        self,
    ) -> None:
        """Each entry point returns 1 for a bad run."""
        import post_processing
        import post_processing_inertial_odometry
        import post_processing_kalman_filter
        import post_processing_visual_odometry
        import post_processing_wheel_odometry

        for module in (
            post_processing,
            post_processing_inertial_odometry,
            post_processing_kalman_filter,
            post_processing_visual_odometry,
            post_processing_wheel_odometry,
        ):
            with self.subTest(module=module.__name__):
                exit_code = module.main(
                    ["--test-run", "/nonexistent/path/for/test"]
                )
                self.assertEqual(exit_code, 1)


if __name__ == "__main__":
    unittest.main()
