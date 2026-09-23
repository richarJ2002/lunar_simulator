"""!
@brief  Tests for python_tools.context: the shared argument parser's
        defaults, `load_parameter_snapshots`'s tolerance of a missing or
        malformed parameter file, `build_report_context`'s validation
        chain, and each entry-point script's CLI error handling for an
        invalid --test-run.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools import context
from python_tools.bag.reader import BagValidationError


class BuildCommonParserTests(unittest.TestCase):
    """!
    @brief  Tests for `build_common_parser`'s shared flag defaults.
    """

    def test_defaults(self) -> None:
        """!
        @brief  Every optional flag has its documented default when only
                the required --test-run is given.
        """
        parser = context.build_common_parser()
        # add_help=False parent parsers cannot be parsed alone (no -h), so
        # wrap in a child parser the way every real entry point does.
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

        self.assertEqual(args.maximum_alignment_gap_s, DEFAULT_MAXIMUM_ALIGNMENT_GAP_S)
        self.assertEqual(args.maximum_image_frames, DEFAULT_MAXIMUM_IMAGE_FRAMES)

    def test_missing_required_test_run_exits_nonzero(self) -> None:
        """!
        @brief  Omitting the required --test-run flag is an argparse
                error (SystemExit with a nonzero code), not a silent
                default.
        """
        import argparse

        child = argparse.ArgumentParser(parents=[context.build_common_parser()])
        with self.assertRaises(SystemExit) as raised:
            child.parse_args([])
        self.assertNotEqual(raised.exception.code, 0)


class LoadParameterSnapshotsTests(unittest.TestCase):
    """!
    @brief  Tests for `load_parameter_snapshots`'s tolerance of missing or
            malformed files.
    """

    def test_missing_test_run_yields_empty_snapshots(self) -> None:
        """!
        @brief  A test run with no parameters/ files at all yields an
                empty (not erroring) snapshot dict.
        """
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(context.load_parameter_snapshots(Path(tmp)), {})

    def test_loads_a_present_file(self) -> None:
        """!
        @brief  A present, well-formed parameter file is loaded under its
                registered node name.
        """
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            relative_path, node_name = context.PARAMETER_FILES[1]  # alpha_kalman_filter
            file_path = test_run_dir / relative_path
            file_path.parent.mkdir(parents=True)
            file_path.write_text(f"{node_name}:\n  ros__parameters:\n    prediction_rate_hz: 100.0\n")
            snapshots = context.load_parameter_snapshots(test_run_dir)
            self.assertIn(node_name, snapshots)
            self.assertEqual(snapshots[node_name].parameters["prediction_rate_hz"], 100.0)

    def test_malformed_file_is_skipped_not_raised(self) -> None:
        """!
        @brief  A parameter file present but missing its expected
                ros__parameters section is skipped rather than raising.
        """
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp)
            relative_path, node_name = context.PARAMETER_FILES[1]
            file_path = test_run_dir / relative_path
            file_path.parent.mkdir(parents=True)
            file_path.write_text("not_the_expected_node: {}\n")
            snapshots = context.load_parameter_snapshots(test_run_dir)
            self.assertNotIn(node_name, snapshots)


class BuildReportContextValidationTests(unittest.TestCase):
    """!
    @brief  Tests for `build_report_context`'s validation chain, driven
            through the module every entry point actually calls.
    """

    def test_output_dir_as_existing_file_raises(self) -> None:
        """!
        @brief  Passing --output-dir pointing at an existing file (not a
                directory) raises an actionable error.
        """
        with tempfile.TemporaryDirectory() as tmp:
            test_run_dir = Path(tmp) / "run"
            (test_run_dir / "parameters").mkdir(parents=True)
            (test_run_dir / "ros" / "bags" / "localisation").mkdir(parents=True)
            (test_run_dir / "ros" / "bags" / "localisation" / "metadata.yaml").write_text("")
            output_dir_as_file = Path(tmp) / "not_a_directory"
            output_dir_as_file.write_text("")
            with self.assertRaises(BagValidationError):
                context.build_report_context(test_run_dir, output_dir_as_file, None, 1.0, 24)


class EntryPointCliErrorHandlingTests(unittest.TestCase):
    """!
    @brief  Tests that every entry-point script's `main()` reports a
            clean, non-crashing failure (exit code 1) for an invalid
            --test-run, rather than an uncaught traceback.
    """

    def test_every_entry_point_handles_invalid_test_run(self) -> None:
        """!
        @brief  Each of the five entry-point scripts' `main()` returns 1
                for a nonexistent --test-run path.
        """
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
                exit_code = module.main(["--test-run", "/nonexistent/path/for/test"])
                self.assertEqual(exit_code, 1)


if __name__ == "__main__":
    unittest.main()
