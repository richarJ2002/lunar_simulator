"""Tests for reporting HTML shell.

Contents:
    RenderNavTests: navigation links.
    EscapingTests: title and warning escaping.
    PlotlyAssetReferenceTests: local asset reference.
    PlotlyDarkModeThemeScriptTests: theme script.
    PlotlyInitialRenderRaceTests: deferred theming.
    WriteReportTests: atomic write and manifest.
    ManifestMergeAcrossRegenerationTests: merge behavior.
    CrossRunOutputDirectoryTests: run identity guard.

Pages reference the shared local Plotly asset and carry
one shared dark-mode theme helper per page.
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import plotly.graph_objects as go

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.data.models import (
    BagIngestResult,
    DiagnosticLog,
    ReportContext,
    ReportPage,
    RunMetadata,
    TopicHealth,
)
from python_tools.reporting import html


def _make_context(
    output_dir: Path, run_name: str = "fake", topic_count: int = 10
) -> ReportContext:
    """Build a minimal ReportContext for shell tests."""
    bag = BagIngestResult(
        start_time_ns=0,
        end_time_ns=1_000_000_000,
        storage_identifier="mcap",
        topic_health={
            "/clock": TopicHealth(
                "/clock",
                "rosgraph_msgs/msg/Clock",
                topic_count,
                0.0,
                1.0,
                10.0,
                0.1,
                0,
            )
        },
        warnings=(),
        odometry={},
        imu={},
        joint_states={},
        wheel_actuators={},
        wheel_scalars={},
        twist_commands={},
        visual_reset=None,
        point_cloud=None,
        images={},
    )
    run_metadata = RunMetadata(
        test_run_dir=Path(f"/tmp/{run_name}"),
        bag_path=Path(f"/tmp/{run_name}/ros/bags/localisation"),
        world="lunar_surface",
        system="alpha",
        command_line=None,
        ros_domain_id=None,
        gz_partition=None,
        recording_profile=None,
        source_revision=None,
        dirty_worktree=None,
        storage_identifier="mcap",
        start_time_ns=0,
        end_time_ns=1_000_000_000,
    )
    return ReportContext(
        run_metadata=run_metadata,
        bag=bag,
        diagnostics=DiagnosticLog(log_path=Path("/tmp/fake.log")),
        parameters={},
        output_dir=output_dir,
        maximum_alignment_gap_s=1.0,
        maximum_image_frames=24,
    )


class RenderNavTests(unittest.TestCase):
    """Tests for render_nav."""

    def test_every_page_is_linked(self) -> None:
        """Every NAV_PAGES entry appears as a link."""
        nav = html.render_nav("index.html")
        for filename, _label in html.NAV_PAGES:
            self.assertIn(f'href="{filename}"', nav)

    def test_active_page_is_marked(self) -> None:
        """Only the active link carries the class."""
        nav = html.render_nav("kalman_filter.html")
        self.assertIn('href="kalman_filter.html" class="active"', nav)
        self.assertNotIn('href="index.html" class="active"', nav)


class EscapingTests(unittest.TestCase):
    """Tests for HTML escaping."""

    def test_title_is_escaped(self) -> None:
        """Title markup is escaped."""
        page_html = html.render_page(
            "<script>evil()</script>", "index.html", "<p>body</p>", []
        )
        self.assertNotIn("<script>evil()</script>", page_html)
        self.assertIn("&lt;script&gt;", page_html)

    def test_warnings_are_escaped(self) -> None:
        """Warning markup is escaped."""
        page_html = html.render_page(
            "Title", "index.html", "",
            ['<img src=x onerror=alert(1)>'],
        )
        self.assertNotIn(
            "<img src=x onerror=alert(1)>", page_html
        )


class PlotlyAssetReferenceTests(unittest.TestCase):
    """Tests for the shared local Plotly asset."""

    def test_references_local_asset(self) -> None:
        """Page references the local Plotly file."""
        page_html = html.render_page("Title", "index.html", "", [])
        self.assertIn(
            f'src="{html.PLOTLY_ASSET_RELATIVE_PATH}"', page_html
        )
        self.assertNotIn("cdn.plot.ly", page_html)


class PlotlyDarkModeThemeScriptTests(unittest.TestCase):
    """Tests for the shared Plotly theme script."""

    def test_script_appears_exactly_once_per_page(self) -> None:
        """Theme helper is defined exactly once."""
        page_html = html.render_page(
            "Title", "index.html", "<p>body</p>", []
        )
        self.assertEqual(
            page_html.count(
                "window.__reportApplyPlotlyTheme = function"
            ),
            1,
        )

    def test_theme_values_match_the_shared_style_source(
        self,
    ) -> None:
        """Theme colors match the page chrome source."""
        script = html._plotly_theme_script()
        self.assertIn(html.CHROME_LIGHT["text_primary"], script)
        self.assertIn(html.CHROME_LIGHT["gridline"], script)
        self.assertIn(html.CHROME_DARK["text_primary"], script)
        self.assertIn(html.CHROME_DARK["gridline"], script)

    def test_responds_live_to_a_theme_change_not_only_at_load(
        self,
    ) -> None:
        """Script listens for live theme changes."""
        script = html._plotly_theme_script()
        self.assertIn("matchMedia", script)
        self.assertIn('addEventListener("change"', script)

    def test_generically_covers_every_axis_not_just_a_single_subplot(
        self,
    ) -> None:
        """Theme covers every axis key generically."""
        script = html._plotly_theme_script()
        self.assertIn("Plotly.relayout", script)
        self.assertIn("[xy]axis", script)

    def test_does_not_eagerly_sweep_the_page_on_load(self) -> None:
        """Shared script never sweeps eagerly on load."""
        # Eager sweeps raced figure promises, so the sweep
        # is only referenced, never invoked as a statement.
        script = html._plotly_theme_script()
        self.assertNotIn("applyToEveryRenderedFigure();", script)


class PlotlyInitialRenderRaceTests(unittest.TestCase):
    """Tests for deferred per-figure theming."""

    def _fragment_for(self, element_id: str) -> str:
        """Render a minimal figure fragment."""
        figure = go.Figure(data=[go.Scatter(x=[0, 1], y=[0, 1])])
        return html.figure_to_fragment(figure, element_id)

    def test_theme_call_is_chained_after_newplot_not_alongside_it(
        self,
    ) -> None:
        """Theme call sits inside .then()."""
        fragment = self._fragment_for("race-fig")
        then_index = fragment.find(").then(function(){")
        theme_call_index = fragment.find("__reportApplyPlotlyTheme")
        self.assertNotEqual(
            then_index, -1, "post_script was not chained via .then()"
        )
        self.assertGreater(
            theme_call_index,
            then_index,
            "theme call must be inside .then(), not before it",
        )

    def test_theme_call_targets_this_figures_own_div(self) -> None:
        """Chained call targets its own div."""
        fragment = self._fragment_for("race-fig-42")
        self.assertIn(
            'document.getElementById("race-fig-42")', fragment
        )
        self.assertNotIn("querySelectorAll", fragment)

    def test_theme_call_is_guarded_against_a_missing_helper(
        self,
    ) -> None:
        """Missing helper no-ops rather than throws."""
        fragment = self._fragment_for("race-fig")
        self.assertIn(
            "if (window.__reportApplyPlotlyTheme)", fragment
        )

    def test_theme_helper_is_defined_before_the_first_figure(
        self,
    ) -> None:
        """Helper is defined before the first figure."""
        body = self._fragment_for("fig-a") + self._fragment_for(
            "fig-b"
        )
        page_html = html.render_page(
            "Title", "index.html", body, []
        )
        helper_index = page_html.find(
            "window.__reportApplyPlotlyTheme = function"
        )
        first_figure_index = page_html.find("plotly-graph-div")
        self.assertNotEqual(helper_index, -1)
        self.assertNotEqual(first_figure_index, -1)
        self.assertLess(helper_index, first_figure_index)

    def test_element_ids_needing_js_escaping_do_not_break_the_script(
        self,
    ) -> None:
        """Quoted ids stay inside the string literal."""
        fragment = self._fragment_for('odd"id\\name')
        self.assertIn(json.dumps('odd"id\\name'), fragment)


class WriteReportTests(unittest.TestCase):
    """Tests for atomic report writing."""

    def setUp(self) -> None:
        """Create a fresh temporary output directory."""
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """Remove the temporary output directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def test_writes_every_page_and_shared_assets(self) -> None:
        """Pages and assets land with no staging left."""
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page(
                "Kalman Filter",
                "kalman_filter.html",
                "<p>x</p>",
                [],
            ),
            summary_stats={},
            warnings=(),
        )
        html.write_report([page], self.output_dir, context)
        self.assertTrue(
            (self.output_dir / "kalman_filter.html").is_file()
        )
        self.assertTrue(
            (self.output_dir / "assets" / "plotly.min.js").is_file()
        )
        self.assertTrue(
            (self.output_dir / "assets" / "report.css").is_file()
        )
        self.assertFalse(
            (self.output_dir / ".post_processing_staging").exists()
        )

    def test_preserves_unrelated_analyst_file(self) -> None:
        """Unrelated analyst files are preserved."""
        self.output_dir.mkdir(parents=True)
        analyst_file = self.output_dir / "my_notes.txt"
        analyst_file.write_text("keep me")
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page(
                "Kalman Filter",
                "kalman_filter.html",
                "<p>x</p>",
                [],
            ),
            summary_stats={},
            warnings=(),
        )
        html.write_report([page], self.output_dir, context)
        self.assertTrue(analyst_file.is_file())
        self.assertEqual(analyst_file.read_text(), "keep me")

    def test_manifest_contents(self) -> None:
        """Manifest carries version, pages, and counts."""
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page(
                "Kalman Filter",
                "kalman_filter.html",
                "<p>x</p>",
                [],
            ),
            summary_stats={},
            warnings=("some warning",),
        )
        html.write_report([page], self.output_dir, context)
        manifest = json.loads(
            (self.output_dir / "report_manifest.json").read_text()
        )
        self.assertEqual(
            manifest["generator_version"], html.GENERATOR_VERSION
        )
        self.assertEqual(
            manifest["generated_pages"], ["kalman_filter.html"]
        )
        self.assertEqual(manifest["topic_counts"]["/clock"], 10)
        self.assertAlmostEqual(
            manifest["maximum_alignment_gap_s"], 1.0
        )
        self.assertIn(
            "kalman_filter.html: some warning",
            manifest["omissions"],
        )


class ManifestMergeAcrossRegenerationTests(unittest.TestCase):
    """Tests for manifest merging across regeneration."""

    def setUp(self) -> None:
        """Create a fresh temporary output directory."""
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """Remove the temporary output directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def _page(self, filename: str, warnings: tuple = ()) -> ReportPage:
        return ReportPage(
            filename=filename,
            title=filename,
            html=html.render_page(
                filename, filename, "<p>x</p>", list(warnings)
            ),
            summary_stats={},
            warnings=warnings,
        )

    def test_standalone_regeneration_preserves_other_pages(
        self,
    ) -> None:
        """Regeneration keeps sibling pages in the manifest."""
        context = _make_context(self.output_dir)
        aggregate_pages = [
            self._page("index.html"),
            self._page(
                "kalman_filter.html",
                warnings=("kalman warning",),
            ),
            self._page("visual_odometry.html"),
            self._page("inertial_odometry.html"),
            self._page(
                "wheel_odometry.html",
                warnings=("wheel warning",),
            ),
        ]
        html.write_report(aggregate_pages, self.output_dir, context)

        # Standalone rewrite of one page, as its own main does.
        standalone_page = self._page(
            "wheel_odometry.html",
            warnings=("new wheel warning",),
        )
        html.write_report(
            [standalone_page], self.output_dir, context
        )

        for filename in (
            "index.html",
            "kalman_filter.html",
            "visual_odometry.html",
            "inertial_odometry.html",
            "wheel_odometry.html",
        ):
            self.assertTrue((self.output_dir / filename).is_file())

        manifest = json.loads(
            (self.output_dir / "report_manifest.json").read_text()
        )
        self.assertEqual(
            set(manifest["generated_pages"]),
            {
                "index.html",
                "kalman_filter.html",
                "visual_odometry.html",
                "inertial_odometry.html",
                "wheel_odometry.html",
            },
        )
        # Kalman omission survives untouched.
        self.assertIn(
            "kalman_filter.html: kalman warning",
            manifest["omissions"],
        )
        # Wheel omission reflects the regenerated page.
        self.assertIn(
            "wheel_odometry.html: new wheel warning",
            manifest["omissions"],
        )
        self.assertNotIn(
            "wheel_odometry.html: wheel warning",
            manifest["omissions"],
        )

    def test_a_manually_deleted_page_is_dropped_from_the_manifest(
        self,
    ) -> None:
        """Deleted pages leave the manifest."""
        context = _make_context(self.output_dir)
        html.write_report(
            [
                self._page("kalman_filter.html"),
                self._page("wheel_odometry.html"),
            ],
            self.output_dir,
            context,
        )
        (self.output_dir / "wheel_odometry.html").unlink()

        html.write_report(
            [self._page("kalman_filter.html")],
            self.output_dir,
            context,
        )

        manifest = json.loads(
            (self.output_dir / "report_manifest.json").read_text()
        )
        self.assertEqual(
            manifest["generated_pages"], ["kalman_filter.html"]
        )


class CrossRunOutputDirectoryTests(unittest.TestCase):
    """Tests for the cross-run output guard."""

    def setUp(self) -> None:
        """Create a fresh temporary output directory."""
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """Remove the temporary output directory."""
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def _page(self, filename: str, warnings: tuple = ()) -> ReportPage:
        return ReportPage(
            filename=filename,
            title=filename,
            html=html.render_page(
                filename, filename, "<p>x</p>", list(warnings)
            ),
            summary_stats={},
            warnings=warnings,
        )

    def test_standalone_into_an_empty_directory_succeeds(
        self,
    ) -> None:
        """First write into an empty directory succeeds."""
        context = _make_context(
            self.output_dir, run_name="run_only"
        )
        html.write_report(
            [self._page("wheel_odometry.html")],
            self.output_dir,
            context,
        )
        self.assertTrue(
            (self.output_dir / "wheel_odometry.html").is_file()
        )
        manifest = json.loads(
            (self.output_dir / "report_manifest.json").read_text()
        )
        self.assertEqual(
            manifest["generated_pages"], ["wheel_odometry.html"]
        )
        self.assertEqual(
            manifest["run_identity"]["test_run_dir"], "/tmp/run_only"
        )

    def test_run_b_targeting_run_as_report_directory_fails(
        self,
    ) -> None:
        """Second run reusing the directory is refused."""
        context_a = _make_context(
            self.output_dir, run_name="run_A", topic_count=500
        )
        html.write_report(
            [
                self._page("index.html"),
                self._page("kalman_filter.html"),
                self._page("visual_odometry.html"),
                self._page("inertial_odometry.html"),
                self._page(
                    "wheel_odometry.html",
                    warnings=("A-specific warning",),
                ),
            ],
            self.output_dir,
            context_a,
        )

        context_b = _make_context(
            self.output_dir, run_name="run_B", topic_count=999
        )
        with self.assertRaises(
            html.CrossRunOutputDirectoryError
        ) as raised:
            html.write_report(
                [
                    self._page(
                        "wheel_odometry.html",
                        warnings=("B-specific warning",),
                    )
                ],
                self.output_dir,
                context_b,
            )
        # The error names the directory and both runs.
        message = str(raised.exception)
        self.assertIn(str(self.output_dir), message)
        self.assertIn("run_A", message)
        self.assertIn("run_B", message)

    def test_failed_cross_run_attempt_leaves_every_file_unchanged(
        self,
    ) -> None:
        """Rejected cross-run write changes nothing."""
        context_a = _make_context(
            self.output_dir, run_name="run_A"
        )
        html.write_report(
            [self._page("index.html"), self._page("kalman_filter.html")],
            self.output_dir,
            context_a,
        )
        before = {
            path.relative_to(self.output_dir): path.read_bytes()
            for path in sorted(self.output_dir.rglob("*"))
            if path.is_file()
        }

        context_b = _make_context(
            self.output_dir, run_name="run_B"
        )
        with self.assertRaises(html.CrossRunOutputDirectoryError):
            html.write_report(
                [self._page("kalman_filter.html")],
                self.output_dir,
                context_b,
            )

        after = {
            path.relative_to(self.output_dir): path.read_bytes()
            for path in sorted(self.output_dir.rglob("*"))
            if path.is_file()
        }
        self.assertEqual(before, after)
        self.assertFalse(
            (self.output_dir / ".post_processing_staging").exists()
        )

    def test_manifest_missing_identity_with_generated_pages_is_not_merged(
        self,
    ) -> None:
        """Manifest without identity is not merged."""
        self.output_dir.mkdir(parents=True)
        (self.output_dir / "kalman_filter.html").write_text(
            "<html>old</html>"
        )
        (self.output_dir / "report_manifest.json").write_text(
            json.dumps(
                {"generated_pages": ["kalman_filter.html"],
                 "omissions": []}
            )
        )

        context = _make_context(self.output_dir, run_name="run_new")
        with self.assertRaises(
            html.CrossRunOutputDirectoryError
        ) as raised:
            html.write_report(
                [self._page("kalman_filter.html")],
                self.output_dir,
                context,
            )
        self.assertIn(
            "does not record which run", str(raised.exception)
        )
        # Pre-existing files stay untouched.
        self.assertEqual(
            (self.output_dir / "kalman_filter.html").read_text(),
            "<html>old</html>",
        )


if __name__ == "__main__":
    unittest.main()
