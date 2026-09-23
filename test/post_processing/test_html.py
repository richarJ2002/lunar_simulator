"""!
@brief  Tests for python_tools.reporting.html: generated navigation links,
        the local Plotly asset reference, HTML-escaped metadata, manifest
        contents, and atomic report writing.
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import plotly.graph_objects as go

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

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
    """!
    @brief   Builds a minimal `ReportContext` for HTML-shell tests.

    @param   output_dir
             Directory the context's report would be written to.
    @param   run_name
             Distinguishes one synthetic run's `test_run_dir`/`bag_path`
             identity from another's, for cross-run tests
             (`_ensure_same_run_or_raise`); defaults to the identity every
             existing test in this module already used before this
             parameter was added, so those callers are unaffected.
    @param   topic_count
             The lone `/clock` topic's message count, varied by cross-run
             tests to make two "different runs" also carry different
             `topic_counts` in their manifests.

    @return  The assembled `ReportContext`.
    """
    bag = BagIngestResult(
        start_time_ns=0,
        end_time_ns=1_000_000_000,
        storage_identifier="mcap",
        topic_health={
            "/clock": TopicHealth(
                "/clock", "rosgraph_msgs/msg/Clock", topic_count, 0.0, 1.0, 10.0, 0.1, 0
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
    """!
    @brief  Tests for `render_nav`.
    """

    def test_every_page_is_linked(self) -> None:
        """!
        @brief  Every entry in NAV_PAGES appears as a link.
        """
        nav = html.render_nav("index.html")
        for filename, _label in html.NAV_PAGES:
            self.assertIn(f'href="{filename}"', nav)

    def test_active_page_is_marked(self) -> None:
        """!
        @brief  Only the active page's link carries the active class.
        """
        nav = html.render_nav("kalman_filter.html")
        self.assertIn('href="kalman_filter.html" class="active"', nav)
        self.assertNotIn('href="index.html" class="active"', nav)


class EscapingTests(unittest.TestCase):
    """!
    @brief  Tests that user/run-controlled text is HTML-escaped before
            embedding.
    """

    def test_title_is_escaped(self) -> None:
        """!
        @brief  A title containing HTML-significant characters is escaped
                in the rendered page.
        """
        page_html = html.render_page("<script>evil()</script>", "index.html", "<p>body</p>", [])
        self.assertNotIn("<script>evil()</script>", page_html)
        self.assertIn("&lt;script&gt;", page_html)

    def test_warnings_are_escaped(self) -> None:
        """!
        @brief  A warning message containing HTML-significant characters
                is escaped, not injected verbatim.
        """
        page_html = html.render_page("Title", "index.html", "", ['<img src=x onerror=alert(1)>'])
        self.assertNotIn("<img src=x onerror=alert(1)>", page_html)


class PlotlyAssetReferenceTests(unittest.TestCase):
    """!
    @brief  Tests that every rendered page references the shared local
            Plotly asset, never a CDN.
    """

    def test_references_local_asset(self) -> None:
        """!
        @brief  The page's <head> references the local plotly.min.js path.
        """
        page_html = html.render_page("Title", "index.html", "", [])
        self.assertIn(f'src="{html.PLOTLY_ASSET_RELATIVE_PATH}"', page_html)
        self.assertNotIn("cdn.plot.ly", page_html)


class PlotlyDarkModeThemeScriptTests(unittest.TestCase):
    """!
    @brief  Tests for the shared dark-mode Plotly re-theming script
            (Defect 5 in the plan's 2026-09-22 audit: a figure's own
            internal text/gridline/axis colors were hardcoded to light
            mode, unlike the page chrome, which already responds to
            `prefers-color-scheme`).
    """

    def test_script_appears_exactly_once_per_page(self) -> None:
        """!
        @brief  One shared script defines the re-theming helper -- it is
                not duplicated per figure. (Renamed from the removed
                top-level `applyPlotlyTheme` function as part of the
                2026-09-23 rendering-defect fix: see
                `PlotlyInitialRenderRaceTests` below for why the helper is
                now a named, externally-callable function rather than a
                page-load-only closure.)
        """
        page_html = html.render_page("Title", "index.html", "<p>body</p>", [])
        self.assertEqual(page_html.count("window.__reportApplyPlotlyTheme = function"), 1)

    def test_theme_values_match_the_shared_style_source(self) -> None:
        """!
        @brief  The embedded light/dark colors come from CHROME_LIGHT/
                CHROME_DARK -- the same source build_css() uses for the
                page chrome -- so the two can never drift apart.
        """
        script = html._plotly_theme_script()
        self.assertIn(html.CHROME_LIGHT["text_primary"], script)
        self.assertIn(html.CHROME_LIGHT["gridline"], script)
        self.assertIn(html.CHROME_DARK["text_primary"], script)
        self.assertIn(html.CHROME_DARK["gridline"], script)

    def test_responds_live_to_a_theme_change_not_only_at_load(self) -> None:
        """!
        @brief  The script listens for a live `prefers-color-scheme`
                change, not just a one-time choice baked in when the page
                was generated.
        """
        script = html._plotly_theme_script()
        self.assertIn("matchMedia", script)
        self.assertIn('addEventListener("change"', script)

    def test_generically_covers_every_axis_not_just_a_single_subplot(self) -> None:
        """!
        @brief  The re-theming logic discovers every `xaxis`/`yaxis`-style
                key on a figure's own layout (covering both a single-axis
                figure and a multi-row subplot figure such as
                `three_axis_time_series`) rather than hardcoding just
                `xaxis`/`yaxis`.
        """
        script = html._plotly_theme_script()
        self.assertIn("Plotly.relayout", script)
        self.assertIn("[xy]axis", script)

    def test_does_not_eagerly_sweep_the_page_on_load(self) -> None:
        """!
        @brief  The shared script must not call its own page-wide sweep
                function at parse time any more (2026-09-23 fix): doing so
                was the root cause of the initial-render race below, since
                it ran `Plotly.relayout()` against every `.plotly-graph-
                div` already on the page regardless of whether each one's
                own `Plotly.newPlot()` promise had actually settled. The
                sweep function must only ever be *referenced* (passed to
                `addEventListener`) or *defined*, never invoked as a bare
                statement.
        """
        script = html._plotly_theme_script()
        self.assertNotIn("applyToEveryRenderedFigure();", script)


class PlotlyInitialRenderRaceTests(unittest.TestCase):
    """!
    @brief  Regression tests for the 2026-09-23 rendering defect: figures
            intermittently rendered blank on first load (readable only
            after a user interaction such as a double-click or drag)
            because the shared theme script called `Plotly.relayout()`
            against every figure eagerly, as soon as its own `<script>`
            element ran -- which is not guaranteed to be after that
            figure's own `Plotly.newPlot()` promise has settled (confirmed
            directly: `Figure.to_html()`'s generated call is a bare,
            unchained `Plotly.newPlot(...)`, and replaying that exact call
            against the real installed Plotly bundle shows its promise
            needs multiple microtask turns to settle -- it is not resolved
            in the same synchronous pass the old script relied on).

            The fix instead defers each figure's own initial theming until
            that figure's own `Plotly.newPlot()` promise resolves, using
            `plotly.io.to_html`'s own supported `post_script` mechanism
            (which chains via `.then()`), rather than inferring readiness
            from `<script>` tag order.
    """

    def _fragment_for(self, element_id: str) -> str:
        """!
        @brief   Renders a minimal figure fragment for race-condition
                  assertions; the figure's own content is irrelevant here.

        @param   element_id
                  The figure's container `<div>` id.

        @return  The figure's HTML fragment.
        """
        figure = go.Figure(data=[go.Scatter(x=[0, 1], y=[0, 1])])
        return html.figure_to_fragment(figure, element_id)

    def test_theme_call_is_chained_after_newplot_not_alongside_it(self) -> None:
        """!
        @brief  The figure's own theme call must be inside `Plotly.
                newPlot(...).then(...)`, not a separate, independently-
                timed statement.
        """
        fragment = self._fragment_for("race-fig")
        then_index = fragment.find(").then(function(){")
        theme_call_index = fragment.find("__reportApplyPlotlyTheme")
        self.assertNotEqual(then_index, -1, "post_script was not chained via .then()")
        self.assertGreater(
            theme_call_index,
            then_index,
            "the theme call must be inside the .then() callback, not before it",
        )

    def test_theme_call_targets_this_figures_own_div(self) -> None:
        """!
        @brief  The chained theme call looks up this exact figure's own
                container id, not a page-wide sweep re-introduced into the
                per-figure fragment.
        """
        fragment = self._fragment_for("race-fig-42")
        self.assertIn('document.getElementById("race-fig-42")', fragment)
        self.assertNotIn("querySelectorAll", fragment)

    def test_theme_call_is_guarded_against_a_missing_helper(self) -> None:
        """!
        @brief  If the shared theme script's helper somehow is not defined
                (e.g. a page fragment used outside `render_page()`), the
                figure's own post_script must no-op rather than throw.
        """
        fragment = self._fragment_for("race-fig")
        self.assertIn("if (window.__reportApplyPlotlyTheme)", fragment)

    def test_theme_helper_is_defined_before_the_first_figure_on_the_page(self) -> None:
        """!
        @brief  `render_page()` must place the shared theme script (which
                defines `window.__reportApplyPlotlyTheme`) before any
                figure fragment, so that a figure's own chained post_script
                can always find the helper already defined -- regardless
                of how quickly that figure's own `Plotly.newPlot()`
                resolves.
        """
        body = self._fragment_for("fig-a") + self._fragment_for("fig-b")
        page_html = html.render_page("Title", "index.html", body, [])
        helper_index = page_html.find("window.__reportApplyPlotlyTheme = function")
        first_figure_index = page_html.find("plotly-graph-div")
        self.assertNotEqual(helper_index, -1)
        self.assertNotEqual(first_figure_index, -1)
        self.assertLess(helper_index, first_figure_index)

    def test_element_ids_needing_js_escaping_do_not_break_the_script(self) -> None:
        """!
        @brief  Several call sites build element ids from run-derived
                labels (e.g. `f"wheel-drive-{label}"`, `f"visual-images-
                {topic}"`). The id is embedded via `json.dumps`, so even an
                id containing a quote or backslash must not break out of
                the generated `<script>` block's string literal.
        """
        fragment = self._fragment_for('odd"id\\name')
        self.assertIn(json.dumps('odd"id\\name'), fragment)


class WriteReportTests(unittest.TestCase):
    """!
    @brief  Tests for `write_report`'s atomic write and manifest contents.
    """

    def setUp(self) -> None:
        """!
        @brief  Creates a fresh temporary output directory.
        """
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """!
        @brief  Removes the temporary output directory.
        """
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def test_writes_every_page_and_shared_assets(self) -> None:
        """!
        @brief  Every page and both shared assets land in the output
                directory, with no leftover staging directory.
        """
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page("Kalman Filter", "kalman_filter.html", "<p>x</p>", []),
            summary_stats={},
            warnings=(),
        )
        html.write_report([page], self.output_dir, context)
        self.assertTrue((self.output_dir / "kalman_filter.html").is_file())
        self.assertTrue((self.output_dir / "assets" / "plotly.min.js").is_file())
        self.assertTrue((self.output_dir / "assets" / "report.css").is_file())
        self.assertFalse((self.output_dir / ".post_processing_staging").exists())

    def test_preserves_unrelated_analyst_file(self) -> None:
        """!
        @brief  Regenerating a report does not delete an unrelated file an
                analyst placed in the output directory (only this tool's
                own fixed filenames are replaced).
        """
        self.output_dir.mkdir(parents=True)
        analyst_file = self.output_dir / "my_notes.txt"
        analyst_file.write_text("keep me")
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page("Kalman Filter", "kalman_filter.html", "<p>x</p>", []),
            summary_stats={},
            warnings=(),
        )
        html.write_report([page], self.output_dir, context)
        self.assertTrue(analyst_file.is_file())
        self.assertEqual(analyst_file.read_text(), "keep me")

    def test_manifest_contents(self) -> None:
        """!
        @brief  report_manifest.json carries the generator version,
                generated page list, topic counts, and alignment tolerance.
        """
        context = _make_context(self.output_dir)
        page = ReportPage(
            filename="kalman_filter.html",
            title="Kalman Filter",
            html=html.render_page("Kalman Filter", "kalman_filter.html", "<p>x</p>", []),
            summary_stats={},
            warnings=("some warning",),
        )
        html.write_report([page], self.output_dir, context)
        manifest = json.loads((self.output_dir / "report_manifest.json").read_text())
        self.assertEqual(manifest["generator_version"], html.GENERATOR_VERSION)
        self.assertEqual(manifest["generated_pages"], ["kalman_filter.html"])
        self.assertEqual(manifest["topic_counts"]["/clock"], 10)
        self.assertAlmostEqual(manifest["maximum_alignment_gap_s"], 1.0)
        self.assertIn("kalman_filter.html: some warning", manifest["omissions"])


class ManifestMergeAcrossRegenerationTests(unittest.TestCase):
    """!
    @brief  Integration coverage for Defect 4 in the plan's 2026-09-22
            audit: an "aggregate generation, then standalone regeneration
            of one page" workflow must leave every page, navigation, and
            report_manifest.json's own generated_pages/omissions mutually
            consistent -- a standalone script's manifest write must never
            claim fewer pages than are still actually on disk.
    """

    def setUp(self) -> None:
        """!
        @brief  Creates a fresh temporary output directory.
        """
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """!
        @brief  Removes the temporary output directory.
        """
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def _page(self, filename: str, warnings: tuple = ()) -> ReportPage:
        return ReportPage(
            filename=filename,
            title=filename,
            html=html.render_page(filename, filename, "<p>x</p>", list(warnings)),
            summary_stats={},
            warnings=warnings,
        )

    def test_standalone_regeneration_preserves_other_pages_in_the_manifest(self) -> None:
        """!
        @brief  After an aggregate-style write of five pages, a
                standalone-style rewrite of just one of them still lists
                all five files in generated_pages (all five remain on
                disk), with the regenerated page's own entry current and
                every other page's omissions carried forward unchanged.
        """
        context = _make_context(self.output_dir)
        aggregate_pages = [
            self._page("index.html"),
            self._page("kalman_filter.html", warnings=("kalman warning",)),
            self._page("visual_odometry.html"),
            self._page("inertial_odometry.html"),
            self._page("wheel_odometry.html", warnings=("wheel warning",)),
        ]
        html.write_report(aggregate_pages, self.output_dir, context)

        # Standalone-style regeneration of only wheel_odometry.html, as
        # post_processing_wheel_odometry.py's own main() does.
        standalone_page = self._page("wheel_odometry.html", warnings=("new wheel warning",))
        html.write_report([standalone_page], self.output_dir, context)

        for filename in (
            "index.html",
            "kalman_filter.html",
            "visual_odometry.html",
            "inertial_odometry.html",
            "wheel_odometry.html",
        ):
            self.assertTrue((self.output_dir / filename).is_file())

        manifest = json.loads((self.output_dir / "report_manifest.json").read_text())
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
        # kalman_filter.html's own omission survives untouched.
        self.assertIn("kalman_filter.html: kalman warning", manifest["omissions"])
        # wheel_odometry.html's omission reflects the just-regenerated
        # page, not the stale aggregate-run warning.
        self.assertIn("wheel_odometry.html: new wheel warning", manifest["omissions"])
        self.assertNotIn("wheel_odometry.html: wheel warning", manifest["omissions"])

    def test_a_manually_deleted_page_is_dropped_from_the_manifest(self) -> None:
        """!
        @brief  If a previously-generated page's file no longer exists
                (e.g. an analyst deleted it), the next regeneration does
                not keep claiming it in generated_pages.
        """
        context = _make_context(self.output_dir)
        html.write_report(
            [self._page("kalman_filter.html"), self._page("wheel_odometry.html")],
            self.output_dir,
            context,
        )
        (self.output_dir / "wheel_odometry.html").unlink()

        html.write_report([self._page("kalman_filter.html")], self.output_dir, context)

        manifest = json.loads((self.output_dir / "report_manifest.json").read_text())
        self.assertEqual(manifest["generated_pages"], ["kalman_filter.html"])


class CrossRunOutputDirectoryTests(unittest.TestCase):
    """!
    @brief  Integration coverage for the 2026-09-23 correction: Defect 4's
            original manifest-merge fix (2026-09-22) correctly preserved a
            same-run standalone regeneration's sibling pages, but never
            checked that two writes into the same output_dir actually
            described the *same* run. Reusing --output-dir across two
            different test runs used to silently produce a manifest whose
            source_bag/topic_counts named one run while generated_pages
            still listed another run's stale, un-regenerated HTML files.
            `_ensure_same_run_or_raise` now fails such a write before
            anything is touched.
    """

    def setUp(self) -> None:
        """!
        @brief  Creates a fresh temporary output directory.
        """
        self._temp_dir = tempfile.mkdtemp()
        self.output_dir = Path(self._temp_dir) / "post_processing"

    def tearDown(self) -> None:
        """!
        @brief  Removes the temporary output directory.
        """
        shutil.rmtree(self._temp_dir, ignore_errors=True)

    def _page(self, filename: str, warnings: tuple = ()) -> ReportPage:
        return ReportPage(
            filename=filename,
            title=filename,
            html=html.render_page(filename, filename, "<p>x</p>", list(warnings)),
            summary_stats={},
            warnings=warnings,
        )

    def test_standalone_into_an_empty_directory_succeeds(self) -> None:
        """!
        @brief  A standalone single-page write into a directory that has
                never held a report before (no manifest to compare
                against) succeeds normally -- the identity check only ever
                protects an *existing* report.
        """
        context = _make_context(self.output_dir, run_name="run_only")
        html.write_report([self._page("wheel_odometry.html")], self.output_dir, context)
        self.assertTrue((self.output_dir / "wheel_odometry.html").is_file())
        manifest = json.loads((self.output_dir / "report_manifest.json").read_text())
        self.assertEqual(manifest["generated_pages"], ["wheel_odometry.html"])
        self.assertEqual(manifest["run_identity"]["test_run_dir"], "/tmp/run_only")

    def test_run_b_targeting_run_as_report_directory_fails(self) -> None:
        """!
        @brief  Run A's aggregate write, followed by run B's standalone
                write into the *same* output_dir, is refused before
                anything is written -- the two runs' different
                test_run_dir/source_bag identities are detected.
        """
        context_a = _make_context(self.output_dir, run_name="run_A", topic_count=500)
        html.write_report(
            [
                self._page("index.html"),
                self._page("kalman_filter.html"),
                self._page("visual_odometry.html"),
                self._page("inertial_odometry.html"),
                self._page("wheel_odometry.html", warnings=("A-specific warning",)),
            ],
            self.output_dir,
            context_a,
        )

        context_b = _make_context(self.output_dir, run_name="run_B", topic_count=999)
        with self.assertRaises(html.CrossRunOutputDirectoryError) as raised:
            html.write_report(
                [self._page("wheel_odometry.html", warnings=("B-specific warning",))],
                self.output_dir,
                context_b,
            )
        # The error is actionable: it names the conflicting directory and
        # both runs' identities, not just a generic failure.
        message = str(raised.exception)
        self.assertIn(str(self.output_dir), message)
        self.assertIn("run_A", message)
        self.assertIn("run_B", message)

    def test_failed_cross_run_attempt_leaves_every_file_unchanged(self) -> None:
        """!
        @brief  After a rejected cross-run write, every file run A wrote
                -- including report_manifest.json itself -- is byte-for-
                byte identical to before the attempt, and no partial or
                staging artefact is left behind.
        """
        context_a = _make_context(self.output_dir, run_name="run_A")
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

        context_b = _make_context(self.output_dir, run_name="run_B")
        with self.assertRaises(html.CrossRunOutputDirectoryError):
            html.write_report([self._page("kalman_filter.html")], self.output_dir, context_b)

        after = {
            path.relative_to(self.output_dir): path.read_bytes()
            for path in sorted(self.output_dir.rglob("*"))
            if path.is_file()
        }
        self.assertEqual(before, after)
        self.assertFalse((self.output_dir / ".post_processing_staging").exists())

    def test_manifest_missing_identity_with_generated_pages_is_not_merged(self) -> None:
        """!
        @brief  An old or externally-written manifest that lists generated
                pages but carries no run_identity field at all cannot be
                merged into optimistically, even though its listed pages
                are genuinely present on disk -- there is nothing to prove
                it belongs to the same run as the new write.
        """
        self.output_dir.mkdir(parents=True)
        (self.output_dir / "kalman_filter.html").write_text("<html>old</html>")
        (self.output_dir / "report_manifest.json").write_text(
            json.dumps({"generated_pages": ["kalman_filter.html"], "omissions": []})
        )

        context = _make_context(self.output_dir, run_name="run_new")
        with self.assertRaises(html.CrossRunOutputDirectoryError) as raised:
            html.write_report([self._page("kalman_filter.html")], self.output_dir, context)
        self.assertIn("does not record which run", str(raised.exception))
        # The pre-existing files are untouched by the rejected attempt.
        self.assertEqual(
            (self.output_dir / "kalman_filter.html").read_text(), "<html>old</html>"
        )


if __name__ == "__main__":
    unittest.main()
