"""!
@brief  Regression tests for the 2026-09-23 Plotly rendering defects that
        survived the earlier initial-render-race fix: WebGL-context
        exhaustion (every 2-D trace was a `Scattergl`), subplot axis
        titles/range sliders landing on the wrong row, default-template
        table cells unreadable in dark mode, and a theme script that
        missed 3-D scenes and could not report a failed relayout. The
        JavaScript behaviour tests execute the real generated theme script
        under Node.js against a minimal Plotly/DOM stand-in, and are
        skipped when `node` is not installed.
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np
import plotly.graph_objects as go

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.data.models import ImageFrame, PointCloudSnapshot
from python_tools.reporting import figures, html
from python_tools.reporting.style import CHROME_DARK, CHROME_LIGHT, TABLE_RULE_COLOR


def _time_series(sample_count: int = 50) -> tuple[np.ndarray, np.ndarray]:
    """!
    @brief   Builds a small deterministic (N,) time vector and (N, 3) series.

    @param   sample_count
             Number of samples, N.

    @return  `(times_s, values)`.
    """
    times_s = np.linspace(0.0, 10.0, sample_count)
    values = np.stack([np.sin(times_s), np.cos(times_s), 0.1 * times_s], axis=1)
    return times_s, values


def _every_2d_figure() -> dict[str, go.Figure]:
    """!
    @brief   Builds one instance of every 2-D line-plot constructor.

    @return  Constructor name -> figure.
    """
    times_s, values = _time_series()
    scalar = values[:, 0]
    return {
        "three_axis_time_series": figures.three_axis_time_series(
            times_s, [("a", values, "#000000"), ("b", values * 2, "#111111")], ("X", "Y", "Z"), "m"
        ),
        "trajectory_xy": figures.trajectory_xy([("a", values, "#000000")]),
        "estimate_truth_error_panel": figures.estimate_truth_error_panel(
            times_s, scalar, scalar, scalar * 0.1, "Value (m)", "Error (m)",
            sigma_band=np.full_like(scalar, 0.01),
        ),
        "six_wheel_small_multiples": figures.six_wheel_small_multiples(
            times_s, np.tile(scalar[:, None], (1, 6)), "Speed (rad/s)"
        ),
        "rate_count_plot": figures.rate_count_plot(times_s, scalar, "Rate (Hz)"),
    }


class NoWebglTraceTests(unittest.TestCase):
    """!
    @brief  No 2-D constructor may emit a WebGL trace: a report page holds
            more WebGL figures than a browser keeps contexts for, and the
            evicted figures render blank.
    """

    def test_no_constructor_emits_scattergl(self) -> None:
        """!
        @brief  Every 2-D line trace is an SVG `scatter`.
        """
        for name, figure in _every_2d_figure().items():
            with self.subTest(constructor=name):
                trace_types = {trace.type for trace in figure.data}
                self.assertNotIn("scattergl", trace_types)
                self.assertEqual(trace_types, {"scatter"})

    def test_generated_fragment_contains_no_scattergl(self) -> None:
        """!
        @brief  The trace array serialized into the HTML (what the browser
                actually receives) carries no `scattergl` trace either.
                Only the `newPlot` data argument is inspected: the
                embedded `plotly_white` template lists default styles for
                every trace type, `scattergl` included, which creates no
                trace and no WebGL context.
        """
        decoder = json.JSONDecoder()
        for name, figure in _every_2d_figure().items():
            with self.subTest(constructor=name):
                fragment = html.figure_to_fragment(figure, f"fig-{name}")
                # The data array is the JSON value right after the div id.
                call = re.search(r'Plotly\.newPlot\(\s*"[^"]+",\s*', fragment)
                data, _ = decoder.raw_decode(fragment, call.end())
                self.assertEqual({trace.get("type") for trace in data}, {"scatter"})


class SubplotAxisLayoutTests(unittest.TestCase):
    """!
    @brief  Shared axis styling reaches every subplot axis, titles land on
            the right row, and exactly one range slider sits on the bottom
            row (previously `update_layout(xaxis=..., yaxis=...)` hit only
            the top-left subplot).
    """

    def _axis_title(self, axis) -> str:
        """!
        @brief   Reads an axis title's text, `""` when unset.

        @param   axis
                 A Plotly layout axis object.

        @return  The title text.
        """
        return axis.title.text or ""

    def _visible_rangeslider_axes(self, figure: go.Figure) -> list[str]:
        """!
        @brief   Lists the layout X axes carrying a visible range slider.

        @param   figure
                 The figure to inspect.

        @return  Axis names, e.g. `["xaxis3"]`.
        """
        layout = figure.to_plotly_json()["layout"]
        return sorted(
            key
            for key, value in layout.items()
            if re.fullmatch(r"xaxis\d*", key)
            and isinstance(value, dict)
            and "rangeslider" in value
            and value["rangeslider"].get("visible", True)
        )

    def test_one_axis_figure_keeps_both_titles_and_no_slider(self) -> None:
        """!
        @brief  A single-axis figure keeps its X/Y titles; the shared
                layout's empty `rangeslider: {}` is not serialized.
        """
        figure = _every_2d_figure()["rate_count_plot"]
        self.assertEqual(self._axis_title(figure.layout.xaxis), "Time (s)")
        self.assertEqual(self._axis_title(figure.layout.yaxis), "Rate (Hz)")
        self.assertEqual(self._visible_rangeslider_axes(figure), [])

    def test_three_axis_rows_keep_their_titles(self) -> None:
        """!
        @brief  Each row's Y title survives; the X title is on the bottom
                row only; the range slider is only on the bottom row; axis
                styling reaches `xaxis2`/`yaxis3`, not just the first axes.
        """
        figure = _every_2d_figure()["three_axis_time_series"]
        layout = figure.layout
        self.assertEqual(self._axis_title(layout.yaxis), "X<br>(m)")
        self.assertEqual(self._axis_title(layout.yaxis2), "Y<br>(m)")
        self.assertEqual(self._axis_title(layout.yaxis3), "Z<br>(m)")
        self.assertEqual(self._axis_title(layout.xaxis), "")
        self.assertEqual(self._axis_title(layout.xaxis2), "")
        self.assertEqual(self._axis_title(layout.xaxis3), "Time (s)")
        self.assertEqual(self._visible_rangeslider_axes(figure), ["xaxis3"])
        for axis in (layout.xaxis, layout.xaxis2, layout.xaxis3, layout.yaxis, layout.yaxis3):
            self.assertEqual(axis.gridcolor, CHROME_LIGHT["gridline"])
            self.assertEqual(axis.linecolor, CHROME_LIGHT["axis"])

    def test_error_covariance_panel_layout(self) -> None:
        """!
        @brief  The estimate/error panel keeps its top-row value title,
                puts the X title on the error row (it previously sat on
                the top row, between the two panels), and has exactly one
                range slider, on the error row.
        """
        figure = _every_2d_figure()["estimate_truth_error_panel"]
        layout = figure.layout
        self.assertEqual(self._axis_title(layout.yaxis), "Value (m)")
        self.assertEqual(self._axis_title(layout.yaxis2), "Error (m)")
        self.assertEqual(self._axis_title(layout.xaxis), "")
        self.assertEqual(self._axis_title(layout.xaxis2), "Time (s)")
        self.assertEqual(self._visible_rangeslider_axes(figure), ["xaxis2"])

    def test_six_wheel_grid_titles_bottom_row_only(self) -> None:
        """!
        @brief  In the 2x3 grid the X title is on the three bottom-row
                axes (`xaxis4`-`xaxis6`) and on no top-row axis.
        """
        layout = _every_2d_figure()["six_wheel_small_multiples"].layout
        for name in ("xaxis", "xaxis2", "xaxis3"):
            self.assertEqual(self._axis_title(layout[name]), "", name)
        for name in ("xaxis4", "xaxis5", "xaxis6"):
            self.assertEqual(self._axis_title(layout[name]), "Time (s)", name)


class SummaryTableStyleTests(unittest.TestCase):
    """!
    @brief  Summary tables stay readable in both themes.
    """

    def test_cells_are_transparent_with_neutral_rules(self) -> None:
        """!
        @brief  Cells carry no fixed pale fill (which stayed light in dark
                mode under white themed text) and use the shared neutral
                rule color; the header keeps white-on-blue.
        """
        table = figures.summary_table(["Metric", "Value"], [["RMSE", "0.1"]]).data[0]
        self.assertEqual(table.cells.fill.color, "rgba(0,0,0,0)")
        self.assertEqual(table.cells.line.color, TABLE_RULE_COLOR)
        self.assertEqual(table.header.font.color, "white")

    def test_table_starts_with_the_shared_light_text_color(self) -> None:
        """!
        @brief  The table's initial text color matches every other figure,
                so the theme script's `font.color` relayout governs it.
        """
        figure = figures.summary_table(["A"], [["1"]])
        self.assertEqual(figure.layout.font.color, CHROME_LIGHT["text_primary"])


class SixWheelGridTitleTests(unittest.TestCase):
    """!
    @brief  The 2x3 wheel grid's long Y title fits and clears the modebar.
            Repeated per ~150 px row, a title such as "Public wheel command
            velocity (rad/s)" overflowed the 440 px figure at both edges
            (measured in headless Chromium), and the modebar covered the
            top-right subplot title.
    """

    def setUp(self) -> None:
        times_s, _ = _time_series()
        self.figure = figures.six_wheel_small_multiples(
            times_s, np.zeros((times_s.size, 6)), "Public wheel command velocity (rad/s)"
        )

    def test_y_title_is_one_centred_annotation(self) -> None:
        """!
        @brief  No subplot Y axis repeats the title; one rotated
                paper-referenced annotation spans both rows instead.
        """
        layout = self.figure.layout
        for name in ("yaxis", "yaxis2", "yaxis3", "yaxis4", "yaxis5", "yaxis6"):
            self.assertIn(layout[name].title.text, (None, ""), name)
        shared = [a for a in layout.annotations if a.text == "Public wheel command velocity (rad/s)"]
        self.assertEqual(len(shared), 1)
        self.assertEqual((shared[0].xref, shared[0].yref, shared[0].y), ("paper", "paper", 0.5))
        self.assertEqual(shared[0].textangle, -90)

    def test_top_margin_clears_the_modebar(self) -> None:
        """!
        @brief  Subplot titles sit below the modebar's band at the top.
        """
        self.assertGreaterEqual(self.figure.layout.margin.t, figures.SMALL_MULTIPLES_TOP_MARGIN_PX)
        self.assertGreaterEqual(figures.SMALL_MULTIPLES_TOP_MARGIN_PX, 60)


class TrajectoryLegendOnlyTests(unittest.TestCase):
    """!
    @brief  A diagnostic-only trajectory can start hidden-but-available.
            The Kalman overlay's unaided inertial trace reaches kilometres
            while every fused source stays within metres; drawn by default
            it forces autorange to ~+/-2 km and hides the useful traces.
    """

    def test_named_series_is_legend_only_and_others_visible(self) -> None:
        """!
        @brief  Only the named label starts as `legendonly` (excluded from
                autorange, restorable from the legend).
        """
        positions = np.zeros((3, 3))
        figure = figures.trajectory_xy(
            [("truth", positions, "#000000"), ("inertial", positions * 1e3, "#111111")],
            legend_only_labels=("inertial",),
        )
        self.assertEqual([trace.visible for trace in figure.data], [None, "legendonly"])

    def test_default_draws_every_series(self) -> None:
        """!
        @brief  Existing callers that name nothing keep every trace drawn.
        """
        figure = figures.trajectory_xy([("a", np.zeros((2, 3)), "#000000")])
        self.assertIsNone(figure.data[0].visible)


class PointCloudInitialFrameTests(unittest.TestCase):
    """!
    @brief  The 3-D point-cloud slider opens on a frame that has points.
            A real capture's first sampled cloud is published before visual
            odometry initializes and holds zero points, which previously
            left the panel blank on every load until the slider was moved.
    """

    @staticmethod
    def _snapshot(time_s: float, point_count: int) -> PointCloudSnapshot:
        return PointCloudSnapshot(
            time_s=time_s,
            points_m=np.ones((point_count, 3)),
            is_inlier_cloud=True,
            frame_id="camera",
        )

    def test_opens_on_first_non_empty_snapshot(self) -> None:
        """!
        @brief  An empty leading snapshot is skipped for the initial view,
                and the slider's active step names the same frame.
        """
        figure = figures.point_cloud_3d_slider(
            [self._snapshot(0.0, 0), self._snapshot(1.0, 5), self._snapshot(2.0, 7)]
        )
        self.assertEqual([trace.visible for trace in figure.data], [False, True, False])
        self.assertEqual(figure.layout.sliders[0].active, 1)

    def test_all_empty_snapshots_fall_back_to_the_first(self) -> None:
        """!
        @brief  With no non-empty frame there is nothing better to show, so
                the first frame stays the initial view.
        """
        figure = figures.point_cloud_3d_slider([self._snapshot(0.0, 0), self._snapshot(1.0, 0)])
        self.assertEqual([trace.visible for trace in figure.data], [True, False])
        self.assertEqual(figure.layout.sliders[0].active, 0)


class PostInitializationHookTests(unittest.TestCase):
    """!
    @brief  Every figure on a page -- whatever its family -- receives the
            post-`newPlot()` theme hook exactly once, for its own div.
    """

    def test_every_figure_family_gets_the_hook_exactly_once(self) -> None:
        """!
        @brief  Renders one of every figure family into one page and checks
                one `newPlot` per figure and one own-id theme call each.
        """
        family_figures = dict(_every_2d_figure())
        family_figures["summary_table"] = figures.summary_table(["A"], [["1"]])
        family_figures["trajectory_3d"] = figures.trajectory_3d(
            [("a", _time_series()[1], "#000000")]
        )
        family_figures["point_cloud_3d_slider"] = figures.point_cloud_3d_slider(
            [
                PointCloudSnapshot(
                    time_s=float(i),
                    points_m=np.zeros((4, 3)),
                    is_inlier_cloud=True,
                    frame_id="camera",
                )
                for i in range(2)
            ]
        )
        family_figures["image_slider_figure"] = figures.image_slider_figure(
            [
                ImageFrame(
                    time_s=float(i),
                    rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                    source_encoding="rgb8",
                )
                for i in range(2)
            ]
        )
        body = "".join(
            html.figure_to_fragment(figure, f"fig-{name}") for name, figure in family_figures.items()
        )
        page_html = html.render_page("Title", "index.html", body, [])
        self.assertEqual(page_html.count("Plotly.newPlot("), len(family_figures))
        self.assertEqual(
            page_html.count("window.__reportApplyPlotlyTheme(document.getElementById("),
            len(family_figures),
        )
        for name in family_figures:
            with self.subTest(figure=name):
                call = f'window.__reportApplyPlotlyTheme(document.getElementById("fig-{name}"))'
                self.assertEqual(page_html.count(call), 1)


# Node.js driver: evaluates the real theme script against a stand-in DOM,
# `matchMedia`, and `Plotly.relayout`, then runs a scenario and prints a
# JSON result. `SCRIPT`, `FIGURES` and `SCENARIO` are substituted in.
_NODE_DRIVER = r"""
const scriptSource = SCRIPT;
const figureSpecs = FIGURES;
const scenario = SCENARIO;
const listeners = [];
const media = { matches: scenario.initialDark,
  addEventListener: (type, fn) => { if (type === "change") listeners.push(fn); } };
const errors = [];
const unhandled = [];
process.on("unhandledRejection", (reason) => unhandled.push(String(reason)));
const divs = figureSpecs.map((spec) => ({ id: spec.id, layout: spec.layout, attrs: {},
  setAttribute(name, value) { this.attrs[name] = value; } }));
const calls = [];
global.window = { matchMedia: () => media, console: { error: (m) => errors.push(m) } };
global.console.error = (m) => errors.push(m);
global.document = { querySelectorAll: () => divs };
global.Plotly = { relayout: (div, update) => {
  calls.push({ id: div.id, update });
  const mode = (scenario.failures || {})[div.id];
  if (mode === "reject") return Promise.reject(new Error("relayout rejected"));
  if (mode === "throw") throw new Error("relayout threw");
  return Promise.resolve(div);
} };
eval(scriptSource);
(async () => {
  for (const div of divs) { await window.__reportApplyPlotlyTheme(div); }
  if (scenario.toggleTo !== undefined) {
    media.matches = scenario.toggleTo;
    listeners.forEach((fn) => fn({ matches: media.matches }));
  }
  await new Promise((resolve) => setTimeout(resolve, 20));
  console.log(JSON.stringify({ calls, errors, unhandled,
    attrs: Object.fromEntries(divs.map((d) => [d.id, d.attrs])) }));
})();
"""

# Every figure family's layout keys as the browser sees them in
# `graphDiv.layout`: one-axis, three-row subplot with a bottom-row slider,
# and a 3-D scene.
_FIGURE_SPECS = [
    {"id": "one-axis", "layout": {"xaxis": {}, "yaxis": {}}},
    {
        "id": "subplots",
        "layout": {
            "xaxis": {}, "xaxis2": {}, "xaxis3": {"rangeslider": {"visible": True}},
            "yaxis": {}, "yaxis2": {}, "yaxis3": {},
        },
    },
    {"id": "scene", "layout": {"xaxis": {}, "yaxis": {}, "scene": {"aspectmode": "data"}}},
]

# Every key the theme script may write ends in one of these color
# attributes; anything else (range, autorange, visible, images, sliders)
# would risk disturbing the viewer's zoom or state on a live theme change.
_COLOR_ONLY_KEY = re.compile(r"(^|\.)(color|gridcolor|linecolor|zerolinecolor|bgcolor|bordercolor|backgroundcolor)$")


@unittest.skipUnless(shutil.which("node"), "node is not installed; JS behaviour tests skipped")
class ThemeScriptBehaviourTests(unittest.TestCase):
    """!
    @brief  Executes the real generated theme script under Node.js.
    """

    def _run(self, scenario: dict) -> dict:
        """!
        @brief   Runs the Node driver for one scenario.

        @param   scenario
                 `{"initialDark": bool, "toggleTo"?: bool, "failures"?:
                 {div_id: "reject"|"throw"}}`.

        @return  The driver's parsed JSON result.
        """
        script_block = html._plotly_theme_script()
        script_source = re.search(r"<script>(.*)</script>", script_block, re.DOTALL).group(1)
        driver = (
            _NODE_DRIVER.replace("SCRIPT", json.dumps(script_source))
            .replace("FIGURES", json.dumps(_FIGURE_SPECS))
            .replace("SCENARIO", json.dumps(scenario))
        )
        completed = subprocess.run(
            ["node", "-e", driver], capture_output=True, text=True, timeout=30, check=True
        )
        return json.loads(completed.stdout.strip().splitlines()[-1])

    def _updates_by_id(self, result: dict) -> dict[str, list[dict]]:
        """!
        @brief   Groups recorded relayout updates by figure id.

        @param   result
                 A driver result.

        @return  Figure id -> list of update dicts, in call order.
        """
        grouped: dict[str, list[dict]] = {}
        for call in result["calls"]:
            grouped.setdefault(call["id"], []).append(call["update"])
        return grouped

    def test_initial_dark_theme_covers_every_axis_family(self) -> None:
        """!
        @brief  Dark mode themes the font, every cartesian axis including
                `xaxis2`/`yaxis3`, the visible range slider, the hover
                label, and every 3-D scene axis background.
        """
        updates = self._updates_by_id(self._run({"initialDark": True}))
        self.assertEqual(sorted(updates), ["one-axis", "scene", "subplots"])
        subplots = updates["subplots"][0]
        self.assertEqual(subplots["font.color"], CHROME_DARK["text_primary"])
        for axis in ("xaxis", "xaxis2", "xaxis3", "yaxis", "yaxis2", "yaxis3"):
            self.assertEqual(subplots[f"{axis}.gridcolor"], CHROME_DARK["gridline"])
        self.assertIn("xaxis3.rangeslider.bordercolor", subplots)
        self.assertNotIn("xaxis.rangeslider.bordercolor", subplots)
        self.assertEqual(subplots["hoverlabel.bgcolor"], CHROME_DARK["surface"])
        scene = updates["scene"][0]
        for axis in ("xaxis", "yaxis", "zaxis"):
            self.assertEqual(scene[f"scene.{axis}.backgroundcolor"], CHROME_DARK["surface"])

    def test_initial_light_theme(self) -> None:
        """!
        @brief  Light mode applies the light palette.
        """
        updates = self._updates_by_id(self._run({"initialDark": False}))
        one_axis = updates["one-axis"][0]
        self.assertEqual(one_axis["font.color"], CHROME_LIGHT["text_primary"])
        self.assertEqual(one_axis["xaxis.gridcolor"], CHROME_LIGHT["gridline"])

    def test_live_theme_change_rethemes_every_figure_with_color_keys_only(self) -> None:
        """!
        @brief  A live dark->light change re-themes every figure once more,
                and every written key is a color -- ranges, zoom, sliders,
                images and trace visibility are never touched.
        """
        result = self._run({"initialDark": True, "toggleTo": False})
        updates = self._updates_by_id(result)
        for figure_id, figure_updates in updates.items():
            with self.subTest(figure=figure_id):
                self.assertEqual(len(figure_updates), 2)
                self.assertEqual(figure_updates[0]["font.color"], CHROME_DARK["text_primary"])
                self.assertEqual(figure_updates[1]["font.color"], CHROME_LIGHT["text_primary"])
                for update in figure_updates:
                    for key in update:
                        self.assertRegex(key, _COLOR_ONLY_KEY)

    def test_rejected_relayout_is_contained_and_reported(self) -> None:
        """!
        @brief  One figure's rejected `Plotly.relayout()` promise does not
                stop the others, leaves no unhandled rejection, and is
                reported on its div and the console.
        """
        result = self._run({"initialDark": True, "failures": {"one-axis": "reject"}})
        updates = self._updates_by_id(result)
        self.assertEqual(sorted(updates), ["one-axis", "scene", "subplots"])
        self.assertEqual(result["unhandled"], [])
        self.assertIn("relayout rejected", result["attrs"]["one-axis"]["data-report-theme-error"])
        self.assertNotIn("data-report-theme-error", result["attrs"]["subplots"])
        self.assertTrue(any("one-axis" in message for message in result["errors"]))

    def test_synchronous_relayout_throw_is_contained(self) -> None:
        """!
        @brief  A synchronous throw from `Plotly.relayout()` is caught the
                same way and later figures are still themed.
        """
        result = self._run({"initialDark": True, "failures": {"subplots": "throw"}})
        updates = self._updates_by_id(result)
        self.assertIn("scene", updates)
        self.assertIn("relayout threw", result["attrs"]["subplots"]["data-report-theme-error"])
        self.assertEqual(result["unhandled"], [])


if __name__ == "__main__":
    unittest.main()
