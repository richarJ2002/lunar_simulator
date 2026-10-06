"""Tests for Plotly rendering fixes.

Contents:
    NoWebglTraceTests: no WebGL traces.
    SubplotAxisLayoutTests: axis and slider layout.
    SummaryTableStyleTests: readable tables.
    SixWheelGridTitleTests: grid title and margin.
    TrajectoryLegendOnlyTests: hidden diagnostics.
    PointCloudInitialFrameTests: initial frame.
    PostInitializationHookTests: theme hook coverage.
    ThemeScriptBehaviourTests: live theme behavior.

JavaScript behavior tests run the real theme script
under Node.js and skip when node is missing.
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

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.data.models import ImageFrame, PointCloudSnapshot
from python_tools.reporting import figures, html
from python_tools.reporting.style import (
    CHROME_DARK,
    CHROME_LIGHT,
    TABLE_RULE_COLOR,
)


def _time_series(
    sample_count: int = 50,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a small deterministic time series."""
    times_s = np.linspace(0.0, 10.0, sample_count)
    values = np.stack(
        [np.sin(times_s), np.cos(times_s), 0.1 * times_s], axis=1
    )
    return times_s, values


def _every_2d_figure() -> dict[str, go.Figure]:
    """Build one of every 2-D line-plot constructor."""
    times_s, values = _time_series()
    scalar = values[:, 0]
    return {
        "three_axis_time_series": figures.three_axis_time_series(
            times_s,
            [("a", values, "#000000"),
             ("b", values * 2, "#111111")],
            ("X", "Y", "Z"),
            "m",
        ),
        "trajectory_xy": figures.trajectory_xy(
            [("a", values, "#000000")]
        ),
        "estimate_truth_error_panel": (
            figures.estimate_truth_error_panel(
                times_s,
                scalar,
                scalar,
                scalar * 0.1,
                "Value (m)",
                "Error (m)",
                sigma_band=np.full_like(scalar, 0.01),
            )
        ),
        "six_wheel_small_multiples": (
            figures.six_wheel_small_multiples(
                times_s,
                np.tile(scalar[:, None], (1, 6)),
                "Speed (rad/s)",
            )
        ),
        "rate_count_plot": figures.rate_count_plot(
            times_s, scalar, "Rate (Hz)"
        ),
    }


class NoWebglTraceTests(unittest.TestCase):
    """Tests that no 2-D trace uses WebGL."""

    def test_no_constructor_emits_scattergl(self) -> None:
        """Every 2-D trace is SVG scatter."""
        for name, figure in _every_2d_figure().items():
            with self.subTest(constructor=name):
                trace_types = {trace.type for trace in figure.data}
                self.assertNotIn("scattergl", trace_types)
                self.assertEqual(trace_types, {"scatter"})

    def test_generated_fragment_contains_no_scattergl(self) -> None:
        """Serialized HTML carries no scattergl trace."""
        # Only the newPlot data argument is checked; the
        # embedded template lists every trace type by default.
        decoder = json.JSONDecoder()
        for name, figure in _every_2d_figure().items():
            with self.subTest(constructor=name):
                fragment = html.figure_to_fragment(
                    figure, f"fig-{name}"
                )
                call = re.search(
                    r'Plotly\.newPlot\(\s*"[^"]+",\s*', fragment
                )
                data, _ = decoder.raw_decode(fragment, call.end())
                self.assertEqual(
                    {trace.get("type") for trace in data},
                    {"scatter"},
                )


class SubplotAxisLayoutTests(unittest.TestCase):
    """Tests for subplot axis and slider layout."""

    def _axis_title(self, axis) -> str:
        """Read an axis title, empty when unset."""
        return axis.title.text or ""

    def _visible_rangeslider_axes(
        self, figure: go.Figure
    ) -> list[str]:
        """List X axes with a visible range slider."""
        layout = figure.to_plotly_json()["layout"]
        return sorted(
            key
            for key, value in layout.items()
            if re.fullmatch(r"xaxis\d*", key)
            and isinstance(value, dict)
            and "rangeslider" in value
            and value["rangeslider"].get("visible", True)
        )

    def test_one_axis_figure_keeps_both_titles_and_no_slider(
        self,
    ) -> None:
        """Single-axis titles survive with no slider."""
        figure = _every_2d_figure()["rate_count_plot"]
        self.assertEqual(
            self._axis_title(figure.layout.xaxis), "Time (s)"
        )
        self.assertEqual(
            self._axis_title(figure.layout.yaxis), "Rate (Hz)"
        )
        self.assertEqual(self._visible_rangeslider_axes(figure), [])

    def test_three_axis_rows_keep_their_titles(self) -> None:
        """Each row keeps titles and shared styling."""
        figure = _every_2d_figure()["three_axis_time_series"]
        layout = figure.layout
        self.assertEqual(self._axis_title(layout.yaxis), "X<br>(m)")
        self.assertEqual(
            self._axis_title(layout.yaxis2), "Y<br>(m)"
        )
        self.assertEqual(
            self._axis_title(layout.yaxis3), "Z<br>(m)"
        )
        self.assertEqual(self._axis_title(layout.xaxis), "")
        self.assertEqual(self._axis_title(layout.xaxis2), "")
        self.assertEqual(
            self._axis_title(layout.xaxis3), "Time (s)"
        )
        self.assertEqual(
            self._visible_rangeslider_axes(figure), ["xaxis3"]
        )
        for axis in (
            layout.xaxis,
            layout.xaxis2,
            layout.xaxis3,
            layout.yaxis,
            layout.yaxis3,
        ):
            self.assertEqual(
                axis.gridcolor, CHROME_LIGHT["gridline"]
            )
            self.assertEqual(axis.linecolor, CHROME_LIGHT["axis"])

    def test_error_covariance_panel_layout(self) -> None:
        """Error panel keeps titles and one slider."""
        figure = _every_2d_figure()["estimate_truth_error_panel"]
        layout = figure.layout
        self.assertEqual(
            self._axis_title(layout.yaxis), "Value (m)"
        )
        self.assertEqual(
            self._axis_title(layout.yaxis2), "Error (m)"
        )
        self.assertEqual(self._axis_title(layout.xaxis), "")
        self.assertEqual(
            self._axis_title(layout.xaxis2), "Time (s)"
        )
        self.assertEqual(
            self._visible_rangeslider_axes(figure), ["xaxis2"]
        )

    def test_six_wheel_grid_titles_bottom_row_only(self) -> None:
        """Grid X title sits on the bottom row only."""
        layout = _every_2d_figure()[
            "six_wheel_small_multiples"
        ].layout
        for name in ("xaxis", "xaxis2", "xaxis3"):
            self.assertEqual(
                self._axis_title(layout[name]), "", name
            )
        for name in ("xaxis4", "xaxis5", "xaxis6"):
            self.assertEqual(
                self._axis_title(layout[name]), "Time (s)", name
            )


class SummaryTableStyleTests(unittest.TestCase):
    """Tests for readable summary tables."""

    def test_cells_are_transparent_with_neutral_rules(self) -> None:
        """Cells use transparent fill and neutral rules."""
        table = figures.summary_table(
            ["Metric", "Value"], [["RMSE", "0.1"]]
        ).data[0]
        self.assertEqual(table.cells.fill.color, "rgba(0,0,0,0)")
        self.assertEqual(
            table.cells.line.color, TABLE_RULE_COLOR
        )
        self.assertEqual(table.header.font.color, "white")

    def test_table_starts_with_the_shared_light_text_color(
        self,
    ) -> None:
        """Table starts with the shared light color."""
        figure = figures.summary_table(["A"], [["1"]])
        self.assertEqual(
            figure.layout.font.color,
            CHROME_LIGHT["text_primary"],
        )


class SixWheelGridTitleTests(unittest.TestCase):
    """Tests for the wheel-grid title and margin."""

    def setUp(self) -> None:
        times_s, _ = _time_series()
        self.figure = figures.six_wheel_small_multiples(
            times_s,
            np.zeros((times_s.size, 6)),
            "Public wheel command velocity (rad/s)",
        )

    def test_y_title_is_one_centred_annotation(self) -> None:
        """Title is one centred paper annotation."""
        layout = self.figure.layout
        for name in (
            "yaxis",
            "yaxis2",
            "yaxis3",
            "yaxis4",
            "yaxis5",
            "yaxis6",
        ):
            self.assertIn(
                layout[name].title.text, (None, ""), name
            )
        shared = [
            a
            for a in layout.annotations
            if a.text == "Public wheel command velocity (rad/s)"
        ]
        self.assertEqual(len(shared), 1)
        self.assertEqual(
            (shared[0].xref, shared[0].yref, shared[0].y),
            ("paper", "paper", 0.5),
        )
        self.assertEqual(shared[0].textangle, -90)

    def test_top_margin_clears_the_modebar(self) -> None:
        """Titles sit below the modebar band."""
        self.assertGreaterEqual(
            self.figure.layout.margin.t,
            figures.SMALL_MULTIPLES_TOP_MARGIN_PX,
        )
        self.assertGreaterEqual(
            figures.SMALL_MULTIPLES_TOP_MARGIN_PX, 60
        )


class TrajectoryLegendOnlyTests(unittest.TestCase):
    """Tests for hidden diagnostic trajectories."""

    def test_named_series_is_legend_only_and_others_visible(
        self,
    ) -> None:
        """Only the named label starts hidden."""
        positions = np.zeros((3, 3))
        figure = figures.trajectory_xy(
            [
                ("truth", positions, "#000000"),
                ("inertial", positions * 1e3, "#111111"),
            ],
            legend_only_labels=("inertial",),
        )
        self.assertEqual(
            [trace.visible for trace in figure.data],
            [None, "legendonly"],
        )

    def test_default_draws_every_series(self) -> None:
        """Default draws every trace."""
        figure = figures.trajectory_xy(
            [("a", np.zeros((2, 3)), "#000000")]
        )
        self.assertIsNone(figure.data[0].visible)


class PointCloudInitialFrameTests(unittest.TestCase):
    """Tests for the point-cloud initial frame."""

    @staticmethod
    def _snapshot(
        time_s: float, point_count: int
    ) -> PointCloudSnapshot:
        return PointCloudSnapshot(
            time_s=time_s,
            points_m=np.ones((point_count, 3)),
            is_inlier_cloud=True,
            frame_id="camera",
        )

    def test_opens_on_first_non_empty_snapshot(self) -> None:
        """Empty leading frame is skipped initially."""
        figure = figures.point_cloud_3d_slider(
            [
                self._snapshot(0.0, 0),
                self._snapshot(1.0, 5),
                self._snapshot(2.0, 7),
            ]
        )
        self.assertEqual(
            [trace.visible for trace in figure.data],
            [False, True, False],
        )
        self.assertEqual(figure.layout.sliders[0].active, 1)

    def test_all_empty_snapshots_fall_back_to_the_first(
        self,
    ) -> None:
        """All-empty input keeps the first frame."""
        figure = figures.point_cloud_3d_slider(
            [self._snapshot(0.0, 0), self._snapshot(1.0, 0)]
        )
        self.assertEqual(
            [trace.visible for trace in figure.data],
            [True, False],
        )
        self.assertEqual(figure.layout.sliders[0].active, 0)


class PostInitializationHookTests(unittest.TestCase):
    """Tests for the post-newPlot theme hook."""

    def test_every_figure_family_gets_the_hook_exactly_once(
        self,
    ) -> None:
        """Every family gets exactly one hook."""
        family_figures = dict(_every_2d_figure())
        family_figures["summary_table"] = figures.summary_table(
            ["A"], [["1"]]
        )
        family_figures["trajectory_3d"] = figures.trajectory_3d(
            [("a", _time_series()[1], "#000000")]
        )
        family_figures["point_cloud_3d_slider"] = (
            figures.point_cloud_3d_slider(
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
        )
        family_figures["image_slider_figure"] = (
            figures.image_slider_figure(
                [
                    ImageFrame(
                        time_s=float(i),
                        rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                        source_encoding="rgb8",
                    )
                    for i in range(2)
                ]
            )
        )
        body = "".join(
            html.figure_to_fragment(figure, f"fig-{name}")
            for name, figure in family_figures.items()
        )
        page_html = html.render_page(
            "Title", "index.html", body, []
        )
        self.assertEqual(
            page_html.count("Plotly.newPlot("),
            len(family_figures),
        )
        self.assertEqual(
            page_html.count(
                "window.__reportApplyPlotlyTheme("
                "document.getElementById("
            ),
            len(family_figures),
        )
        for name in family_figures:
            with self.subTest(figure=name):
                call = (
                    "window.__reportApplyPlotlyTheme("
                    f'document.getElementById("fig-{name}"))'
                )
                self.assertEqual(page_html.count(call), 1)


# Node.js driver: evaluates the real theme script with a
# stand-in DOM, matchMedia, and Plotly.relayout, then runs
# one scenario and prints a JSON result. SCRIPT, FIGURES and
# SCENARIO are substituted in before running node.
_NODE_DRIVER = r"""
const scriptSource = SCRIPT;
const figureSpecs = FIGURES;
const scenario = SCENARIO;
const listeners = [];
const media = { matches: scenario.initialDark,
  addEventListener: (type, fn) => {
    if (type === "change") listeners.push(fn);
  } };
const errors = [];
const unhandled = [];
process.on("unhandledRejection", (reason) => {
  unhandled.push(String(reason));
});
const divs = figureSpecs.map((spec) => ({
  id: spec.id, layout: spec.layout, attrs: {},
  setAttribute(name, value) { this.attrs[name] = value; }
}));
const calls = [];
global.window = { matchMedia: () => media,
  console: { error: (m) => errors.push(m) } };
global.console.error = (m) => errors.push(m);
global.document = { querySelectorAll: () => divs };
global.Plotly = { relayout: (div, update) => {
  calls.push({ id: div.id, update });
  const mode = (scenario.failures || {})[div.id];
  if (mode === "reject") {
    return Promise.reject(new Error("relayout rejected"));
  }
  if (mode === "throw") throw new Error("relayout threw");
  return Promise.resolve(div);
} };
eval(scriptSource);
(async () => {
  for (const div of divs) {
    await window.__reportApplyPlotlyTheme(div);
  }
  if (scenario.toggleTo !== undefined) {
    media.matches = scenario.toggleTo;
    listeners.forEach((fn) => fn({ matches: media.matches }));
  }
  await new Promise((resolve) => setTimeout(resolve, 20));
  console.log(JSON.stringify({ calls, errors, unhandled,
    attrs: Object.fromEntries(
      divs.map((d) => [d.id, d.attrs])) }));
})();
"""

# Every figure family's layout keys as the browser sees them:
# one-axis, three-row subplot with a bottom slider, and 3-D.
_FIGURE_SPECS = [
    {"id": "one-axis", "layout": {"xaxis": {}, "yaxis": {}}},
    {
        "id": "subplots",
        "layout": {
            "xaxis": {},
            "xaxis2": {},
            "xaxis3": {"rangeslider": {"visible": True}},
            "yaxis": {},
            "yaxis2": {},
            "yaxis3": {},
        },
    },
    {
        "id": "scene",
        "layout": {
            "xaxis": {},
            "yaxis": {},
            "scene": {"aspectmode": "data"},
        },
    },
]

# Every theme key ends in a color attribute; anything else
# would risk disturbing zoom or state on a theme change.
_COLOR_ONLY_KEY = re.compile(
    r"(^|\.)(color|gridcolor|linecolor|zerolinecolor|"
    r"bgcolor|bordercolor|backgroundcolor)$"
)


@unittest.skipUnless(
    shutil.which("node"), "node is not installed; JS tests skipped"
)
class ThemeScriptBehaviourTests(unittest.TestCase):
    """Tests for the generated theme script under Node."""

    def _run(self, scenario: dict) -> dict:
        """Run the Node driver for one scenario."""
        script_block = html._plotly_theme_script()
        script_source = re.search(
            r"<script>(.*)</script>", script_block, re.DOTALL
        ).group(1)
        driver = (
            _NODE_DRIVER.replace(
                "SCRIPT", json.dumps(script_source)
            )
            .replace("FIGURES", json.dumps(_FIGURE_SPECS))
            .replace("SCENARIO", json.dumps(scenario))
        )
        completed = subprocess.run(
            ["node", "-e", driver],
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
        return json.loads(completed.stdout.strip().splitlines()[-1])

    def _updates_by_id(
        self, result: dict
    ) -> dict[str, list[dict]]:
        """Group relayout updates by figure id."""
        grouped: dict[str, list[dict]] = {}
        for call in result["calls"]:
            grouped.setdefault(call["id"], []).append(call["update"])
        return grouped

    def test_initial_dark_theme_covers_every_axis_family(
        self,
    ) -> None:
        """Dark mode themes every axis family."""
        updates = self._updates_by_id(self._run({"initialDark": True}))
        self.assertEqual(
            sorted(updates), ["one-axis", "scene", "subplots"]
        )
        subplots = updates["subplots"][0]
        self.assertEqual(
            subplots["font.color"], CHROME_DARK["text_primary"]
        )
        for axis in (
            "xaxis",
            "xaxis2",
            "xaxis3",
            "yaxis",
            "yaxis2",
            "yaxis3",
        ):
            self.assertEqual(
                subplots[f"{axis}.gridcolor"],
                CHROME_DARK["gridline"],
            )
        self.assertIn("xaxis3.rangeslider.bordercolor", subplots)
        self.assertNotIn("xaxis.rangeslider.bordercolor", subplots)
        self.assertEqual(
            subplots["hoverlabel.bgcolor"], CHROME_DARK["surface"]
        )
        scene = updates["scene"][0]
        for axis in ("xaxis", "yaxis", "zaxis"):
            self.assertEqual(
                scene[f"scene.{axis}.backgroundcolor"],
                CHROME_DARK["surface"],
            )

    def test_initial_light_theme(self) -> None:
        """Light mode applies the light palette."""
        updates = self._updates_by_id(
            self._run({"initialDark": False})
        )
        one_axis = updates["one-axis"][0]
        self.assertEqual(
            one_axis["font.color"], CHROME_LIGHT["text_primary"]
        )
        self.assertEqual(
            one_axis["xaxis.gridcolor"], CHROME_LIGHT["gridline"]
        )

    def test_live_theme_change_rethemes_every_figure(self) -> None:
        """Live change re-themes with color keys only."""
        result = self._run({"initialDark": True, "toggleTo": False})
        updates = self._updates_by_id(result)
        for figure_id, figure_updates in updates.items():
            with self.subTest(figure=figure_id):
                self.assertEqual(len(figure_updates), 2)
                self.assertEqual(
                    figure_updates[0]["font.color"],
                    CHROME_DARK["text_primary"],
                )
                self.assertEqual(
                    figure_updates[1]["font.color"],
                    CHROME_LIGHT["text_primary"],
                )
                for update in figure_updates:
                    for key in update:
                        self.assertRegex(key, _COLOR_ONLY_KEY)

    def test_rejected_relayout_is_contained_and_reported(
        self,
    ) -> None:
        """Rejected relayout is contained and reported."""
        result = self._run(
            {"initialDark": True, "failures": {"one-axis": "reject"}}
        )
        updates = self._updates_by_id(result)
        self.assertEqual(
            sorted(updates), ["one-axis", "scene", "subplots"]
        )
        self.assertEqual(result["unhandled"], [])
        self.assertIn(
            "relayout rejected",
            result["attrs"]["one-axis"]["data-report-theme-error"],
        )
        self.assertNotIn(
            "data-report-theme-error", result["attrs"]["subplots"]
        )
        self.assertTrue(
            any(
                "one-axis" in message
                for message in result["errors"]
            )
        )

    def test_synchronous_relayout_throw_is_contained(self) -> None:
        """Synchronous throw is caught like a rejection."""
        result = self._run(
            {"initialDark": True, "failures": {"subplots": "throw"}}
        )
        updates = self._updates_by_id(result)
        self.assertIn("scene", updates)
        self.assertIn(
            "relayout threw",
            result["attrs"]["subplots"]["data-report-theme-error"],
        )
        self.assertEqual(result["unhandled"], [])


if __name__ == "__main__":
    unittest.main()
