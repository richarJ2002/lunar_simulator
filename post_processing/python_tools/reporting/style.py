"""!
@brief  This report's one design system: the fixed categorical palette
        (assigned by series identity, never cycled), status colors,
        typography, and the shared Plotly layout template every figure in
        python_tools.reporting.figures is built from, so every page reads
        as one system. Palette values are the dataviz skill's validated
        reference instance (accessible contrast, colorblind-safe adjacent
        pairs); this module does not re-derive or re-validate them.
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

from python_tools.data.models import WHEEL_ORDER

# Fixed-order categorical palette. Slots are assigned by series identity
# below (COLOR_*), never reassigned by a filter or a series' rank, per the
# dataviz skill's "color follows the entity" rule.
CATEGORICAL_COLORS: tuple[str, ...] = (
    "#2a78d6",  # 1 blue
    "#eb6834",  # 2 orange
    "#1baf7a",  # 3 aqua
    "#eda100",  # 4 yellow
    "#e87ba4",  # 5 magenta
    "#008300",  # 6 green
    "#4a3aa7",  # 7 violet
    "#e34948",  # 8 red
)

# Semantic series-color assignment, held constant across every page so a
# reader never has to re-learn what a color means: ground truth is always
# green, the primary estimate under test is always blue, and so on.
COLOR_GROUND_TRUTH = CATEGORICAL_COLORS[5]
COLOR_PRIMARY_ESTIMATE = CATEGORICAL_COLORS[0]
COLOR_SECONDARY_ESTIMATE = CATEGORICAL_COLORS[1]
COLOR_TERTIARY_ESTIMATE = CATEGORICAL_COLORS[6]
COLOR_RAW_SOURCE = CATEGORICAL_COLORS[3]
COLOR_ERROR = CATEGORICAL_COLORS[7]

# Fixed per-wheel color, in WHEEL_ORDER, reused by every six-wheel small
# multiple so a wheel's color is stable across every plot it appears in.
WHEEL_COLORS: dict[str, str] = dict(zip(WHEEL_ORDER, CATEGORICAL_COLORS))

# Status colors: reserved for state, never reused as a series color.
STATUS_GOOD = "#0ca30c"
STATUS_WARNING = "#fab219"
STATUS_SERIOUS = "#ec835a"
STATUS_CRITICAL = "#d03b3b"

# Diverging pair for signed residual/error bands: blue (negative) <-> red
# (positive) around a neutral gray midpoint.
DIVERGING_NEGATIVE = "#2a78d6"
DIVERGING_MIDPOINT = "#f0efec"
DIVERGING_POSITIVE = "#e34948"

# Sequential single-hue ramp (light -> dark blue) for continuous magnitude
# encodings (point-cloud depth, image intensity).
SEQUENTIAL_BLUE: tuple[str, ...] = (
    "#cde2fb",
    "#9ec5f4",
    "#5598e7",
    "#2a78d6",
    "#1c5cab",
    "#0d366b",
)

# Chart/page chrome, light and dark. Every page's <style> block is built
# from these two dicts so both modes stay in sync with this one source.
CHROME_LIGHT = {
    "surface": "#fcfcfb",
    "page": "#f9f9f7",
    "text_primary": "#0b0b0b",
    "text_secondary": "#52514e",
    "text_muted": "#898781",
    "gridline": "#e1e0d9",
    "axis": "#c3c2b7",
    "border": "rgba(11,11,11,0.10)",
}
CHROME_DARK = {
    "surface": "#1a1a19",
    "page": "#0d0d0d",
    "text_primary": "#ffffff",
    "text_secondary": "#c3c2b7",
    "text_muted": "#898781",
    "gridline": "#2c2c2a",
    "axis": "#383835",
    "border": "rgba(255,255,255,0.10)",
}

# Table cell rule color: a mid-gray at partial opacity, legible against
# both the light and the dark card surface, so a table needs no per-theme
# restyle of its trace.
TABLE_RULE_COLOR = "rgba(137,135,129,0.45)"

# The system sans stack, per the dataviz skill: no display/serif face.
FONT_FAMILY = 'system-ui, -apple-system, "Segoe UI", sans-serif'


def plotly_layout(
    x_title: str,
    y_title: str,
    legend: bool = True,
    height: int = 420,
) -> dict:
    """!
    @brief   Builds the shared Plotly `layout` dict every figure in
             python_tools.reporting.figures starts from: transparent
             backgrounds (so the figure blends into the surrounding card in
             either light or dark mode), unified font, recessive
             gridlines, and a responsive width.

    @param   x_title
             X-axis title. Shall already carry its unit, per the plan's
             "unit-bearing axis titles" requirement (e.g. "Time (s)").
    @param   y_title
             Y-axis title, unit-bearing (e.g. "Position error (m)").
    @param   legend
             Whether to reserve and show the legend.
    @param   height
             Figure height in pixels; width is left responsive.

    @return  A `layout` dict passable to `plotly.graph_objects.Figure`.
    """
    return {
        "template": "plotly_white",
        "autosize": True,
        "height": height,
        "margin": {"l": 64, "r": 24, "t": 32, "b": 56},
        "paper_bgcolor": "rgba(0,0,0,0)",
        "plot_bgcolor": "rgba(0,0,0,0)",
        "font": {"family": FONT_FAMILY, "size": 13, "color": CHROME_LIGHT["text_primary"]},
        "xaxis": {
            "title": {"text": x_title},
            "gridcolor": CHROME_LIGHT["gridline"],
            "linecolor": CHROME_LIGHT["axis"],
            "zerolinecolor": CHROME_LIGHT["axis"],
            "rangeslider": {},
        },
        "yaxis": {
            "title": {"text": y_title},
            "gridcolor": CHROME_LIGHT["gridline"],
            "linecolor": CHROME_LIGHT["axis"],
            "zerolinecolor": CHROME_LIGHT["axis"],
        },
        "showlegend": legend,
        "legend": {"orientation": "h", "y": -0.25},
        "hovermode": "x unified",
        # Disables Plotly's own figure-editing UI (title/axis drag-edit)
        # per the plan, while the modebar config below keeps zoom/pan/hover/
        # legend-toggle/image-export/reset available.
        "modebar": {"remove": ["editInChartStudio"]},
    }


def plotly_config() -> dict:
    """!
    @brief   Builds the shared Plotly `config` dict: disables editing,
             keeps zoom/pan/hover/legend-toggle/image-export/reset, and
             requires no CDN (the caller embeds a local plotly.min.js).

    @return  A `config` dict passable to `plotly.io.to_html`.
    """
    return {
        "displaylogo": False,
        "editable": False,
        "responsive": True,
        "modeBarButtonsToRemove": ["editInChartStudio", "sendDataToCloud"],
        "toImageButtonOptions": {"format": "png"},
    }


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone palette self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Print this report's categorical palette and semantic color assignments."
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the categorical palette and its
             semantic assignments, for quick manual reference.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    parser.parse_args(argv)
    print("Categorical palette:", ", ".join(CATEGORICAL_COLORS))
    print("Ground truth:", COLOR_GROUND_TRUTH)
    print("Primary estimate:", COLOR_PRIMARY_ESTIMATE)
    print("Wheel colors:", WHEEL_COLORS)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
