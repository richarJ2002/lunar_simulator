"""!
@brief  Reusable Plotly figure constructors shared by every subsystem
        report page: three-axis time-series subplots, 2-D/3-D trajectory
        plots, estimate/truth/error panels with optional +/-3-sigma bands,
        six-wheel small multiples, rate/count plots, summary tables,
        sampled image sliders, bounded point-cloud snapshots, and
        empty-state warning cards. Every constructor takes already-computed
        arrays (from python_tools.data.metrics/alignment) and returns a
        `plotly.graph_objects.Figure` or an HTML snippet -- no bag, log or
        file access here.

        The four constructors that plot a raw, possibly high-rate time
        series (`three_axis_time_series`, `six_wheel_small_multiples`,
        `estimate_truth_error_panel`, `rate_count_plot`) transparently
        decimate what they plot to `DEFAULT_MAXIMUM_DISPLAY_POINTS` per
        trace via `_decimated_indices` -- see Defect 3 in the plan's
        2026-09-22 audit, where the undecimated ~1 kHz driver joint-state
        feed alone made a real generated wheel_odometry.html 45 MB.
        Decimation only ever affects what is rendered here; every metric
        in python_tools.data.metrics is computed upstream, in the
        entry-point scripts, against the full-resolution arrays before
        they ever reach this module.

        Every 2-D line trace is an SVG `go.Scatter`, never a WebGL
        `go.Scattergl` (2026-09-23 rendering fix): each `Scattergl` figure
        holds two WebGL contexts, a report page carries up to ~13 such
        figures, and browsers cap live WebGL contexts per page (Chromium
        at 16), silently evicting the oldest -- measured in headless
        Firefox with its context cap lowered to Chromium's 16, the first
        11 of the Kalman page's 13 figures drew no trace pixels at all.
        Display decimation already bounds
        each time-series trace at `DEFAULT_MAXIMUM_DISPLAY_POINTS`, well
        within SVG's comfortable range, so WebGL bought nothing here.
"""

from __future__ import annotations

import argparse
import base64
import html as html_module
import struct
import zlib
from typing import Collection, Optional, Sequence

import numpy as np
import numpy.typing as npt
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from python_tools.data.models import ImageFrame, PointCloudSnapshot, WHEEL_ORDER
from python_tools.reporting.style import (
    CATEGORICAL_COLORS,
    CHROME_LIGHT,
    COLOR_ERROR,
    DIVERGING_MIDPOINT,
    FONT_FAMILY,
    TABLE_RULE_COLOR,
    WHEEL_COLORS,
    plotly_layout,
)

# Documented upper bound on the number of samples any one decimated trace
# renders, per Defect 3 in the plan's 2026-09-22 audit. Applies only to
# display (see `_decimated_indices`); never to metric computation.
DEFAULT_MAXIMUM_DISPLAY_POINTS = 2000

# Top margin for the 2x3 wheel small multiples: the modebar occupies roughly
# the top 30 px of a figure, so the top row's subplot titles need to start
# below it (2026-09-23: at the shared 32 px margin the modebar covered the
# top-right "centre left" title, measured in headless Chromium).
SMALL_MULTIPLES_TOP_MARGIN_PX = 64

# Left margin for the same grid, wide enough for its tick labels plus the
# single rotated Y title annotation placed left of them.
SMALL_MULTIPLES_LEFT_MARGIN_PX = 84


def _segment_boundaries(times_s: npt.NDArray[np.float64]) -> list[int]:
    """!
    @brief   Finds discontinuity boundaries in a time series for display
             decimation, so a plotted line never visually bridges a real
             discontinuity. Mirrors python_tools.bag.timestamps.
             segment_series's own two discontinuity conditions -- a
             strictly backward time jump, or a forward gap much larger
             than the series' own typical spacing -- rather than only
             detecting large forward gaps (2026-09-23 correction: the
             previous version here only checked `diff > gap_threshold`,
             so a genuine backward time jump, e.g. a clock reset, was
             invisible to it even though `segment_series` itself always
             treats one as a discontinuity regardless of magnitude). A
             repeated timestamp (a zero diff) is not, by itself, a
             discontinuity here, matching `segment_series`'s own strict
             `<` (not `<=`) backward-jump condition.

    @param   times_s
             Sample times, seconds, shape (N,).

    @return  Sorted boundary indices `[0, ..., N]` such that consecutive
             pairs define half-open `[start, end)` segments covering every
             index of `times_s` exactly once.
    """
    sample_count = times_s.size
    if sample_count <= 1:
        return [0, sample_count]
    diffs = np.diff(times_s)
    # A gap much larger than the series' own typical (median positive)
    # inter-sample spacing marks a discontinuity; segments never blend
    # bucket ranking or first/last selection across one.
    positive_diffs = diffs[diffs > 0]
    typical_gap = float(np.median(positive_diffs)) if positive_diffs.size else 0.0
    gap_threshold = max(typical_gap * 10.0, 1e-9)
    is_boundary = (diffs < 0) | (diffs > gap_threshold)
    boundary_positions = (np.nonzero(is_boundary)[0] + 1).tolist()
    return [0, *boundary_positions, sample_count]


def _evenly_spaced_positions(total_count: int, maximum_count: int) -> list[int]:
    """!
    @brief   Picks up to `maximum_count` positions, evenly spaced across
             `[0, total_count)`, sorted and unique. Used both to select a
             representative subset of segments across the whole run and,
             as a final safety net, to trim an over-assembled index set
             down to the hard cap -- in both uses, deterministic and
             never biased toward the start of the range (2026-09-23
             correction: `_decimated_indices` no longer relies on any
             selection step that could silently favor early segments).

    @param   total_count
             The number of positions available to choose from.
    @param   maximum_count
             The largest number of positions to return.

    @return  A sorted list of unique positions, length
             `min(total_count, maximum_count)`. For `maximum_count >= 2`,
             both the first and last position are always included. A
             single requested position (`maximum_count == 1`) resolves to
             the LAST position, not the first -- matching
             python_tools.data.extractors._BoundedEvenSampler's identical
             one-slot policy: a single representative is more useful as
             "how the run ended" than "how it began".
    """
    if total_count <= 0 or maximum_count <= 0:
        return []
    if maximum_count == 1:
        return [total_count - 1]
    if maximum_count >= total_count:
        return list(range(total_count))
    positions = np.linspace(0, total_count - 1, num=maximum_count)
    return sorted(set(int(round(position)) for position in positions))


def _decimated_indices(
    times_s: npt.NDArray[np.float64],
    magnitude: npt.NDArray[np.float64],
    maximum_points: int = DEFAULT_MAXIMUM_DISPLAY_POINTS,
) -> npt.NDArray[np.intp]:
    """!
    @brief   Selects a bounded, deterministic set of display indices from
             a time series, for `Figure` construction only -- never for
             metric computation. `len(result) <= maximum_points` always
             holds unconditionally (2026-09-23 correction: the previous
             version's per-segment "at least 2 points" floor had no global
             cap and could overshoot `maximum_points` several-fold when a
             series carried many short, genuinely separate segments --
             e.g. 1000 real 5-sample bursts separated by real gaps
             returned ~3350 points against a 2000-point budget; see the
             plan's 2026-09-23 correction pass).

             Splits on a discontinuity via `_segment_boundaries` (a
             backward time jump, or a forward gap much larger than the
             series' own typical spacing), then:
             - if every detected segment's own start/end fits within the
               budget, every segment is represented, and any leftover
               budget is distributed proportionally to segment length
               (largest-remainder method, so shares sum to exactly the
               leftover) for additional min/max bucket points -- each
               bucket contributes both its minimum- and maximum-magnitude
               sample, preserving a brief spike or dropout far better than
               naive every-Nth sampling;
             - otherwise, there are too many segments to represent every
               one of them within the budget: a deterministic, evenly
               spread subset of segments is chosen instead (covering the
               beginning, middle, and end of the run, never biased toward
               the start), and each selected segment shows its own
               start/end. The discontinuities between unselected segments
               are **not** individually visible in the decimated result --
               this is a genuine, documented trade-off of a hard display
               budget against an unbounded number of real discontinuities,
               not a claim that every segment is still shown.
             The true global first and last sample of the whole series are
             always kept when `maximum_points >= 2`. A final, deterministic,
             evenly spread safety trim (never a prefix/early-biased
             truncation) is applied if the above ever assembles more than
             `maximum_points` indices regardless of the rounding above, so
             the caller-facing bound holds unconditionally.

    @param   times_s
             Sample times, seconds, shape (N,), ascending within each
             segment.
    @param   magnitude
             A single ranking signal, shape (N,), used only to choose
             which samples best represent each bucket (e.g. a per-sample
             vector norm or `abs(error)`); NaN is treated as 0 for ranking
             purposes only -- the actual plotted values at selected
             indices are untouched.
    @param   maximum_points
             The largest number of samples the result may contain, spread
             across every detected segment. When `times_s` already has at
             most this many samples, every index is kept and no
             segmentation or bucketing runs.

    @return  A sorted, deduplicated array of selected indices into
             `times_s`, shape (M,), `M <= maximum_points`, always.
    """
    sample_count = times_s.size
    if sample_count == 0 or maximum_points <= 0:
        return np.array([], dtype=np.intp)
    if sample_count <= maximum_points:
        return np.arange(sample_count)
    if maximum_points == 1:
        # A single point cannot represent both endpoints; the final
        # sample is the more useful one to keep (see
        # _evenly_spaced_positions's identical one-slot policy).
        return np.array([sample_count - 1], dtype=np.intp)

    segment_starts = _segment_boundaries(times_s)
    segments = [
        (start, end)
        for start, end in zip(segment_starts[:-1], segment_starts[1:])
        if end > start
    ]
    num_segments = len(segments)
    safe_magnitude = np.nan_to_num(magnitude, nan=0.0, posinf=0.0, neginf=0.0)

    # The true global first and last sample are always reserved first.
    selected: set[int] = {0, sample_count - 1}
    remaining_budget = maximum_points - len(selected)

    # What every segment showing its own start/end (or its one sample, for
    # a length-1 segment) would cost -- the minimum footprint for "every
    # segment is represented at all".
    base_costs = [min(end - start, 2) for start, end in segments]
    base_total = sum(base_costs)

    if base_total <= remaining_budget:
        # Every segment fits its own start/end within budget: represent
        # all of them, then distribute the leftover proportionally to
        # segment length for additional min/max bucket points inside each
        # one.
        for start, end in segments:
            selected.add(start)
            selected.add(end - 1)
        leftover = remaining_budget - base_total
        if leftover > 0 and num_segments > 0:
            lengths = np.array([end - start for start, end in segments], dtype=np.float64)
            raw_shares = leftover * lengths / lengths.sum()
            shares = np.floor(raw_shares).astype(np.intp)
            # Largest-remainder method: hand the leftover single-point
            # remainder to the segments with the largest fractional share,
            # so the shares sum to exactly `leftover`, never more.
            remainder = leftover - int(shares.sum())
            if remainder > 0:
                fractional = raw_shares - shares
                for index in np.argsort(-fractional)[:remainder]:
                    shares[index] += 1
            for (start, end), share in zip(segments, shares):
                length = end - start
                if share <= 0 or length <= 2:
                    continue
                bucket_count = max(1, int(share) // 2)
                bucket_edges = np.linspace(0, length, bucket_count + 1).astype(np.intp)
                for bucket_start, bucket_end in zip(bucket_edges[:-1], bucket_edges[1:]):
                    if bucket_end <= bucket_start:
                        continue
                    bucket = safe_magnitude[start + bucket_start : start + bucket_end]
                    selected.add(start + bucket_start + int(np.argmin(bucket)))
                    selected.add(start + bucket_start + int(np.argmax(bucket)))
    elif num_segments > 0:
        # Too many segments to represent every one of them within budget:
        # select a deterministic, evenly spread subset instead (covering
        # the beginning, middle, and end of the run) and show each
        # selected segment's own start/end. See this function's own
        # docstring for what this sacrifices.
        target_segment_count = max(1, remaining_budget // 2)
        for segment_index in _evenly_spaced_positions(
            num_segments, min(target_segment_count, num_segments)
        ):
            start, end = segments[segment_index]
            selected.add(start)
            selected.add(end - 1)

    result = np.array(sorted(selected), dtype=np.intp)
    if result.size > maximum_points:
        # Deterministic, evenly spread safety trim: guarantees the hard
        # cap unconditionally regardless of any rounding above. The global
        # first/last (already members of `selected`) remain the endpoints
        # this trim itself always keeps.
        trim_positions = _evenly_spaced_positions(result.size, maximum_points)
        result = result[trim_positions]
    return result


def _apply_subplot_layout(
    figure: go.Figure,
    x_title: str,
    bottom_row: int,
    height: int,
    legend: bool = True,
) -> None:
    """!
    @brief   Applies the shared `style.plotly_layout` to a `make_subplots`
             figure without its single-axis `xaxis`/`yaxis` entries
             landing on the top-left subplot only (2026-09-23 rendering
             fix: `update_layout(xaxis=..., yaxis=...)` addresses the
             *first* subplot's axes, so the previous direct call put the
             "Time (s)" title on the top row, erased the top row's own
             Y-axis title set earlier via `update_yaxes`, and styled only
             the top row's gridlines/axis lines). Axis styling is instead
             applied to every
             subplot axis, and the X title to the bottom row only; no
             range slider is enabled here -- a caller enables one
             explicitly on its bottom row.

    @param   figure
             The subplot figure to style, modified in place.
    @param   x_title
             Unit-bearing X-axis title, shown on the bottom row.
    @param   bottom_row
             One-based index of the figure's bottom subplot row.
    @param   height
             Figure height in pixels.
    @param   legend
             Whether to show the legend.

    @return  None
    """
    layout = plotly_layout(x_title, "", legend=legend, height=height)
    # Split the single-axis entries out so they can be applied to every
    # subplot axis rather than only to `xaxis`/`yaxis` (the first subplot).
    x_axis_style = layout.pop("xaxis")
    y_axis_style = layout.pop("yaxis")
    # Titles are per-axis, and the range slider is opt-in per caller.
    for axis_style in (x_axis_style, y_axis_style):
        axis_style.pop("title", None)
        axis_style.pop("rangeslider", None)
    figure.update_layout(**layout)
    figure.update_xaxes(**x_axis_style)
    figure.update_yaxes(**y_axis_style)
    # One X title, on the bottom row, where the shared tick labels are.
    figure.update_xaxes(title_text=x_title, row=bottom_row)


def three_axis_time_series(
    times_s: npt.NDArray[np.float64],
    series: Sequence[tuple[str, npt.NDArray[np.float64], str]],
    axis_labels: tuple[str, str, str],
    y_unit: str,
    x_title: str = "Time (s)",
) -> go.Figure:
    """!
    @brief   Builds a three-row, x-linked time-series figure with a shared
             range slider, one row per axis, overlaying every supplied
             series on each row.

    @param   times_s
             Sample times, seconds, shape (N,); shared by every series.
    @param   series
             `(label, values, color)` tuples, `values` shape (N, 3); a
             series may legitimately share `times_s` with a different
             length than another series only if pre-aligned by the caller
             -- this function assumes all inputs share `times_s`.
    @param   axis_labels
             Row titles, one per axis (e.g. `("X", "Y", "Z")`).
    @param   y_unit
             Unit string appended to every row's Y-axis title.
    @param   x_title
             X-axis title, shown once on the bottom row.

    @return  The assembled figure.
    """
    # Decimate for display only (see this module's docstring and
    # `_decimated_indices`): rank by the largest per-sample 3-vector norm
    # across every overlaid series, so a spike in any one of them is still
    # preserved, then apply the same selected indices to every series and
    # to times_s so channels and timestamps stay aligned.
    combined_magnitude = np.max(
        np.stack([np.linalg.norm(values, axis=-1) for _, values, _ in series], axis=0),
        axis=0,
    )
    indices = _decimated_indices(times_s, combined_magnitude)
    display_times_s = times_s[indices]

    figure = make_subplots(rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.06)
    for axis_index in range(3):
        for label, values, color in series:
            figure.add_trace(
                go.Scatter(
                    x=display_times_s,
                    y=values[indices, axis_index],
                    mode="lines",
                    name=label,
                    legendgroup=label,
                    showlegend=(axis_index == 0),
                    line={"color": color, "width": 2},
                ),
                row=axis_index + 1,
                col=1,
            )
        # The unit goes on its own line: each row is only ~110 px tall, and
        # a long single-line label (e.g. "Received (count / interval)")
        # otherwise overruns into the neighbouring rows' titles.
        figure.update_yaxes(
            title_text=f"{axis_labels[axis_index]}<br>({y_unit})", row=axis_index + 1, col=1
        )
    _apply_subplot_layout(figure, x_title, bottom_row=3, height=560)
    # A range slider only needs to be declared once, on the bottom row;
    # with shared_xaxes it governs every linked row.
    figure.update_xaxes(rangeslider={"visible": True}, row=3, col=1)
    return figure


def trajectory_xy(
    series: Sequence[tuple[str, npt.NDArray[np.float64], str]],
    x_title: str = "X (m)",
    y_title: str = "Y (m)",
    legend_only_labels: Collection[str] = (),
) -> go.Figure:
    """!
    @brief   Builds a 2-D XY trajectory plot with an equal aspect ratio, so
             a straight-line drive is not visually distorted into a curve.

    @param   series
             `(label, positions, color)` tuples, `positions` shape (N, 3)
             (only X/Y are used).
    @param   x_title
             X-axis title.
    @param   y_title
             Y-axis title.
    @param   legend_only_labels
             Labels whose traces start as `legendonly`: hidden and excluded
             from autorange, but one legend click away. Used for a
             diagnostic-only series whose scale (e.g. an unaided inertial
             trace reaching kilometres) would otherwise shrink every other
             trace to a dot.

    @return  The assembled figure.
    """
    figure = go.Figure()
    for label, positions, color in series:
        figure.add_trace(
            go.Scatter(
                x=positions[:, 0],
                y=positions[:, 1],
                mode="lines",
                name=label,
                line={"color": color, "width": 2},
                visible="legendonly" if label in legend_only_labels else None,
            )
        )
    layout = plotly_layout(x_title, y_title, height=480)
    # An equal scaleratio keeps one metre on X visually equal to one metre
    # on Y, so path curvature and heading are not distorted.
    layout["yaxis"]["scaleanchor"] = "x"
    layout["yaxis"]["scaleratio"] = 1
    figure.update_layout(**layout)
    return figure


def trajectory_3d(
    series: Sequence[tuple[str, npt.NDArray[np.float64], str]],
) -> go.Figure:
    """!
    @brief   Builds a 3-D trajectory plot with an equal aspect ratio across
             all three axes.

    @param   series
             `(label, positions, color)` tuples, `positions` shape (N, 3).

    @return  The assembled figure.
    """
    figure = go.Figure()
    for label, positions, color in series:
        figure.add_trace(
            go.Scatter3d(
                x=positions[:, 0],
                y=positions[:, 1],
                z=positions[:, 2],
                mode="lines",
                name=label,
                line={"color": color, "width": 4},
            )
        )
    layout = plotly_layout("X (m)", "Y (m)", height=560)
    layout["scene"] = {
        "aspectmode": "data",
        "xaxis": {"title": {"text": "X (m)"}},
        "yaxis": {"title": {"text": "Y (m)"}},
        "zaxis": {"title": {"text": "Z (m)"}},
    }
    figure.update_layout(**layout)
    return figure


def estimate_truth_error_panel(
    times_s: npt.NDArray[np.float64],
    estimate: npt.NDArray[np.float64],
    truth: npt.NDArray[np.float64],
    error: npt.NDArray[np.float64],
    value_title: str,
    error_title: str,
    sigma_band: Optional[npt.NDArray[np.float64]] = None,
) -> go.Figure:
    """!
    @brief   Builds a two-row figure: the estimate overlaid on ground
             truth (top row), and the resulting error with an optional
             +/-3-sigma consistency band (bottom row).

    @param   times_s
             Sample times, seconds, shape (N,).
    @param   estimate
             Estimator scalar value, shape (N,).
    @param   truth
             Interpolated ground-truth scalar value, shape (N,); NaN where
             unaligned (rendered as a gap, not interpolated across).
    @param   error
             Signed error, shape (N,); NaN where unaligned.
    @param   value_title
             Y-axis title for the top row.
    @param   error_title
             Y-axis title for the bottom row.
    @param   sigma_band
             Reported one-sigma standard deviation, shape (N,), if the
             source publishes a usable covariance; `None` to omit the band.

    @return  The assembled figure.
    """
    # Decimate for display only (see this module's docstring and
    # `_decimated_indices`): rank by the error magnitude (the most
    # diagnostically important channel here), then apply the same
    # selected indices to every channel so they stay aligned.
    indices = _decimated_indices(times_s, np.abs(error))
    display_times_s = times_s[indices]
    display_estimate = estimate[indices]
    display_truth = truth[indices]
    display_error = error[indices]
    display_sigma_band = sigma_band[indices] if sigma_band is not None else None

    figure = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.08)
    figure.add_trace(
        go.Scatter(
            x=display_times_s,
            y=display_estimate,
            mode="lines",
            name="Estimate",
            line={"color": CATEGORICAL_COLORS[0], "width": 2},
        ),
        row=1,
        col=1,
    )
    figure.add_trace(
        go.Scatter(
            x=display_times_s,
            y=display_truth,
            mode="lines",
            name="Ground truth",
            line={"color": CATEGORICAL_COLORS[5], "width": 2, "dash": "dot"},
        ),
        row=1,
        col=1,
    )
    if display_sigma_band is not None:
        # A filled +/-3-sigma band drawn as two traces sharing one fill,
        # the standard Plotly "upper then lower with fill='tonexty'" idiom.
        three_sigma = 3.0 * display_sigma_band
        figure.add_trace(
            go.Scatter(
                x=display_times_s,
                y=display_error + three_sigma,
                mode="lines",
                line={"width": 0},
                showlegend=False,
                hoverinfo="skip",
            ),
            row=2,
            col=1,
        )
        figure.add_trace(
            go.Scatter(
                x=display_times_s,
                y=display_error - three_sigma,
                mode="lines",
                line={"width": 0},
                fill="tonexty",
                fillcolor="rgba(42,120,214,0.15)",
                name="+/-3 sigma",
                hoverinfo="skip",
            ),
            row=2,
            col=1,
        )
    figure.add_trace(
        go.Scatter(
            x=display_times_s,
            y=display_error,
            mode="lines",
            name="Error",
            line={"color": COLOR_ERROR, "width": 2},
        ),
        row=2,
        col=1,
    )
    figure.add_hline(y=0, line={"color": DIVERGING_MIDPOINT, "width": 1}, row=2, col=1)
    figure.update_yaxes(title_text=value_title, row=1, col=1)
    figure.update_yaxes(title_text=error_title, row=2, col=1)
    _apply_subplot_layout(figure, "Time (s)", bottom_row=2, height=520)
    # One range slider, on the bottom row; shared_xaxes links the top row.
    figure.update_xaxes(rangeslider={"visible": True}, row=2, col=1)
    return figure


def six_wheel_small_multiples(
    times_s: npt.NDArray[np.float64],
    values: npt.NDArray[np.float64],
    y_title: str,
) -> go.Figure:
    """!
    @brief   Builds a 2x3 grid of small multiples, one per wheel in
             WHEEL_ORDER, using each wheel's fixed color.

    @param   times_s
             Sample times, seconds, shape (N,).
    @param   values
             Per-wheel values, shape (N, 6), columns in WHEEL_ORDER.
    @param   y_title
             Shared Y-axis title (unit-bearing).

    @return  The assembled figure.
    """
    # Decimate for display only (see this module's docstring and
    # `_decimated_indices`): rank by the largest magnitude across all six
    # wheels, so a spike on any one wheel is still preserved, then apply
    # the same selected indices to every wheel so they stay aligned. This
    # is exactly the ~1 kHz raw joint-state feed that made a real
    # generated wheel_odometry.html 45 MB (Defect 3 in the plan's
    # 2026-09-22 audit).
    indices = _decimated_indices(times_s, np.max(np.abs(values), axis=-1))
    display_times_s = times_s[indices]

    figure = make_subplots(
        rows=2,
        cols=3,
        shared_xaxes=True,
        subplot_titles=[name.replace("_", " ") for name in WHEEL_ORDER],
    )
    for index, wheel_name in enumerate(WHEEL_ORDER):
        row = index // 3 + 1
        col = index % 3 + 1
        figure.add_trace(
            go.Scatter(
                x=display_times_s,
                y=values[indices, index],
                mode="lines",
                name=wheel_name.replace("_", " "),
                line={"color": WHEEL_COLORS[wheel_name], "width": 2},
                showlegend=False,
            ),
            row=row,
            col=col,
        )
    _apply_subplot_layout(figure, "Time (s)", bottom_row=2, height=440, legend=False)
    # One Y title centred across both rows (2026-09-23 rendering fix):
    # repeated on each ~150 px row, a long title such as "Public wheel
    # command velocity (rad/s)" was taller than its row and overflowed the
    # figure at both edges. As a paper-referenced annotation it inherits the
    # themed `font.color`, like the subplot titles.
    figure.add_annotation(
        text=y_title,
        textangle=-90,
        xref="paper",
        yref="paper",
        x=0.0,
        y=0.5,
        xanchor="right",
        yanchor="middle",
        xshift=-(SMALL_MULTIPLES_LEFT_MARGIN_PX - 24),
        showarrow=False,
    )
    figure.update_layout(
        margin={"t": SMALL_MULTIPLES_TOP_MARGIN_PX, "l": SMALL_MULTIPLES_LEFT_MARGIN_PX}
    )
    return figure


def rate_count_plot(
    times_s: npt.NDArray[np.float64],
    values: npt.NDArray[np.float64],
    y_title: str,
    color: str = CATEGORICAL_COLORS[0],
    x_title: str = "Time (s)",
) -> go.Figure:
    """!
    @brief   Builds a simple single-series line plot for a rate, count or
             latency series.

    @param   times_s
             Sample times, seconds, shape (N,).
    @param   values
             Values to plot, shape (N,).
    @param   y_title
             Y-axis title (unit-bearing).
    @param   color
             Line color.
    @param   x_title
             X-axis title. Defaults to bag-elapsed "Time (s)"; callers
             plotting diagnostic-log-derived series (which are not on the
             bag's elapsed-time axis -- see python_tools.diagnostics.
             log_parser's docstring) shall override this to make that
             explicit.

    @return  The assembled figure.
    """
    # Decimate for display only (see this module's docstring and
    # `_decimated_indices`), ranking by the series' own magnitude.
    indices = _decimated_indices(times_s, np.abs(values))
    figure = go.Figure(
        go.Scatter(
            x=times_s[indices],
            y=values[indices],
            mode="lines+markers",
            line={"color": color, "width": 2},
        )
    )
    layout = plotly_layout(x_title, y_title, legend=False, height=320)
    figure.update_layout(**layout)
    return figure


def summary_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> go.Figure:
    """!
    @brief   Builds a Plotly summary table.

    @param   headers
             Column headers.
    @param   rows
             Row values, each the same length as `headers`.

    @return  The assembled figure.
    """
    columns = list(zip(*rows)) if rows else [[] for _ in headers]
    figure = go.Figure(
        go.Table(
            header={
                "values": list(headers),
                "fill_color": CATEGORICAL_COLORS[0],
                "font": {"color": "white"},
                "line_color": TABLE_RULE_COLOR,
                "align": "left",
            },
            # Transparent cells (2026-09-23 rendering fix): the default
            # template's fixed pale-blue cell fill stayed light in dark
            # mode while the themed text turned white, leaving the body
            # unreadable. With no fill, the card surface shows through and
            # the text follows the themed `font.color` in either mode.
            cells={
                "values": columns,
                "fill_color": "rgba(0,0,0,0)",
                "line_color": TABLE_RULE_COLOR,
                "align": "left",
            },
        )
    )
    figure.update_layout(
        margin={"l": 0, "r": 0, "t": 0, "b": 0},
        height=44 + 28 * max(1, len(rows)),
        paper_bgcolor="rgba(0,0,0,0)",
        # Same initial (light) text color and font as every other figure;
        # the page's theme script re-colors it for dark mode.
        font={"family": FONT_FAMILY, "size": 13, "color": CHROME_LIGHT["text_primary"]},
    )
    return figure


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    """!
    @brief   Builds one length-prefixed, CRC-suffixed PNG chunk.

    @param   chunk_type
             The four-byte chunk type code (e.g. `b"IHDR"`).
    @param   data
             The chunk's payload.

    @return  The complete chunk, ready to append to a PNG byte stream.
    """
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
    )


def _encode_png_data_uri(rgb: npt.NDArray[np.uint8]) -> str:
    """!
    @brief   Encodes an (H, W, 3) RGB array as a base64 PNG data URI,
             using only the standard library (`zlib`/`struct`/`base64`),
             so embedding sampled frames does not add a Pillow/image
             dependency beyond the plan's NumPy/Plotly/PyYAML set. Used
             instead of a `go.Image` trace, whose `z` pixel array is
             embedded as JSON numbers -- 5-10x larger than a raw byte
             stream even before compression -- which made a full report
             page hundreds of megabytes for a real camera-resolution
             sampled sequence.

    @param   rgb
             Decoded RGB pixel data, shape (H, W, 3).

    @return  A `data:image/png;base64,...` URI string.
    """
    height, width, _ = rgb.shape
    # PNG requires one filter-type byte (0 = None) prefixed to every
    # scanline before the row's raw pixel bytes.
    raw_scanlines = b"".join(
        b"\x00" + rgb[row].tobytes() for row in range(height)
    )
    compressed = zlib.compress(raw_scanlines, level=6)
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit truecolor RGB
    png_bytes = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", compressed)
        + _png_chunk(b"IEND", b"")
    )
    return "data:image/png;base64," + base64.b64encode(png_bytes).decode("ascii")


def image_slider_figure(frames: Sequence[ImageFrame]) -> go.Figure:
    """!
    @brief   Builds a sampled image figure with a time slider, one frame
             per `ImageFrame`, rendered as a compressed PNG data URI
             (`layout.images`) rather than a raw-pixel `go.Image` trace so
             the page stays a reasonable size.

    @param   frames
             The evenly sampled, decoded frames to display.

    @return  The assembled figure.
    """
    figure = go.Figure()
    if not frames:
        return figure
    height, width, _ = frames[0].rgb.shape

    def _image_spec(data_uri: str) -> dict:
        """!
        @brief   Builds one `layout.images` entry for a frame's data URI.

        @param   data_uri
                 The frame's PNG data URI.

        @return  The image spec dict.
        """
        return {
            "source": data_uri,
            "xref": "x",
            "yref": "y",
            "x": 0,
            "y": 0,
            "sizex": width,
            "sizey": height,
            "sizing": "stretch",
            "layer": "below",
        }

    data_uris = [_encode_png_data_uri(frame.rgb) for frame in frames]
    figure.update_layout(images=[_image_spec(data_uris[0])])
    if len(frames) > 1:
        steps = [
            {
                "method": "relayout",
                "args": [{"images": [_image_spec(data_uri)]}],
                "label": f"{frame.time_s:.2f}s",
            }
            for frame, data_uri in zip(frames, data_uris)
        ]
        figure.update_layout(
            sliders=[{"active": 0, "currentvalue": {"prefix": "t="}, "steps": steps}]
        )
    figure.update_layout(
        margin={"l": 0, "r": 0, "t": 0, "b": 40},
        paper_bgcolor="rgba(0,0,0,0)",
        height=height / width * 600 + 80 if width else 420,
    )
    # A fixed-range, equal-aspect, inverted-Y image coordinate system so
    # the frame renders at its correct proportions without axis chrome.
    figure.update_xaxes(visible=False, range=[0, width], constrain="domain")
    figure.update_yaxes(
        visible=False, range=[height, 0], scaleanchor="x", scaleratio=1
    )
    return figure


def point_cloud_3d_slider(snapshots: Sequence[PointCloudSnapshot]) -> go.Figure:
    """!
    @brief   Builds a 3-D point-cloud figure with a time slider across a
             bounded set of snapshots.

    @param   snapshots
             The evenly sampled, bounded point-cloud snapshots to display.

    @return  The assembled figure.
    """
    figure = go.Figure()
    for snapshot in snapshots:
        points = snapshot.points_m
        figure.add_trace(
            go.Scatter3d(
                x=points[:, 0],
                y=points[:, 1],
                z=points[:, 2],
                mode="markers",
                marker={"size": 2, "color": CATEGORICAL_COLORS[0]},
                visible=False,
                name=(
                    "PnP inliers" if snapshot.is_inlier_cloud else "Reconstructed points"
                ),
            )
        )
    # Open on the first snapshot that has points (2026-09-23 rendering fix):
    # a real capture's first sampled cloud is published before visual
    # odometry initializes and holds zero points, which left the panel blank
    # on every load. All-empty input falls back to the first snapshot.
    initial_index = next(
        (index for index, snapshot in enumerate(snapshots) if len(snapshot.points_m) > 0), 0
    )
    if figure.data:
        figure.data[initial_index].visible = True
    if len(snapshots) > 1:
        steps = [
            {
                "method": "update",
                "args": [{"visible": [index == i for i in range(len(snapshots))]}],
                "label": f"{snapshot.time_s:.2f}s",
            }
            for index, snapshot in enumerate(snapshots)
        ]
        figure.update_layout(
            sliders=[{"active": initial_index, "currentvalue": {"prefix": "t="}, "steps": steps}]
        )
    layout = plotly_layout("X (m)", "Y (m)", legend=False, height=520)
    layout["scene"] = {"aspectmode": "data"}
    figure.update_layout(**layout)
    return figure


def empty_state_card_html(message: str, severity: str = "warning") -> str:
    """!
    @brief   Builds a self-contained HTML warning/empty-state card for a
             topic that was not recorded, published zero messages, or was
             disabled by the snapshotted parameters, per the plan's
             requirement that this be visible rather than an empty plot.

    @param   message
             The human-readable explanation to show; HTML-escaped before
             embedding (Defect 7 in the plan's 2026-09-22 audit: every
             other HTML-producing function in this codebase escapes its
             dynamic text via python_tools.reporting.html.escape, but this
             one did not -- not currently exploitable given today's
             call sites pass only fixed topic-name-based strings, but a
             latent risk for any future caller passing less-controlled
             text, e.g. a parameter value).
    @param   severity
             One of "warning", "serious" or "critical"; selects the
             card's accent color.

    @return  The card's HTML fragment.
    """
    color_by_severity = {
        "warning": "var(--status-warning)",
        "serious": "var(--status-serious)",
        "critical": "var(--status-critical)",
    }
    accent = color_by_severity.get(severity, color_by_severity["warning"])
    return (
        f'<div class="empty-state-card" style="border-left-color:{accent}">'
        f"<strong>{severity.capitalize()}:</strong> {html_module.escape(message, quote=True)}"
        "</div>"
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone self-check mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Render a tiny sample figure to a standalone HTML file, for manual visual inspection."
    )
    parser.add_argument("output_path", help="Path to write the sample HTML file to.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: writes a small sample figure to an HTML file
             for manual visual inspection of this module's style.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    times_s = np.linspace(0, 10, 100)
    values = np.stack([np.sin(times_s), np.cos(times_s), times_s * 0.1], axis=1)
    figure = three_axis_time_series(
        times_s, [("sample", values, CATEGORICAL_COLORS[0])], ("X", "Y", "Z"), "m"
    )
    figure.write_html(args.output_path, include_plotlyjs="cdn")
    print(f"wrote {args.output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
