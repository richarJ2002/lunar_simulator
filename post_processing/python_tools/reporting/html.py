"""!
@brief  The report's semantic HTML shell: the shared page template, nav
        bar, run-summary header, warnings section, the light/dark CSS
        asset, and atomic report writing (temporary siblings, replaced
        only after every page renders successfully, so a failure never
        leaves a partial report). Also writes `report_manifest.json`. Pure
        string/file assembly -- no bag or ROS access here.
"""

from __future__ import annotations

import argparse
import datetime
import html as html_module
import json
import shutil
from pathlib import Path
from typing import Optional, Sequence

import plotly.graph_objects as go
import plotly.offline as plotly_offline

from python_tools.data.models import ReportContext, ReportPage
from python_tools.reporting.style import CHROME_DARK, CHROME_LIGHT, FONT_FAMILY, plotly_config

# This tool's own version, written into report_manifest.json so a later
# comparison between two reports can tell whether the generator itself
# changed, not just the run.
GENERATOR_VERSION = "1.0.0"


class CrossRunOutputDirectoryError(Exception):
    """!
    @brief  Raised when `write_report()` is asked to write into an output
            directory that already holds tool-generated pages for a
            *different* run, per the plan's 2026-09-23 correction (Defect
            4's original fix correctly merged a standalone regeneration's
            manifest with an aggregate run's own prior pages, but never
            checked that both writes actually described the *same* run --
            reusing `--output-dir` across two different test runs silently
            produced a manifest whose `source_bag`/`topic_counts` named one
            run while `generated_pages` still listed another run's stale,
            un-regenerated HTML files). Every entry-point script's `main()`
            already catches broad `Exception` at its top-level CLI
            boundary and prints it as `error: {message}`, so this needs no
            additional handling at any call site.
    """

# Every page in the site, in nav-bar order. The first entry is always the
# index; entry filenames match what each post_processing_*.py script and
# post_processing.py itself write.
NAV_PAGES: tuple[tuple[str, str], ...] = (
    ("index.html", "Overview"),
    ("kalman_filter.html", "Kalman Filter"),
    ("visual_odometry.html", "Visual Odometry"),
    ("inertial_odometry.html", "Inertial Odometry"),
    ("wheel_odometry.html", "Wheel Odometry"),
)

# The one local Plotly asset every page shares, relative to a page's own
# directory (report pages and assets/ are siblings in the output dir).
PLOTLY_ASSET_RELATIVE_PATH = "assets/plotly.min.js"
CSS_ASSET_RELATIVE_PATH = "assets/report.css"


def escape(text: str) -> str:
    """!
    @brief   Escapes text for safe embedding in an HTML document.

    @param   text
             The raw text to escape.

    @return  The HTML-escaped text.
    """
    return html_module.escape(text, quote=True)


def figure_to_fragment(figure: go.Figure, element_id: str) -> str:
    """!
    @brief   Renders one Plotly figure as an embeddable HTML fragment,
             referencing the page's already-loaded local Plotly asset
             rather than bundling the library again per figure.

             The fragment's own `Plotly.newPlot(...)` call is a bare,
             unchained call (confirmed by inspecting `Figure.to_html()`'s
             output directly) -- its returned promise is not guaranteed to
             have settled by the time a later `<script>` element on the
             page runs, even though that later element executes after this
             one in document order (confirmed empirically: replaying this
             exact call against the real installed Plotly bundle shows its
             promise needs multiple microtask turns to settle, i.e. it is
             not resolved in the same synchronous pass). A `post_script`
             here (`plotly.io.to_html`'s own supported mechanism, chained
             via `.then()` after `Plotly.newPlot()`) is therefore the only
             point at which touching this figure's `graphDiv` -- such as
             the shared theme script's initial per-figure `Plotly.relayout`
             call in `_plotly_theme_script()` -- is safe. Calling
             `Plotly.relayout()` any earlier, before that promise resolves,
             was the root cause of figures rendering blank until a user
             interaction (which triggers a fresh, non-racing redraw) --
             see the plan document's Plotly rendering defect entry.

    @param   figure
             The figure to render.
    @param   element_id
             A page-unique HTML id for the figure's container `<div>`.

    @return  The figure's HTML fragment (a `<div>` plus its `<script>`).
    """
    return figure.to_html(
        full_html=False,
        include_plotlyjs=False,
        config=plotly_config(),
        div_id=element_id,
        post_script=(
            "if (window.__reportApplyPlotlyTheme) { "
            f"window.__reportApplyPlotlyTheme(document.getElementById({json.dumps(element_id)})); "
            "}"
        ),
    )


def render_nav(active_filename: str) -> str:
    """!
    @brief   Renders the persistent site navigation bar.

    @param   active_filename
             The current page's filename, used to mark the active link.

    @return  The nav bar's HTML fragment.
    """
    links = []
    for filename, label in NAV_PAGES:
        active_class = ' class="active"' if filename == active_filename else ""
        links.append(f'<a href="{filename}"{active_class}>{escape(label)}</a>')
    return '<nav class="site-nav" aria-label="Report pages">' + "".join(links) + "</nav>"


def render_warnings_section(warnings: Sequence[str]) -> str:
    """!
    @brief   Renders a page's warnings as a visible section, per the
             plan's "missing/disabled/stale data is visible" requirement.

    @param   warnings
             The warnings to display; an empty sequence renders nothing.

    @return  The warnings section's HTML fragment, or an empty string.
    """
    if not warnings:
        return ""
    items = "".join(f"<li>{escape(message)}</li>" for message in warnings)
    return f'<section class="warnings" aria-label="Warnings"><h2>Warnings</h2><ul>{items}</ul></section>'


def _plotly_theme_script() -> str:
    """!
    @brief   Builds the one shared `<script>` block that keeps every
             Plotly figure on the page readable in both light and dark
             mode (Defect 5 in the plan's 2026-09-22 audit: `style.
             plotly_layout` bakes CHROME_LIGHT's colors into every
             figure's JSON at generation time, so a figure's own text,
             gridlines and axis lines stayed light-mode-only even though
             `build_css`'s page chrome already responds to
             `prefers-color-scheme`). One implementation, not duplicated
             per figure or page.

             This only *defines* `window.__reportApplyPlotlyTheme` (themes
             one already-initialized graph div) and registers the live
             `prefers-color-scheme` listener that re-themes every figure
             already on the page; it deliberately does not sweep
             `.plotly-graph-div` itself on load any more (2026-09-23
             rendering-defect fix). `render_page()` places this script
             before any figure fragment, and each figure's own
             `figure_to_fragment()` post_script calls
             `window.__reportApplyPlotlyTheme` on just its own graph div,
             chained via Plotly's own `.then()` onto that specific
             figure's `Plotly.newPlot()` promise. A blanket
             `document.querySelectorAll(".plotly-graph-div")` sweep run
             eagerly at parse time -- the previous approach -- raced each
             figure's own still-settling `Plotly.newPlot()` promise
             (`Plotly.relayout()` was reachable before that figure's
             layout/axis-range/rangeslider initialization had actually
             finished), which is what previously left a figure rendered
             blank until a user interaction forced a fresh, non-racing
             redraw. The live theme-change listener below is unaffected by
             that race: by the time a user's OS theme changes, every
             figure's own initial `Plotly.newPlot()` has long since
             resolved.

    @return  The `<script>` block, self-contained (no external file).
    """
    # The same two palettes build_css() already uses for the page chrome,
    # re-exported to JavaScript so both stay driven by one source of
    # truth; "</" is escaped so this JSON can never prematurely close the
    # surrounding <script> tag even though these values are only ever
    # hex colors and a font stack today.
    theme_json = json.dumps(
        {
            "light": {
                "text_primary": CHROME_LIGHT["text_primary"],
                "gridline": CHROME_LIGHT["gridline"],
                "axis": CHROME_LIGHT["axis"],
            },
            "dark": {
                "text_primary": CHROME_DARK["text_primary"],
                "gridline": CHROME_DARK["gridline"],
                "axis": CHROME_DARK["axis"],
            },
        }
    ).replace("</", "<\\/")
    return f"""<script>
(function () {{
  var reportTheme = {theme_json};
  function themeUpdateFor(graphDiv) {{
    var isDark = window.matchMedia("(prefers-color-scheme: dark)").matches;
    var theme = isDark ? reportTheme.dark : reportTheme.light;
    var update = {{"font.color": theme.text_primary}};
    Object.keys(graphDiv.layout || {{}}).forEach(function (key) {{
      if (/^[xy]axis\\d*$/.test(key)) {{
        update[key + ".gridcolor"] = theme.gridline;
        update[key + ".linecolor"] = theme.axis;
        update[key + ".zerolinecolor"] = theme.axis;
      }}
    }});
    return update;
  }}
  window.__reportApplyPlotlyTheme = function (graphDiv) {{
    if (!graphDiv || !graphDiv.layout || typeof Plotly === "undefined") {{ return; }}
    Plotly.relayout(graphDiv, themeUpdateFor(graphDiv));
  }};
  function applyToEveryRenderedFigure() {{
    document.querySelectorAll(".plotly-graph-div").forEach(window.__reportApplyPlotlyTheme);
  }}
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", applyToEveryRenderedFigure);
}})();
</script>"""


def render_page(
    title: str,
    active_filename: str,
    body_html: str,
    warnings: Sequence[str],
) -> str:
    """!
    @brief   Renders one complete, self-contained report page.

    @param   title
             The page's title, used in `<title>` and the page header.
    @param   active_filename
             This page's own filename, for nav-bar highlighting.
    @param   body_html
             The page's already-rendered main content.
    @param   warnings
             Warnings to surface at the top of the page.

    @return  The complete HTML document as a string.
    """
    generated_at = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%d %H:%M:%S UTC"
    )
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escape(title)} - Lunar Simulator Post-Processing</title>
<link rel="stylesheet" href="{CSS_ASSET_RELATIVE_PATH}">
<script src="{PLOTLY_ASSET_RELATIVE_PATH}"></script>
{_plotly_theme_script()}
</head>
<body>
<header class="site-header">
  <h1>{escape(title)}</h1>
  {render_nav(active_filename)}
</header>
<main>
{render_warnings_section(warnings)}
{body_html}
</main>
<footer class="site-footer">
  <p>Generated by lunar_simulator post_processing {GENERATOR_VERSION} at {generated_at}</p>
</footer>
</body>
</html>
"""


def render_index_page(context: ReportContext, pages: Sequence[ReportPage]) -> ReportPage:
    """!
    @brief   Renders the index page: a run summary header, links to every
             subsystem page with its headline stats, topic coverage, the
             parameter snapshot location, and any run-level warnings.

    @param   context
             The report context the run was generated from.
    @param   pages
             The already-rendered subsystem pages, to summarize and link
             to.

    @return  The assembled index `ReportPage`.
    """
    run = context.run_metadata
    duration_s = (context.bag.end_time_ns - context.bag.start_time_ns) / 1e9
    cards = []
    for page in pages:
        stats_items = "".join(
            f"<dt>{escape(key)}</dt><dd>{escape(value)}</dd>"
            for key, value in page.summary_stats.items()
        )
        warning_badge = (
            f'<span class="badge badge-warning">{len(page.warnings)} warning(s)</span>'
            if page.warnings
            else '<span class="badge badge-good">OK</span>'
        )
        cards.append(
            f'<a class="summary-card" href="{escape(page.filename)}">'
            f"<h3>{escape(page.title)} {warning_badge}</h3>"
            f"<dl>{stats_items}</dl>"
            "</a>"
        )

    topic_rows = "".join(
        f"<tr><td>{escape(health.topic_name)}</td><td>{escape(health.message_type)}</td>"
        f"<td>{health.message_count}</td>"
        f"<td>{f'{health.effective_rate_hz:.2f}' if health.effective_rate_hz else 'n/a'}</td></tr>"
        for health in sorted(context.bag.topic_health.values(), key=lambda h: h.topic_name)
    )

    body = f"""
<section class="run-summary">
  <h2>Run summary</h2>
  <dl>
    <dt>World</dt><dd>{escape(run.world)}</dd>
    <dt>System</dt><dd>{escape(run.system)}</dd>
    <dt>Duration</dt><dd>{duration_s:.1f} s</dd>
    <dt>Recording profile</dt><dd>{escape(run.recording_profile or "unknown")}</dd>
    <dt>Storage</dt><dd>{escape(run.storage_identifier)}</dd>
    <dt>Source revision</dt><dd>{escape(run.source_revision or "unknown")}{" (dirty)" if run.dirty_worktree else ""}</dd>
    <dt>Test run directory</dt><dd>{escape(str(run.test_run_dir))}</dd>
    <dt>Parameter snapshot</dt><dd>{escape(str(run.test_run_dir / "parameters"))}</dd>
  </dl>
</section>
<section class="page-cards">
  <h2>Subsystem reports</h2>
  <div class="card-grid">{"".join(cards)}</div>
</section>
<section class="topic-health">
  <h2>Topic coverage</h2>
  <table>
    <thead><tr><th>Topic</th><th>Type</th><th>Messages</th><th>Rate (Hz)</th></tr></thead>
    <tbody>{topic_rows}</tbody>
  </table>
</section>
"""
    all_warnings = list(context.bag.warnings)
    return ReportPage(
        filename="index.html",
        title="Overview",
        html=render_page("Overview", "index.html", body, all_warnings),
        summary_stats={"Duration": f"{duration_s:.1f} s"},
        warnings=tuple(all_warnings),
    )


def build_css() -> str:
    """!
    @brief   Builds the shared `assets/report.css` stylesheet from
             python_tools.reporting.style's light/dark chrome definitions,
             so both modes stay driven by one source of truth.

    @return  The complete CSS document.
    """
    return f"""
:root {{
  color-scheme: light;
  --surface: {CHROME_LIGHT['surface']};
  --page: {CHROME_LIGHT['page']};
  --text-primary: {CHROME_LIGHT['text_primary']};
  --text-secondary: {CHROME_LIGHT['text_secondary']};
  --text-muted: {CHROME_LIGHT['text_muted']};
  --gridline: {CHROME_LIGHT['gridline']};
  --border: {CHROME_LIGHT['border']};
  --status-good: #0ca30c;
  --status-warning: #fab219;
  --status-serious: #ec835a;
  --status-critical: #d03b3b;
}}
@media (prefers-color-scheme: dark) {{
  :root {{
    color-scheme: dark;
    --surface: {CHROME_DARK['surface']};
    --page: {CHROME_DARK['page']};
    --text-primary: {CHROME_DARK['text_primary']};
    --text-secondary: {CHROME_DARK['text_secondary']};
    --text-muted: {CHROME_DARK['text_muted']};
    --gridline: {CHROME_DARK['gridline']};
    --border: {CHROME_DARK['border']};
  }}
}}
* {{ box-sizing: border-box; }}
body {{
  margin: 0;
  font-family: {FONT_FAMILY};
  background: var(--page);
  color: var(--text-primary);
}}
a {{ color: #2a78d6; }}
a:focus-visible, button:focus-visible {{ outline: 2px solid #2a78d6; outline-offset: 2px; }}
.site-header {{
  padding: 16px 24px;
  background: var(--surface);
  border-bottom: 1px solid var(--border);
}}
.site-header h1 {{ margin: 0 0 8px 0; font-size: 1.4rem; }}
.site-nav a {{
  display: inline-block;
  margin-right: 16px;
  padding: 6px 2px;
  text-decoration: none;
  color: var(--text-secondary);
  border-bottom: 2px solid transparent;
}}
.site-nav a.active {{ color: var(--text-primary); border-bottom-color: #2a78d6; font-weight: 600; }}
main {{ max-width: 1200px; margin: 0 auto; padding: 24px; }}
section {{ margin-bottom: 32px; }}
h2 {{ font-size: 1.1rem; border-bottom: 1px solid var(--gridline); padding-bottom: 6px; }}
dl {{ display: grid; grid-template-columns: max-content 1fr; gap: 4px 16px; margin: 0; }}
dt {{ color: var(--text-muted); }}
dd {{ margin: 0; }}
table {{ width: 100%; border-collapse: collapse; }}
th, td {{ text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--gridline); font-size: 0.9rem; }}
.card-grid {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 16px; }}
.summary-card {{
  display: block;
  padding: 16px;
  border: 1px solid var(--border);
  border-radius: 8px;
  background: var(--surface);
  text-decoration: none;
  color: inherit;
}}
.summary-card h3 {{ margin: 0 0 8px 0; font-size: 1rem; display: flex; justify-content: space-between; align-items: center; }}
.badge {{ font-size: 0.7rem; padding: 2px 8px; border-radius: 999px; color: white; }}
.badge-good {{ background: var(--status-good); }}
.badge-warning {{ background: var(--status-warning); color: #0b0b0b; }}
.empty-state-card {{
  border-left: 4px solid var(--status-warning);
  background: var(--surface);
  padding: 12px 16px;
  margin: 12px 0;
  border-radius: 4px;
}}
.warnings {{
  border-left: 4px solid var(--status-warning);
  background: var(--surface);
  padding: 12px 16px;
  border-radius: 4px;
}}
.warnings ul {{ margin: 8px 0 0 0; padding-left: 20px; }}
.plot-section {{ background: var(--surface); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 20px; }}
.site-footer {{ text-align: center; color: var(--text-muted); font-size: 0.8rem; padding: 24px; }}
.js-plotly-plot {{ width: 100%; }}
@media (max-width: 640px) {{
  main {{ padding: 12px; }}
  .site-header {{ padding: 12px; }}
}}
""".strip()


def write_report(
    pages: Sequence[ReportPage],
    output_dir: Path,
    context: ReportContext,
) -> None:
    """!
    @brief   Writes every page and shared asset atomically: renders to a
             temporary sibling directory first, and only replaces the real
             output directory's fixed page/asset filenames after every
             page rendered successfully, per the plan's "regeneration may
             overwrite only this tool's fixed filenames... do not leave a
             partial report" requirement. The shared `plotly.min.js` asset
             is written from the installed Plotly package's own bundled
             source (`plotly.offline.get_plotlyjs()`), guaranteeing it
             matches the exact library version every page's
             `Plotly.newPlot(...)` call was generated against.

    @param   pages
             Every rendered page to write (including the index).
    @param   output_dir
             The report's output directory (created if absent).
    @param   context
             The report context, used to write `report_manifest.json`.

    @return  None

    @throws  CrossRunOutputDirectoryError
             If `output_dir` already holds tool-generated pages for a
             *different* run (2026-09-23 correction); raised before
             anything on disk is created, moved, or deleted.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # Read any existing manifest and confirm it describes the same run
    # before touching anything else: a standalone script's regeneration of
    # one page must not make report_manifest.json claim fewer pages than
    # are actually still on disk (Defect 4, 2026-09-22 audit), and merging
    # it must never silently mix two different runs' pages into one report
    # (the same defect's own follow-up gap, fixed 2026-09-23). Consulted,
    # never written to directly.
    existing_manifest = _load_existing_manifest(output_dir)
    _ensure_same_run_or_raise(existing_manifest, context, output_dir)

    staging_dir = output_dir / ".post_processing_staging"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_assets_dir = staging_dir / "assets"
    staging_assets_dir.mkdir(parents=True)

    # Render every fixed output into the staging area first; nothing under
    # output_dir itself is touched until this all succeeds.
    for page in pages:
        (staging_dir / page.filename).write_text(page.html, encoding="utf-8")
    (staging_assets_dir / "report.css").write_text(build_css(), encoding="utf-8")
    (staging_assets_dir / "plotly.min.js").write_text(
        plotly_offline.get_plotlyjs(), encoding="utf-8"
    )
    manifest = _build_manifest(context, pages, output_dir, existing_manifest)
    (staging_dir / "report_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )

    # Every staged file rendered successfully: move each one into place,
    # replacing only this tool's own fixed filenames, then remove the
    # now-empty staging directory. Using individual file moves (rather
    # than replacing the whole output_dir) preserves any unrelated analyst
    # files already present, per the plan's "do not delete the output
    # directory or unrelated analyst files" requirement.
    for staged_path in staging_dir.rglob("*"):
        if staged_path.is_dir():
            continue
        relative_path = staged_path.relative_to(staging_dir)
        destination = output_dir / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(staged_path), str(destination))
    shutil.rmtree(staging_dir)


def _resolved_run_identity(context: ReportContext) -> dict:
    """!
    @brief   Builds the stable identity fields written into
             `report_manifest.json` and compared by `_ensure_same_run_or_
             raise` on every later write into the same output directory,
             so a report can prove which run it actually describes
             (2026-09-23 correction).

    @param   context
             The report context being written.

    @return  `{"test_run_dir": ..., "source_bag": ...}`, both resolved to
             absolute paths so a relative-vs-absolute spelling difference
             between two invocations for the *same* run (e.g. one run from
             the repository root, one from inside `test_runs/<run>`) is
             never mistaken for two different runs.
    """
    run = context.run_metadata
    return {
        "test_run_dir": str(run.test_run_dir.resolve()),
        "source_bag": str(run.bag_path.resolve()),
    }


def _ensure_same_run_or_raise(
    existing_manifest: Optional[dict], context: ReportContext, output_dir: Path
) -> None:
    """!
    @brief   Fails, before `write_report()` touches anything else, if
             `output_dir` already holds tool-generated pages for a
             *different* run than `context` describes (2026-09-23
             correction: Defect 4's original fix, 2026-09-22, correctly
             merged a standalone regeneration's manifest with an aggregate
             run's own prior pages, but never checked that both writes
             actually described the *same* run -- reusing `--output-dir`
             across two different test runs silently produced a manifest
             whose `source_bag`/`topic_counts` named one run while
             `generated_pages` still listed another run's stale,
             un-regenerated HTML files). Never deletes, modifies, or
             otherwise touches the existing report -- it only decides
             whether the caller may proceed.

    @param   existing_manifest
             The previous manifest at `output_dir`, or `None` if this is
             the first generation there.
    @param   context
             The report context about to be written.
    @param   output_dir
             The report's output directory.

    @return  None

    @throws  CrossRunOutputDirectoryError
             If `output_dir` contains at least one surviving tool-generated
             page from a previous manifest, and either that manifest lacks
             the identity fields needed to prove it belongs to the same
             run, or those fields name a different run. An existing
             manifest whose every previously-listed page has since been
             deleted from disk is not protected -- Defect 4's own
             manifest-merge logic already reconciles that case correctly
             and there is nothing left from another run to contaminate.
    """
    if not existing_manifest:
        return
    previously_generated_pages = existing_manifest.get("generated_pages", [])
    surviving_pages = [
        filename
        for filename in previously_generated_pages
        if isinstance(filename, str) and (output_dir / filename).is_file()
    ]
    if not surviving_pages:
        return

    new_identity = _resolved_run_identity(context)
    existing_identity = existing_manifest.get("run_identity")
    identity_matches = (
        isinstance(existing_identity, dict)
        and existing_identity.get("test_run_dir") == new_identity["test_run_dir"]
        and existing_identity.get("source_bag") == new_identity["source_bag"]
    )
    if identity_matches:
        return

    if not isinstance(existing_identity, dict):
        # An old or externally-written manifest without proven run
        # identity: never merge into it optimistically just because pages
        # happen to be present.
        reason = (
            "its manifest does not record which run it belongs to, so this "
            "tool cannot confirm the new report would describe the same run"
        )
    else:
        reason = (
            "it belongs to a different run "
            f"(test_run_dir={existing_identity.get('test_run_dir')!r}, "
            f"source_bag={existing_identity.get('source_bag')!r}) than the "
            f"one requested now (test_run_dir={new_identity['test_run_dir']!r}, "
            f"source_bag={new_identity['source_bag']!r})"
        )
    raise CrossRunOutputDirectoryError(
        f"{output_dir} already contains a generated report "
        f"({len(surviving_pages)} page(s): {', '.join(sorted(surviving_pages))}) "
        f"and {reason}. Refusing to mix two runs' pages into one report -- "
        "choose an empty or different --output-dir for this run; the "
        "existing report has not been modified."
    )


def _load_existing_manifest(output_dir: Path) -> Optional[dict]:
    """!
    @brief   Loads `report_manifest.json` from a previous generation into
             this same output directory, if one exists and parses.

    @param   output_dir
             The report's output directory.

    @return  The previous manifest as a dict, or `None` if this is the
             first generation into `output_dir` or the existing file is
             missing/unreadable/malformed (treated the same as absent --
             regeneration must not fail just because an old manifest is
             damaged).
    """
    manifest_path = output_dir / "report_manifest.json"
    if not manifest_path.is_file():
        return None
    try:
        return json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _build_manifest(
    context: ReportContext,
    pages: Sequence[ReportPage],
    output_dir: Path,
    existing_manifest: Optional[dict],
) -> dict:
    """!
    @brief   Builds the `report_manifest.json` contents, merged with any
             previous manifest so that a standalone script's regeneration
             of one page never makes the manifest claim fewer pages than
             are actually still on disk (Defect 4 in the plan's
             2026-09-22 audit: previously, a standalone run's manifest
             replaced `generated_pages`/`omissions` outright, going stale
             relative to the other pages a prior aggregate run had
             written). `topic_counts` is not merged: it is derived
             entirely from this run's own bag and is therefore identical
             regardless of which script computed it.

    @param   context
             The report context the run was generated from.
    @param   pages
             Every page rendered by *this* call (a standalone script
             passes just its own page; the aggregate script passes all
             five).
    @param   output_dir
             The report's output directory, used to confirm a
             previously-listed page file still actually exists before
             carrying its manifest entry forward.
    @param   existing_manifest
             The previous manifest at `output_dir`, or `None` if this is
             the first generation there.

    @return  The manifest as a plain JSON-serializable dict.
    """
    run = context.run_metadata
    regenerated_filenames = {page.filename for page in pages}

    # Carry forward a previously-listed page only if it was not just
    # regenerated (that entry is superseded below) and its file is still
    # actually present -- never claim a page exists that a user deleted.
    carried_forward_pages = [
        filename
        for filename in (existing_manifest or {}).get("generated_pages", [])
        if filename not in regenerated_filenames and (output_dir / filename).is_file()
    ]
    # Same rule for omissions: keep a previous page's own warnings only
    # while that page itself is still being carried forward.
    carried_forward_omissions = [
        omission
        for omission in (existing_manifest or {}).get("omissions", [])
        if omission.split(":", 1)[0] not in regenerated_filenames
        and omission.split(":", 1)[0] in carried_forward_pages
    ]

    return {
        "generator_version": GENERATOR_VERSION,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        # Stable identity this same function's own caller, write_report(),
        # checks on every subsequent write into the same output_dir before
        # merging anything -- see _ensure_same_run_or_raise (2026-09-23
        # correction).
        "run_identity": _resolved_run_identity(context),
        "source_bag": str(run.bag_path),
        "generated_pages": [
            *carried_forward_pages,
            *(page.filename for page in pages),
        ],
        "topic_counts": {
            name: health.message_count for name, health in context.bag.topic_health.items()
        },
        "maximum_alignment_gap_s": context.maximum_alignment_gap_s,
        "maximum_image_frames": context.maximum_image_frames,
        "omissions": [
            *carried_forward_omissions,
            *(
                f"{page.filename}: {warning}"
                for page in pages
                for warning in page.warnings
            ),
        ],
    }


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this module's
             standalone CSS-preview mode.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    parser = argparse.ArgumentParser(
        description="Print the shared report.css stylesheet to stdout, for manual inspection."
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Standalone CLI: prints the shared stylesheet.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  Process exit code; `0` on success.
    """
    parser = _build_arg_parser()
    parser.parse_args(argv)
    print(build_css())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
