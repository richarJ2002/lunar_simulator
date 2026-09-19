# Verification Report — `feature_tracking`

This report records what was actually run and what actually happened,
during the session that implemented `feature_tracking/` and integrated it
into `visual_odometry`. Every number below is a real result from this
project's own build/test/analysis tools, not an estimate or a
certification claim. Per `CPP_CODING_STANDARD.md` §15.4: **general tools
do not establish certification by themselves** — this report is evidence
for a project owner's own review, not a pass/fail compliance verdict.

## 1. Build

- `colcon build --packages-select lunar_simulator` — green at the end of
  every phase (scaffolding, `ShiTomasiCornerDetector`, `PyramidalLucasKanadeTracker`,
  `handleStereo.cc` integration, `.clang-tidy` triage), including the
  target-scoped `-Werror` on `alpha_feature_tracking` (DEV-FT-006).
- **Finding — build-type default (not originally planned, discovered
  during Phase 3 verification):** the repository's `CMakeCache.txt` had an
  empty `CMAKE_BUILD_TYPE` (no optimization flags at all) prior to this
  work. This was invisible before this change because every OpenCV call
  this code replaced runs inside OpenCV's own separately pre-optimized
  library, so the *calling* project's build type never affected OpenCV's
  own performance. `alpha_feature_tracking` is project-owned code compiled
  as part of this project, and fully inherits its build-type flags.
  Measured live (`scripts/launch_simulator.sh --headless`, an instrumented
  build with a temporary per-frame timer, since removed):
  - Unoptimized (`CMAKE_BUILD_TYPE` empty): `handleStereo()` regularly took
    **500–1000+ ms per frame** against a 100 ms (10 Hz) budget, and the
    fusing `continuous_ekf` logged `Dropping visual odometry N s older
    than filter time` on essentially every frame — the visual correction
    channel was, in effect, never being applied.
  - `RelWithDebInfo` (`-O2 -g -DNDEBUG`): `handleStereo()` regularly took
    **110–150 ms per frame**; a full 25-second/75-frame headless run
    produced **zero** dropped-odometry warnings and zero errors.
  - Fix: the root `CMakeLists.txt` now defaults `CMAKE_BUILD_TYPE` to
    `RelWithDebInfo` when the invoker doesn't set one, verified from a
    completely clean `build/`/`install/` tree with a bare `colcon build`
    (no extra flags) — the default reliably takes effect.
  - This is a real, load-bearing finding, not a cosmetic one: without it,
    the new engines are numerically correct (per §3 below) but functionally
    useless in the live pipeline, because the EKF discards every
    correction as stale.

## 2. Unit tests

`colcon test --packages-select lunar_simulator --event-handlers
console_direct+` / direct gtest binary runs, on the `RelWithDebInfo` build:

| Binary | Cases | Result |
|---|---|---|
| `test_kalman_math` | 11 | PASSED (pre-existing, unaffected by this work) |
| `test_corner_detector` | 14 | PASSED |
| `test_optical_flow_tracker` | 10 | PASSED |
| **Total** | **35** | **35/35 PASSED** |

Full binary paths (this repository nests test binaries under
`build/lunar_simulator/test/`, not directly under `build/lunar_simulator/`
— `CLAUDE.md`'s own build-commands section has been corrected to reflect
this):

```
./build/lunar_simulator/test/test_kalman_math
./build/lunar_simulator/test/test_corner_detector
./build/lunar_simulator/test/test_optical_flow_tracker
```

**A real bug was found and fixed by this test suite, not just confirmed
clean by it:** `PyramidalLucasKanadeTracker::initialize()` originally
accepted any `windowSizePx_in`/`maximumPyramidLevel_in` combination without
checking whether the *coarsest* pyramid level was actually large enough to
hold the window. With `maximumPyramidLevel=3` and `windowSizePx=21` on a
128×128 test image, the coarsest level was only 16×16 — smaller than the
window — so every feature was spuriously rejected as out-of-bounds
regardless of image content, and two tests (`IsDeterministicAcrossRepeatedCalls`,
`ConvergesWithinAConstrainedIterationBudget`) were passing *vacuously*
(both asserted only that the two calls agreed with each other, not that
tracking actually succeeded — both calls agreed on "lost" for the wrong
reason). Fixed by: (1) adding an explicit coarsest-level-size check to
`initialize()`, returning `FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION`
for an incompatible combination; (2) correcting every affected test's
image size/pyramid depth to a valid combination; (3) strengthening the two
vacuously-passing tests to assert the feature was actually found, not just
that repeated calls agreed with each other.

## 3. Static analysis (`clang-tidy`)

Tool: `clang-tidy` (LLVM 18.1.3), configuration: repo-root `.clang-tidy`
(created this session — see its own header comment for the enabled/
disabled check groups and rationale). **This is a real static-analysis
pass, not a certified MISRA C++ checker** (`CPP_CODING_STANDARD.md`
§15.4).

**Tooling note:** running all `VisualOdometryNode` `.cc` files through one
`clang-tidy` invocation crashed (a libclang/clang-tidy-18 parser crash
inside `RawComment` construction, reproducible only in a many-translation-
-unit single process, not on any individual file — a `clang-tidy` tool bug,
not a defect in this project's code, confirmed by re-running the exact
file that appeared to trigger it in isolation with a clean result). Worked
around by invoking `clang-tidy` once per source file instead of once per
directory.

### `feature_tracking/` (JSF-profile-scoped code)

Initial run: 110 findings across 10 categories. After triage (fix or
`// NOLINT` with justification, per file — see individual commits/diffs
for the exact changes):

| Category | Count | Disposition |
|---|---|---|
| `cppcoreguidelines-pro-bounds-constant-array-index` | 38 | Check disabled repo-wide (DEV-FT-008) |
| `misc-include-cleaner` | 39 → 20 remaining | 19 fixed (missing `<cstddef>`/`<vector>`/`<array>` includes); 20 remaining are this project's own `feature_tracking::` types, already reliably re-exported by each `.cc` file's own class object header — a deliberate, reviewed non-issue, not suppressed per-line |
| `modernize-use-auto` | 11 | Fixed (all 11) |
| `readability-identifier-length` | 6 | `// NOLINT` (standard `Ix`/`Iy`/`gx`/`gy` image-gradient notation, matching the reference algorithm) |
| `misc-non-private-member-variables-in-classes` | 4 | `// NOLINT` (`ImageView` is a deliberate plain aggregate, matching `Point2D`) |
| `readability-function-cognitive-complexity` | 3 | `// NOLINT` (each is one cohesive algorithmic step matching the reference algorithm's own structure — `buildPyramid`, `refineFeatureAtLevel`, `collectCandidates`) |
| `bugprone-misplaced-widening-cast` | 3 | `// NOLINT` (provably small-bounded `int + int` before the widening cast, e.g. a pyramid-level index or a ±2 kernel-tap offset) |
| `bugprone-easily-swappable-parameters` | 3 | `// NOLINT` (established, tested public API; each parameter independently range-validated) |
| `readability-make-member-function-const` | 2 | Fixed (`buildPyramid`, `refineFeatureAtLevel` marked `const`) |
| `cppcoreguidelines-pro-bounds-pointer-arithmetic` | 1 | `// NOLINT`, citing DEV-FT-003 |

**Final state: zero unexplained findings.** The only findings remaining
after triage are the 20 documented `misc-include-cleaner` won't-fix items
above.

### `visual_odometry/` (ROS/OpenCV integration layer — outside the JSF
profile scope, see `AGENTS.md`)

Initial run (per-file): 327 findings across 11 categories. After triage:

| Category | Count | Disposition |
|---|---|---|
| `clang-diagnostic-shorten-64-to-32` + `bugprone/cppcoreguidelines-narrowing-conversions` | 216 | **Won't-fix, documented.** All 216 are `declare_parameter<int>(...)` call sites in `VisualOdometryNode.h` — the exact same residual class the root `CMakeLists.txt` already documents and accepts for `-Wconversion` (the narrowing happens inside `rclcpp::Node::declare_parameter`'s own template body, not project-owned code; per `CPP_CODING_STANDARD.md` §15.3, project warning-as-error policy does not apply to third-party headers). This includes both pre-existing call sites and the new ones this work added (`image_width_px`, `optical_flow_maximum_pyramid_level`, etc.) — same pattern, same accepted rationale. |
| `misc-include-cleaner` | 51 → 46 remaining | 5 fixed (missing `<vector>`/`<cstddef>` in `publishFeatureImage.cc`/`publishPointCloud.cc`/`updatePose.cc`); remainder are ROS/OpenCV/own types already reliably re-exported by `VisualOdometryNode.h`'s own includes — same rationale as `feature_tracking/`'s won't-fix items |
| `readability-function-cognitive-complexity` | 25 (3 distinct findings, duplicated once per translation unit since the header is shared) | `// NOLINT` — `handleStereo()` (the documented per-frame pipeline orchestrator; inflated further by `RCLCPP_*_THROTTLE` macro expansion, not genuine nested logic) and `VisualOdometryNode`'s constructor/destructor (ordinary ROS node setup/teardown) |
| `modernize-avoid-bind` | 12 (1 distinct finding) | `// NOLINT` — attempted the lambda conversion; it broke `message_filters::Synchronizer::registerCallback`'s template-deduced overload resolution (a real compile error, reverted); `std::bind` kept deliberately |
| `cppcoreguidelines-special-member-functions` | 12 (1 distinct finding) | **Fixed** — `VisualOdometryNode` gained a non-default destructor this session (to terminate the two owned engines); added explicit `= delete` for all four copy/move special members, matching the `KalmanFilterNode` precedent |
| `readability-identifier-length` | 4 | 2 fixed (`xM`/`yM` renamed to `positionXM`/`positionYM`), 2 `// NOLINT` (`u`/`v`, standard pinhole pixel-coordinate notation) |
| `bugprone-easily-swappable-parameters` | 3 | `// NOLINT` (each is a private method with a single, reviewed call site) |
| `bugprone-misplaced-widening-cast` | 2 | `// NOLINT` (provably bounded `index * 6 + index`, `index ∈ [0,6)`, max 35) |
| `misc-const-correctness` | 1 | Fixed (`publishFeatureImage.cc`'s local `output` marked `const`) |
| `cppcoreguidelines-pro-type-vararg` | 1 | `// NOLINT` (`sensor_msgs::PointCloud2Modifier::setPointCloud2FieldsByString` is itself a C-style variadic ROS API; no non-vararg alternative exists) |

**Final state: only the two documented won't-fix categories remain**
(narrowing-conversions and the `feature_tracking`-type subset of
`misc-include-cleaner`), both cross-referenced to an existing, pre-dated
project policy rather than newly invented here.

## 4. Coverage

No numeric coverage percentage is claimed. See `CRITICALITY_RATIONALE.md`
for why (ECSS-Q-ST-80C §6.3.5.2 calls for a coverage *goal* agreed between
customer and supplier, which does not exist for this project) and
`TRACEABILITY_MATRIX.md` for the actual, named list of 31 tracked
requirements this session verified or honestly marked as a gap (30 of 31
directly test-verified).

## 5. Timing

**Indicative only — not a certified worst-case-execution-time (WCET)
analysis.** Both `CompletesDetectionWithinIndicativeWallClockBudget` and
`CompletesTrackingWithinIndicativeWallClockBudget` bound wall-clock time on
whatever machine runs the test, using `std::chrono::steady_clock`, at the
production-realistic 1024×1024/500-feature configuration:

| Test | Unoptimized build | `RelWithDebInfo` build |
|---|---|---|
| Corner detection (`detect()`, 1024×1024, 500 features) | ~81 ms | ~24 ms |
| Optical-flow tracking (`track()`, 1024×1024, 500 features) | ~533–579 ms | ~107–151 ms |
| Full `handleStereo()` per-frame cost, live (headless sim) | ~560–1000+ ms | ~110–150 ms |

A certified WCET analysis would require static timing analysis of the
compiled binary against the target hardware, accounting for cache effects,
branch prediction, and interrupt latency — none of which this project has
the tooling or the target-hardware access to perform. These numbers
establish that the engines are fit for the 10 Hz LocCam rate on the
development machine used, nothing stronger.

## 6. Known issues

- No sanitizer (ASan/UBSan/TSan) build variant exists for this target
  (`JSF_AV_APPLICABILITY_PROFILE.md` §3 gap).
- `DEV-FT-003`'s `ImageView::at()` unchecked-bounds risk depends on every
  future call site respecting an unenforced precondition (see the
  deviation log entry for the full risk statement).
- `DEV-FT-005`'s tracking-error metric has not been verified byte-identical
  against a live, differential OpenCV run (verified equivalent by formula/
  design only).
- `TRACEABILITY_MATRIX.md` REQ-FT-18 (coarsest-pyramid-level-vs-window-size
  rejection) has no dedicated rejection test, only indirect coverage
  through every other test's valid configuration.
- `TRACEABILITY_MATRIX.md` REQ-FT-29/REQ-FT-30 (ROS parameter wiring;
  constructor-failure-on-bad-config) are not gtest-covered, matching the
  pre-existing `ContinuousExtendedKalmanFilter`/`KalmanFilterNode`
  precedent's own gap in this repository.
- No independent (second-person) review has occurred. This report, the
  code, and the tests were produced in one AI-assisted working session
  under a single user's direction.

## 7. Sign-off

**Not signed off.** Per `JSF_AV_APPLICABILITY_PROFILE.md` §9, this whole
compliance package is proposed, not active, until a project owner reviews
it and resolves the outstanding `TODO(project)` fields across that
document and `DEVIATION_LOG.md`. This report is the factual record that
review would start from.

| Role | Name | Date |
|---|---|---|
| Prepared by | Claude (AI coding assistant), directed by the repository owner | 2026-09-17 |
| Reviewed by | `TODO(project)` | `TODO(project)` |
| Approved by | `TODO(project)` | `TODO(project)` |
