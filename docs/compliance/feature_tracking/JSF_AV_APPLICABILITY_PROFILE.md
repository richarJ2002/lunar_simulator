# JSF AV C++ Applicability Profile — `feature_tracking`

Completed per `~/.claude/skills/jsf-av-cpp/references/JSF_AV_CPP_PROFILE_TEMPLATE.md`.
This document records how this project applies its authoritative JSF AV C++
rule set to one scoped part of the repository. It is not the JSF standard
and does not replace a controlled copy of the applicable rules. It is also
not, by itself, a compliance certification — see `CRITICALITY_RATIONALE.md`
and this file's own Activation Declaration (§9).

## 1. Applicability

- Project: `lunar_simulator` (ROS 2 Jazzy + Gazebo lunar rover simulator).
- Profile owner: `TODO(project)` — no named individual/role has accepted
  ownership of this profile yet; see §9.
- Authoritative JSF AV rule source and revision: *Joint Strike Fighter Air
  Vehicle C++ Coding Standards for the System Development and Demonstration
  Program*, Document 2RDU00001, Revision C, December 2005, Lockheed Martin
  Corporation (public copy: <https://www.stroustrup.com/JSF-AV-rules.pdf>).
  Cited via `CPP_CODING_STANDARD.md` Appendix B.
- Effective date: 2026-09-17 (date this profile document and the code it
  governs were completed in this repository).
- Applicable directories/targets: exactly
  `src/localisation/visual_odometry/src/feature_tracking/**`, i.e. the
  `ShiTomasiCornerDetector` and `PyramidalLucasKanadeTracker` classes, their
  headers under `objects/`, their implementations under `methods/`, and the
  shared `FeatureTrackingLimits.h`/`FeatureTrackingStatus.h`/`Point2D.h`/
  `ImageView.h` headers. Built as the `alpha_feature_tracking` CMake STATIC
  library target.
- Excluded generated, third-party or test code:
  - `VisualOdometryNode.h` and every `src/localisation/visual_odometry/src/
    methods/VisualOdometryNode/*.cc` file (ROS/OpenCV integration layer;
    stays under the general robotics profile — see `AGENTS.md`).
  - `test/test_corner_detector.cpp` and `test/test_optical_flow_tracker.cpp`
    (test code; uses gtest macros, `std::vector`, and ordinary exceptions
    from gtest's own assertion machinery, none of which is JSF-profile
    code under test).
  - No generated or third-party code exists under the scoped path — the
    entire engine is project-owned with zero third-party dependencies
    (not even Eigen).
- Safety or assurance level, if applicable: **none assigned**. See
  `CRITICALITY_RATIONALE.md` — this is a documented, deliberate open item,
  not an oversight.

## 2. Precedence

Conflicts within the scoped path resolve in this order:

1. No certification, legal or contractual requirement applies to this
   project (a hobby/personal simulator) — this tier is empty.
2. The authoritative JSF AV C++ rule source named in §1.
3. Approved deviations recorded in `DEVIATION_LOG.md`.
4. `CPP_CODING_STANDARD.md` §3.3 (JSF AV C++ Profile) and its cross-referenced
   sections (§13.3 initialization failure, §15.3–15.5 warnings/analysis/
   casts).
5. Repository conventions (this repo's own `CLAUDE.md`, e.g. the
   "Comment conventions" section, and the one-class-per-header/
   one-method-per-`.cc` layout already used by `alpha_kalman_filter`).

## 3. Language and Toolchain

- C++ standard and permitted extensions: ISO C++17, no compiler
  extensions (`CMAKE_CXX_STANDARD 17`, `CMAKE_CXX_STANDARD_REQUIRED ON`,
  `CMAKE_CXX_EXTENSIONS OFF` in the root `CMakeLists.txt`). JSF AV Rev C
  predates C++17; see §7 deviation DEV-FT-001 for the language-version
  mapping this implies.
- Supported compiler(s) and version(s): whatever GCC/Clang ships with the
  targeted ROS 2 Jazzy toolchain (Ubuntu 24.04 at the time of writing);
  not pinned to a specific compiler build beyond that. No compiler-version
  matrix has been formally qualified.
- Warning configuration: repository-wide `-Wall -Wextra -Wpedantic
  -Wconversion -Wshadow` (root `CMakeLists.txt`), plus target-scoped
  `-Werror` on `alpha_feature_tracking` only — see DEV-FT-006.
- Static-analysis tools and versions: `clang-tidy` (LLVM 18.1.3 at the time
  of writing), repo-root `.clang-tidy`. Per `CPP_CODING_STANDARD.md` §15.4
  and the `.clang-tidy` file's own header comment: **this is a real
  static-analysis pass, not a certified MISRA C++ checker** — "general
  tools do not establish certification by themselves."
- Formatter configuration: repo-root `.clang-format` (LLVM-based, 4-space
  indent, Allman braces, 80 columns), applied repo-wide, not scoped
  specially to this path.
- Sanitizer or dynamic-analysis configuration: **none configured.** No
  AddressSanitizer/UndefinedBehaviorSanitizer/ThreadSanitizer build variant
  exists for this target. This is a gap against `CPP_CODING_STANDARD.md`
  §15.4's "where supported" recommendation, recorded honestly rather than
  omitted.

## 4. API and Failure Policy

- Status type and success/failure values: `feature_tracking::
  FeatureTrackingStatus` (`FeatureTrackingLimits.h`'s sibling header
  `FeatureTrackingStatus.h`), a plain `enum class` with
  `FEATURE_TRACKING_STATUS_SUCCESS`, `_NOT_INITIALIZED`,
  `_INVALID_CONFIGURATION`, `_INVALID_INPUT`. One shared enum for both
  engines (see §7 for why this diverges from the `alpha_kalman_filter`
  precedent's one-class-one-enum).
- Scope of the status-return requirement: every fallible method
  (`initialize()`, `detect()`, `track()`, `terminate()`) returns
  `[[nodiscard]] FeatureTrackingStatus`. Every call site in this repository
  either checks the returned status or explicitly discards it with
  `static_cast<void>(...)` at a documented "not fatal to this frame"
  boundary (`handleStereo.cc`) — never silently ignored.
- Getter and predicate policy: neither engine currently exposes an
  infallible getter/predicate beyond the compile-time `static constexpr`
  capacity constants (`MAXIMUM_SUPPORTED_FEATURES`, etc.), which return by
  value directly per `CPP_CODING_STANDARD.md` §3.3's "infallible accessors
  ... may return values directly."
- Constructor failure policy: both engines' constructors are
  `noexcept = default` and establish only a valid, uninitialized state —
  they cannot fail. Fallible setup is entirely in `initialize()`.
- Initialization method and state machine: two-phase initialization per
  `CPP_CODING_STANDARD.md` §13.3 — construct (non-failing) → `initialize()`
  (the one fallible, allocating step; may be re-called to reconfigure,
  discarding prior state) → any number of `detect()`/`track()` calls →
  `terminate()` (releases storage, returns to uninitialized). Every method
  other than the constructor and `initialize()` itself rejects a call
  before a successful `initialize()` with `FEATURE_TRACKING_STATUS_
  NOT_INITIALIZED`.
- Exception policy: **no exceptions** are thrown, caught, or propagate
  through any function under the scoped path. Every function is `noexcept`
  (constructors, `initialize()`, `detect()`/`track()`, `terminate()`, and
  every private `methods/` helper). Standard-library calls used
  (`std::array`, `std::vector::assign` inside `initialize()` only,
  `std::sort`, `std::sqrt`, `std::abs`) do not throw under the
  preconditions this code establishes (bounded sizes, finite inputs
  validated before use).
- Assertion policy: no runtime assertions (`assert()`/`abort()`-based) are
  used. Every precondition is either validated and turned into a returned
  `FeatureTrackingStatus` (at `initialize()`/`detect()`/`track()`
  boundaries) or left as a documented, unchecked caller responsibility for
  hot-loop performance (`ImageView::at()`, DEV-FT-003).
- Logging and diagnostic policy: **none** — neither engine logs anything
  (no `rclcpp` dependency exists under the scoped path at all). Logging on
  a failure status is the caller's (`VisualOdometryNode`'s) responsibility,
  outside this profile's scope.

## 5. Object and Resource Policy

- Dynamic-allocation restrictions: exactly one allocation site per engine,
  inside `initialize()` (`std::vector::assign` sizing each pyramid level's
  buffers in `PyramidalLucasKanadeTracker`; the fixed-size `std::array`
  members of `ShiTomasiCornerDetector` need no dynamic allocation at all —
  see DEV-FT-002). No allocation occurs in `detect()`/`track()`,
  verified by manual review (§8/`VERIFICATION_REPORT.md`) — grep for
  `new`/`make_unique`/container-growth calls outside `initialize()`/
  `terminate()` across every `.cc` file under the scoped path.
- Ownership policy: every buffer is owned by the engine instance itself
  (`std::array` members, or `std::vector` members sized once). `ImageView`
  is a non-owning, borrowed view constructed by the caller at the ROS/
  OpenCV boundary (outside this profile's scope) and must outlive the call
  that takes it — documented on the struct itself.
- Copy and move policy: both engine classes are explicitly non-copyable
  and non-movable (`= delete` on all four special member functions),
  matching the `alpha_kalman_filter` precedent — an engine's fixed,
  `initialize()`-sized internal storage has no well-defined copy/move
  semantics worth supporting.
- Inheritance policy: neither engine class is polymorphic; neither is
  intended as a base class. No virtual functions exist under the scoped
  path.
- Polymorphic destruction policy: not applicable (§ above — no
  polymorphism under the scoped path).
- Initialization-order requirements: within `initialize()`, validation of
  every parameter happens before any state is committed or any buffer is
  sized/allocated — a validation failure leaves the object's prior state
  (uninitialized, or the previous successful configuration) untouched.
- Threading and synchronization restrictions: neither engine is
  thread-safe and neither documents any synchronization. Both are
  documented as intended for a single-threaded caller, matching
  `VisualOdometryNode`'s own documented single-threaded-executor
  assumption (see its class-level Doxygen).

## 6. Numeric and Robotics Policy

- Permitted numeric types and conversions: `float` for per-pixel image
  data and geometry (`Point2D`, gradients, structure-tensor terms),
  `double` only where the caller's own units are already `double` (none,
  under the scoped path — the engines are `float`-only), `int`/`std::size_t`
  for indices and counts with explicit `static_cast` at every boundary
  crossing (documented per-site where clang-tidy's narrowing-conversion
  checks required a justified `// NOLINT`, e.g. small provably-bounded
  `int + int` expressions before a widening cast to `std::size_t`).
- Floating-point comparison policy: no `==`/`!=` between computed
  floating-point values. Thresholds use strict `>`/`<` against a
  configured tolerance (e.g. `qualityLevel_in × globalMaxResponse`,
  `epsilonPx_` convergence, `minimumEigenvalueThreshold_`); the one
  exact-equality check (`globalMaxResponse == 0.0F`) is against a literal
  zero representing "no gradient energy anywhere in the frame," not a
  computed value compared to another computed value.
- Overflow and range-checking policy: every configuration parameter
  (`width_in`, `height_in`, `maximumFeatures_in`, `maximumPyramidLevel_in`,
  `windowSizePx_in`, etc.) is range-checked against a compile-time ceiling
  in `FeatureTrackingLimits.h` before use. Per-pixel/per-iteration index
  arithmetic is either a small provably-bounded expression (documented at
  each `// NOLINT(bugprone-misplaced-widening-cast)` site) or itself
  derived from an already-validated bound.
- Frame and transformation conventions: **not applicable** — neither
  engine has any concept of a coordinate frame beyond a single image's
  own pixel-coordinate system (`Point2D{x, y}` in pixels). Frame handling
  (optical/body/map) is entirely `VisualOdometryNode`'s responsibility,
  outside this profile's scope.
- Unit conventions: pixels for every `Point2D`/`ImageView` quantity;
  dimensionless for `qualityLevel_in` (a fraction of peak response) and
  `minimumEigenvalueThreshold_in`/`epsilonPx_` (matching OpenCV's own
  parameter units so existing tuning carries over unchanged).
- Time and timestamp conventions: **not applicable** — neither engine
  reads, stores, or reasons about time; `track()`/`detect()` are called
  once per stereo frame by `VisualOdometryNode`, which owns all timing.

## 7. Deviations

See `DEVIATION_LOG.md` for the full, individually-justified list
(DEV-FT-001 through DEV-FT-008 at the time of writing). Summary of the
categories recorded there:

- JSF Rev C (C++03-era) to C++17 language/library mapping.
- The one `initialize()`-time heap allocation.
- `ImageView::at()`'s unchecked bounds precondition.
- The pyramidal tracker's no-border-extrapolation policy.
- The tracking-error metric being a documented equivalent of OpenCV's own
  metric, not a byte-exact reproduction.
- `-Werror` scoped to the `alpha_feature_tracking` CMake target only, not
  the whole repository.
- The corner detector's minimum-distance selection using a simple bounded
  all-pairs check instead of the spatial-grid-bucketing optimization this
  project's own design plan originally sketched.
- `cppcoreguidelines-pro-bounds-constant-array-index` disabled repository-
  wide in `.clang-tidy`, rather than suppressed at each of its ~38
  individual sites.

No deviation is approved merely because it appears in a source comment,
this profile, or an agent's response — per the `jsf-av-cpp` skill's own
"Claims and Evidence" section, each entry in `DEVIATION_LOG.md` still
needs its `Approving authority` / `Approval date` fields resolved by the
project owner (currently `TODO(project)`).

## 8. Verification and Evidence

- Required build variants: at minimum a `RelWithDebInfo`-or-better
  optimized build — see the root `CMakeLists.txt`'s `CMAKE_BUILD_TYPE`
  default and its documented rationale (an unoptimized build cannot keep
  pace with the 10 Hz LocCam rate; this is load-bearing for correct
  operation, not just a nicety). No separate sanitizer build variant
  exists (§3 gap, noted honestly).
- Required unit, integration and system tests: `test_corner_detector.cpp`
  (14 cases) and `test_optical_flow_tracker.cpp` (10 cases), both linking
  directly against `alpha_feature_tracking` with no ROS node involved
  (mirroring `test_kalman_math.cpp`'s precedent) — see
  `TRACEABILITY_MATRIX.md` for the requirement-to-test mapping. System-level
  verification is the one manual `scripts/launch_simulator.sh --headless`
  smoke run recorded in `VERIFICATION_REPORT.md`, not an automated system
  test.
- Required coverage: **no numeric coverage target is mandated.** Per the
  corrected reading of ECSS-Q-ST-80C in `CRITICALITY_RATIONALE.md`,
  "100% code branch coverage" is one optional measure in §6.2.3.2's menu
  for critical software, not a blanket requirement, and §6.3.5.2 instead
  calls for coverage *goals* agreed between customer and supplier — this
  project has no customer, so no goal has been agreed. Every documented
  behavior (success paths, every `INVALID_CONFIGURATION`/`INVALID_INPUT`
  rejection, the low-texture and out-of-bounds "lost" paths, determinism,
  the maximum-feature-batch boundary) is exercised by name in
  `TRACEABILITY_MATRIX.md`, which is offered as the real evidence in place
  of an invented percentage.
- Required analysis reports: one `clang-tidy` report per source file under
  the scoped path (see `VERIFICATION_REPORT.md` for the summarized,
  triaged results and `.clang-tidy` for the enabled/disabled check
  groups).
- Traceability location: `TRACEABILITY_MATRIX.md`.
- Evidence retention location and duration: this repository's own git
  history and the `docs/compliance/feature_tracking/` directory; no
  external retention system exists, and no retention duration has been
  set (`TODO(project)`).
- Review and approval workflow: **none formally established.** This
  profile, the code, and the tests were produced by an AI coding
  assistant (Claude) under a single user's direction in one working
  session, without an independent reviewer. This is stated plainly rather
  than implied away — see §9.

## 9. Activation Declaration

Per the `jsf-av-cpp` skill's own instruction ("do not silently complete
compliance decisions on the project's behalf"), this section is left
**explicitly pending human sign-off** rather than marked active by the
agent that wrote it.

- Proposed scope: `src/localisation/visual_odometry/src/feature_tracking/**`
  (see §1).
- Proposed rule revision: JSF AV Doc 2RDU00001 Rev C (see §1), with the
  C++17 mapping and deviations in §7/`DEVIATION_LOG.md`.
- Proposed authority: `TODO(project)` — the repository owner (or whoever
  they delegate) must review §1–§8 above, resolve every remaining
  `TODO(project)` field (profile owner, approving authority/dates in
  `DEVIATION_LOG.md`, evidence retention duration, review workflow), and
  replace this paragraph with an explicit acceptance before this profile
  is treated as authoritatively active rather than proposed.

**Current status: PROPOSED — PENDING PROJECT OWNER SIGN-OFF.** Until that
sign-off happens, this document records what a completed profile for this
code *would* say, produced in good faith from the real code and real test
results, not a template placeholder — but it is not yet an approved
project decision.
