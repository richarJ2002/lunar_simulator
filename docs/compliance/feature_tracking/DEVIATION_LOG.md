# Deviation Log — `feature_tracking`

Every deviation from the raw JSF AV C++ rule set (or from this project's own
`.clang-tidy` static-analysis baseline) that `feature_tracking/` code takes,
recorded per `JSF_AV_CPP_PROFILE_TEMPLATE.md` §7's required fields.

**No deviation here is "approved" merely because it is written down.** Every
entry's `Approving authority` / `Approval date` is `TODO(project)` — per the
`jsf-av-cpp` skill's own instruction, an agent does not grant itself
approval authority over its own deviations. These entries record the
technical facts (what deviates, why, what risk it carries, what verification
compensates for it) so the project owner has a real basis to approve, reject,
or amend each one.

---

### DEV-FT-001 — JSF AV Rev C → C++17 mapping

- **Affected rule:** The entire JSF AV Doc 2RDU00001 Rev C rule set, which
  targets C++03.
- **Affected scope:** All of `feature_tracking/`.
- **Technical rationale:** JSF AV Rev C (December 2005) predates C++11/14/17
  entirely — it has no opinion on `enum class`, `[[nodiscard]]`,
  `constexpr`, `std::array`, uniform initialization, or `= delete`d special
  member functions, because none of them existed. This project uses all of
  them: `enum class FeatureTrackingStatus` (type-safe status codes, strictly
  stronger than Rev C's own C-style-enum-era guidance), `[[nodiscard]]`
  (compiler-enforced "every returned error is tested," strictly stronger
  than Rev C's manual-review-only equivalent, AV Rule 115), `constexpr`
  compile-time capacity constants (`FeatureTrackingLimits.h`), `std::array`
  fixed-capacity buffers (bounded, stack/member storage, no heap), and
  `= delete`d copy/move (explicit, compiler-enforced non-copyable/
  non-movable, matching the `alpha_kalman_filter` precedent). Per
  `CPP_CODING_STANDARD.md` §3.3: "applying \[Rev C] to C++17 requires a
  documented mapping for newer language and library features" — this entry
  is that mapping.
- **Introduced risk:** None identified where used here — every C++17
  feature listed above is used to make a rule's *intent* (bounded memory,
  tested status returns, non-copyable ownership) more strongly enforced by
  the compiler than Rev C's own C++03-era mechanisms could, not to weaken
  it.
- **Compensating verification:** `-Wall -Wextra -Wpedantic -Wconversion
  -Wshadow` plus target-scoped `-Werror` (DEV-FT-006) catch a
  `[[nodiscard]]` value silently discarded; `clang-tidy`'s `modernize-*`
  and `cppcoreguidelines-*` groups are enabled repository-wide (see
  `.clang-tidy`) to flag any C++17 idiom used incorrectly.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — review if this
  project ever adopts a JSF AV revision published after C++17 support was
  added, since that revision may resolve this mapping itself.

---

### DEV-FT-002 — One-time heap allocation in `initialize()`

- **Affected rule:** AV Rules 206/208-adjacent guidance and
  `CPP_CODING_STANDARD.md` §3.3: "Allocation and deallocation from the
  heap shall not occur after the defined initialization phase."
- **Affected scope:** `PyramidalLucasKanadeTracker::initialize()`
  (`std::vector::assign` sizing each pyramid level's `intensity`/
  `gradientX`/`gradientY` buffers). `ShiTomasiCornerDetector` needs no
  dynamic allocation at all — its buffers are fixed-size `std::array`
  members sized by compile-time constants, not by `initialize()`'s
  runtime resolution arguments.
- **Technical rationale:** Pyramid level buffer sizes depend on the
  runtime `width_in`/`height_in`/`maximumPyramidLevel_in` arguments to
  `initialize()`, which are not known at compile time (they come from ROS
  parameters). A fixed-size `std::array` large enough for the compile-time
  ceiling (`MAXIMUM_SUPPORTED_WIDTH_PX × MAXIMUM_SUPPORTED_HEIGHT_PX`,
  4096×4096) would be ~67 MB *per pyramid level, per pyramid, per buffer*
  — clearly impractical as a fixed member. `CPP_CODING_STANDARD.md` §3.3
  itself frames the rule as "no heap alloc **after** the defined
  initialization phase," and `initialize()` **is** that defined phase (see
  §13.3's two-phase-initialization pattern) — this is the intended use of
  the exception, not a bypass of it.
- **Introduced risk:** `initialize()` can, in principle, fail with
  `std::bad_alloc` inside `std::vector::assign` if the system is out of
  memory. This is not caught (the profile is exception-free) and would
  terminate the process. Given `MAXIMUM_SUPPORTED_WIDTH_PX/HEIGHT_PX` cap
  the maximum possible allocation to a bounded, known size (well under what
  a modern machine running Gazebo already requires), this is judged low
  risk in practice, but it is a real, undischarged gap against a strict
  no-throw guarantee for `initialize()` itself.
- **Compensating verification:** `initialize()`'s own parameter validation
  runs *before* any allocation, rejecting an out-of-range resolution with
  `FEATURE_TRACKING_STATUS_INVALID_CONFIGURATION` rather than attempting
  an unbounded allocation. `terminate()` explicitly `.clear()`s and
  `.shrink_to_fit()`s every buffer, verified by manual review that no
  `new`/`make_unique`/container-growth call exists in `detect()`/`track()`
  (see `TRACEABILITY_MATRIX.md` REQ-FT-11 and `VERIFICATION_REPORT.md`).
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)`

---

### DEV-FT-003 — `ImageView::at()` unchecked bounds precondition

- **Affected rule:** General bounds-safety guidance (`CPP_CODING_STANDARD.md`
  §15.5's caution on unchecked conversions/indexing, and the spirit of
  `cppcoreguidelines-pro-bounds-pointer-arithmetic`, which this project's
  `.clang-tidy` deliberately leaves **enabled** rather than
  blanket-disabling — see `.clang-tidy`'s own header comment).
- **Affected scope:** `ImageView::at(int row_in, int column_in)` — one
  function, in `feature_tracking/objects/ImageView.h`.
- **Technical rationale:** `at()` is called once per sampled pixel inside
  every hot loop in both engines (Sobel gradients, structure-tensor
  accumulation, bilinear sampling's four corner reads). Every call site is
  structurally bounded already — loop limits derived from `width_`/
  `height_`, or a position pre-validated by the caller before the loop
  runs — so a bounds check inside `at()` itself would be pure, provably
  redundant overhead repeated millions of times per frame.
- **Introduced risk:** If a future change introduces a call site whose
  index is *not* structurally bounded, `at()` will read out of bounds
  (undefined behavior) rather than fail safely with a status code. This is
  the single highest-risk deviation in this log precisely because its
  safety depends on every *future* call site respecting an unenforced
  precondition, not on anything `at()` itself can check.
- **Compensating verification:** Every current call site is manually
  reviewed and documented as structurally bounded (see the class-level
  Doxygen comments in `ShiTomasiCornerDetector.h`/
  `PyramidalLucasKanadeTracker.h` referencing this entry). The one
  `cppcoreguidelines-pro-bounds-pointer-arithmetic` finding this produces
  is suppressed per-site with a `// NOLINT` citing this exact entry (see
  `ImageView.h`), not silenced by disabling the check project-wide —
  keeping the checker live to catch a *future*, less-careful call site.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — re-review if
  `ImageView::at()` ever gains a new call site outside the two existing
  engine classes.

---

### DEV-FT-004 — No border extrapolation (stricter-than-OpenCV boundary policy)

- **Affected rule:** Not a JSF rule directly — a deliberate behavioral
  divergence from the OpenCV functions this code replaces
  (`cv::goodFeaturesToTrack`/`cv::calcOpticalFlowPyrLK`), recorded here
  because it changes observable tracking behavior at image borders.
- **Affected scope:** `PyramidalLucasKanadeTracker::sampleBilinear()` (the
  no-extrapolation rejection) and, downstream, `refineFeatureAtLevel()`
  (which marks a feature *lost* rather than extrapolated).
- **Technical rationale:** OpenCV's `calcOpticalFlowPyrLK` internally
  reflects/replicates pixels at the image border (`BORDER_REFLECT_101`-
  style) so a window near the edge still produces a value. This project's
  tracker instead rejects any sample whose 2×2 bilinear footprint would
  touch a pixel outside `[0, width) x [0, height)`, marking that feature
  lost at that pyramid level. This is simpler to reason about, avoids an
  unproven border-extrapolation implementation (border reflection has its
  own edge cases — a window wider than the image, a pyramid level thinner
  than the window), and is deterministic by construction (no fabricated
  pixel values feeding into the Gauss-Newton solve).
- **Introduced risk:** Features genuinely near the image border are lost
  more readily than the equivalent OpenCV call would lose them, reducing
  the number of usable correspondences for PnP RANSAC in border regions.
  Given typical LocCam frames have corner-rich texture away from the
  border and `minimum_correspondences` already gates on overall count,
  this is judged a acceptable trade for determinism, not a correctness
  bug.
- **Compensating verification:** `PyramidalLucasKanadeTrackerTest.
  MarksOutOfBoundsFeatureAsLost` (test_optical_flow_tracker.cpp) exercises
  this exact policy directly.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — revisit if
  real-world tracking near frame borders is ever found insufficient during
  actual simulator use.

---

### DEV-FT-005 — Tracking-error metric: documented equivalent, not byte-exact

- **Affected rule:** Not a JSF rule — a behavioral-equivalence claim about
  the OpenCV function being replaced.
- **Affected scope:** `PyramidalLucasKanadeTracker::refineFeatureAtLevel()`'s
  final residual computation (the value written to `trackingError_out` /
  `error_out`), and its consumer, `VisualOdometryNode::
  MAXIMUM_TRACKING_ERROR_PX`.
- **Technical rationale:** This engine reports the mean absolute intensity
  residual between the fixed template window and the converged current-
  frame window, matching OpenCV's own documented default `err` output for
  `calcOpticalFlowPyrLK` (mean absolute difference over the window when
  `OPTFLOW_LK_GET_MIN_EIGENVALS` is not requested) *by formula* — the same
  computation, reasoned through against OpenCV's public documentation and
  this project's own implementation. It has **not** been verified
  byte-identical against a live, side-by-side OpenCV run on the same input
  images: doing so would mean reintroducing an OpenCV dependency solely
  for verification, which this plan deliberately keeps out of the
  production build (see `VERIFICATION_REPORT.md`'s "Known Issues" section).
- **Introduced risk:** If the two implementations diverge in some detail
  neither this project's design review nor OpenCV's documentation
  surfaced (e.g. a subtly different final-residual sample point, or a
  different window normalization), `MAXIMUM_TRACKING_ERROR_PX = 30.0F`
  (tuned historically against OpenCV's metric) could accept or reject
  matches slightly differently than before. This was explicitly
  re-examined during Phase 3 integration (not carried over unchecked): the
  reasoning above is the re-examination, and no change to the threshold
  was found necessary.
- **Compensating verification:**
  `PyramidalLucasKanadeTrackerTest.ProducesLargerErrorForAMismatchedThanAnAccurateTrack`
  verifies the metric responds correctly in *relative* terms (a genuine
  match scores lower than a deliberately mismatched one), which is what
  `MAXIMUM_TRACKING_ERROR_PX` actually depends on. A live differential
  test against OpenCV remains a documented gap, not silently closed.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — close by adding
  the differential OpenCV-comparison test the plan describes as optional,
  if this project ever wants byte-level confidence rather than
  formula-level confidence.

---

### DEV-FT-006 — `-Werror` scoped to one CMake target only

- **Affected rule:** `CPP_CODING_STANDARD.md` §15.3: "Continuous
  integration should treat project warnings as errors after the baseline
  is clean" (repository-wide framing).
- **Affected scope:** The whole repository's build, versus the single
  `alpha_feature_tracking` target.
- **Technical rationale:** The root `CMakeLists.txt` documents its own
  reason for *not* enabling `-Werror` repository-wide: `-Wconversion`
  reports a `long int → int` narrowing at every `declare_parameter<int>(...)`
  call site across every ROS node in this repository, a warning that
  originates inside `rclcpp::Node::declare_parameter`'s own template body
  (third-party header code), not in any project-owned expression — and
  §15.3 itself says not to apply project warning-as-error flags to
  third-party headers. `alpha_feature_tracking` has **zero** ROS/OpenCV
  header exposure (it is dependency-free even of Eigen), so this
  particular residual-warning class cannot occur there, making it safe to
  hold to the stricter `-Werror` bar the rest of the repository cannot yet
  meet.
- **Introduced risk:** None to `feature_tracking/` itself (it is the
  *stricter*-held target). The risk, if any, is that the rest of the
  repository's warning baseline could silently regress without `-Werror`
  catching it at build time — an existing, pre-dated condition this
  deviation does not create or worsen.
- **Compensating verification:** `alpha_feature_tracking` builds clean
  under `-Werror` (verified every phase of this work — see
  `VERIFICATION_REPORT.md`'s build log summary). The rest of the
  repository's warning baseline is still checked (just not treated as
  fatal) via the repository-wide `-Wall -Wextra -Wpedantic -Wconversion
  -Wshadow`.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — revisit if this
  project ever migrates `declare_parameter` call sites to a wrapper that
  resolves the `-Wconversion` finding, at which point `-Werror` could be
  raised repository-wide and this deviation would close.

---

### DEV-FT-007 — Minimum-distance selection: bounded all-pairs check, not spatial-grid bucketing

- **Affected rule:** Not a JSF rule — a deviation from this project's own
  design plan (`can-you-make-a-polished-axolotl.md` §1), which originally
  sketched a spatial-grid-bucketing optimization for
  `ShiTomasiCornerDetector::selectByMinimumDistance()`.
- **Affected scope:** `ShiTomasiCornerDetector::selectByMinimumDistance()`.
- **Technical rationale:** The plan's grid-bucketing sketch exists to avoid
  an `O(candidateCount × acceptedCount)` all-pairs distance check scaling
  with an *unbounded* `O(width·height)` candidate list. But
  `collectCandidates()` (the stage immediately before selection) already
  bounds the candidate pool to a fixed `MAXIMUM_CANDIDATE_POOL =
  MAXIMUM_SUPPORTED_FEATURES × 8` (8192 at the current
  `MAXIMUM_SUPPORTED_FEATURES = 1024`) via a min-heap, independent of image
  content. Against that already-bounded pool, the worst case for a simple
  all-pairs check is `8192 × 1024 ≈ 8.4M` comparisons — small, fixed, and
  simple enough to review and test directly. The grid-bucketing
  optimization would add real implementation complexity (cell-size
  tuning, neighbor-cell iteration, an edge case at cell boundaries) for a
  benefit that does not materialize once the input is already bounded.
  This was a design judgment made *during* implementation, not
  pre-approved in the plan.
- **Introduced risk:** Slower worst-case selection than the plan's
  original sketch, in a way that could matter if `MAXIMUM_SUPPORTED_
  FEATURES` were raised substantially in the future (the cost is
  quadratic in the pool size, not linear). At the current constant this
  is measured well within the indicative wall-clock budget (see
  `ShiTomasiCornerDetectorTest.CompletesDetectionWithinIndicativeWallClockBudget`).
- **Compensating verification:** The same wall-clock-budget test above
  covers the full `detect()` call, including this selection step, at the
  production-realistic `1024×1024`/500-feature configuration.
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — revisit if
  `MAXIMUM_SUPPORTED_FEATURES` (and therefore `MAXIMUM_CANDIDATE_POOL`) is
  ever raised enough that the quadratic cost becomes measurable against
  the frame budget.

---

### DEV-FT-008 — `cppcoreguidelines-pro-bounds-constant-array-index` disabled repository-wide

- **Affected rule:** Not a JSF rule — a `.clang-tidy` static-analysis
  configuration decision, recorded here because of its scale and because
  it deliberately differs from how DEV-FT-003's sibling check
  (`cppcoreguidelines-pro-bounds-pointer-arithmetic`) is treated.
- **Affected scope:** Every `std::array` index expression across
  `feature_tracking/` — this check fired ~38 times, on the same
  structural pattern each time: a runtime index into a fixed-size
  `std::array` already bounded by an `initialize()`-time-validated range
  or a fixed loop limit (a pyramid level index, a window offset, a
  candidate-pool index).
- **Technical rationale:** Switching every one of those hot-loop accesses
  (per-pixel Sobel/structure-tensor computation, per-iteration Gauss-
  Newton window sampling) to `.at()` would add a bounds check on every
  pixel and every iteration, for no safety benefit the surrounding loop
  structure does not already provide, and would measurably erode the
  real-time margin this project spent real effort restoring (see
  `VERIFICATION_REPORT.md`'s `CMAKE_BUILD_TYPE` finding). Unlike
  `ImageView::at()`'s pointer arithmetic (DEV-FT-003 — one call site,
  whose safety depends on every *future* caller respecting an unenforced
  precondition), each of these 38 sites is already provably safe by
  construction *today*, and scattering ~38 near-identical `// NOLINT`
  comments across the codebase would be noise obscuring genuine findings,
  not an auditable trail.
- **Introduced risk:** A future change that introduces a genuinely
  unbounded or miscalculated `std::array` index anywhere in
  `feature_tracking/` will not be caught by this specific checker (though
  it would still very likely be caught by the existing unit tests, a
  debug-build assertion in `std::array::operator[]` under some standard
  library implementations, or plain crash-and-investigate).
- **Compensating verification:** None specific to this check beyond the
  existing test suite (`test_corner_detector.cpp`/
  `test_optical_flow_tracker.cpp`, 24 cases) and the human review already
  performed to write the technical rationale above for every one of the
  38 original sites (see the git history of `.clang-tidy` and this log for
  the review record).
- **Approving authority:** `TODO(project)`
- **Approval date / review condition:** `TODO(project)` — revisit if a
  profiling result ever shows the hot loops here have enough headroom to
  afford `.at()`'s overhead, or if a future maintainer would rather have
  the per-site audit trail than the performance margin.
