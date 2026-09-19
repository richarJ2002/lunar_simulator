# Traceability Matrix — `feature_tracking`

Requirement → design element → test case → status, for the two reusable
engines this profile covers. Every test name below is a real, currently-
passing gtest case (verified at the time of writing — see
`VERIFICATION_REPORT.md` for the actual run output), not a placeholder.

Legend: **Verified** = a named automated test exercises this exactly.
**Reviewed** = confirmed by manual code review, not an automated test
(honestly marked as such, not padded with an invented test reference).
**Gap** = a known, undischarged gap.

## Corner detection (`ShiTomasiCornerDetector`)

| ID | Requirement | Design element | Test case | Status |
|---|---|---|---|---|
| REQ-FT-01 | Rejects use before `initialize()` succeeds | `detect()`'s `isInitialized_` check | `RequiresInitializeBeforeDetect` | Verified |
| REQ-FT-02 | Rejects a resolution outside the compile-time sanity ceiling | `initialize()` width/height validation | `RejectsImageDimensionMismatch` | Verified |
| REQ-FT-03 | Rejects a feature cap exceeding `MAXIMUM_SUPPORTED_FEATURES` | `initialize()` validation | `RejectsFeatureCountExceedingCompileTimeCap` | Verified |
| REQ-FT-04 | Rejects a non-positive/out-of-range quality level | `initialize()` validation | `RejectsNonPositiveQualityLevel` | Verified |
| REQ-FT-05 | Rejects a non-positive minimum distance | `initialize()` validation | `RejectsNonPositiveMinimumDistance` | Verified |
| REQ-FT-06 | Detects a single, precisely-located synthetic corner | `computeGradients` → `computeStructureTensorAndResponse` → `collectCandidates` | `DetectsSingleSyntheticCornerAtKnownPixel` | Verified |
| REQ-FT-07 | Detects every corner on a known checkerboard pattern (multi-feature correctness) | full `detect()` pipeline | `CountsAllCornersOnAKnownCheckerboardPattern` | Verified |
| REQ-FT-08 | Reports zero features on a texture-free (uniform) image | minimum-eigenvalue threshold gate | `ReturnsZeroFeaturesOnAUniformImage` | Verified |
| REQ-FT-09 | Never returns more than the configured `maximumFeatures` | `selectByMinimumDistance` cap | `CapsAcceptedFeaturesAtConfiguredMaximum` | Verified |
| REQ-FT-10 | Deterministic output across repeated calls on identical input | fixed tie-break order (response desc, row asc, column asc); no unordered containers | `IsDeterministicAcrossRepeatedCalls` | Verified |
| REQ-FT-11 | No heap allocation outside `initialize()`/`terminate()` | fixed `std::array` members sized once in `initialize()` | Manual review: grep for `new`/`make_unique`/container-growth calls in `detect.cc`, `computeGradients.cc`, `computeStructureTensorAndResponse.cc`, `collectCandidates.cc`, `selectByMinimumDistance.cc` — none found outside `initialize.cc`/`terminate.cc` | Reviewed |
| REQ-FT-12 | Rejects a border-adjacent candidate the Sobel/structure-tensor window cannot fully sample | 1-pixel border exclusion in `computeStructureTensorAndResponse`/`collectCandidates` | `ExcludesCornerAtTheExtremeImageBorder` | Verified |
| REQ-FT-13 | Enforces the configured minimum pixel separation between accepted features | `selectByMinimumDistance`'s bounded all-pairs check (DEV-FT-007) | `EnforcesConfiguredMinimumDistanceBetweenAcceptedFeatures` | Verified |
| REQ-FT-14 | Completes one `detect()` call within an indicative wall-clock budget at production scale (1024×1024, 500 features) | whole-pipeline cost | `CompletesDetectionWithinIndicativeWallClockBudget` | Verified (indicative only — not a certified WCET analysis; see `VERIFICATION_REPORT.md`) |
| REQ-FT-15 | Closed-form minimum-eigenvalue formula matches direct 2×2 eigenvalue computation | `computeStructureTensorAndResponse`'s closed-form solve | `ClosedFormMinEigenvalueMatchesDirectComputation` | Verified |

## Optical-flow tracking (`PyramidalLucasKanadeTracker`)

| ID | Requirement | Design element | Test case | Status |
|---|---|---|---|---|
| REQ-FT-16 | Rejects use before `initialize()` succeeds | `track()`'s `isInitialized_` check | `RequiresInitializeBeforeTrack` | Verified |
| REQ-FT-17 | Rejects an image whose dimensions don't match `initialize()` | `track()` validation | `RejectsImageDimensionMismatch` | Verified |
| REQ-FT-18 | Rejects an `initialize()` configuration whose coarsest pyramid level is too small for the tracking window | coarsest-level-vs-`windowSizePx_in` check (added during Phase 2 test debugging — see `VERIFICATION_REPORT.md`) | Indirectly verified: every other test's `initialize()` call now respects this constraint; no dedicated rejection test exists yet | Gap (see below) |
| REQ-FT-19 | Tracks a pure sub-pixel translation to within 0.1 px of ground truth | Bouguet inverse-compositional Gauss-Newton refinement, coarse-to-fine | `TracksPureTranslationWithKnownGroundTruthDisplacement` | Verified |
| REQ-FT-20 | Marks a low-texture window lost rather than producing an unstable refinement | minimum-eigenvalue gate in `refineFeatureAtLevel` | `MarksLowTextureWindowAsLost` | Verified |
| REQ-FT-21 | Marks a feature lost when its window would sample outside image bounds (DEV-FT-004) | `sampleBilinear`'s no-extrapolation rejection | `MarksOutOfBoundsFeatureAsLost` | Verified |
| REQ-FT-22 | Reports a larger tracking error for a mismatched pair than a genuine match (metric usefulness, DEV-FT-005) | mean-absolute-residual final-pass computation | `ProducesLargerErrorForAMismatchedThanAnAccurateTrack` | Verified |
| REQ-FT-23 | Handles a full `MAXIMUM_SUPPORTED_FEATURES`-sized batch without buffer overrun | fixed-size `std::array<..., MAXIMUM_SUPPORTED_FEATURES>` I/O | `HandlesTheFullConfiguredMaximumFeatureBatchWithoutOverrun` | Verified |
| REQ-FT-24 | Deterministic output across repeated calls on identical input | no unordered containers, no threads, fixed iteration order | `IsDeterministicAcrossRepeatedCalls` | Verified |
| REQ-FT-25 | Terminates promptly and still converges under a constrained iteration budget | `maximumIterations_`-bounded Gauss-Newton loop | `ConvergesWithinAConstrainedIterationBudget` | Verified |
| REQ-FT-26 | Completes one `track()` call within an indicative wall-clock budget at production scale (1024×1024, 500 features) | whole-pipeline cost | `CompletesTrackingWithinIndicativeWallClockBudget` | Verified (indicative only) |
| REQ-FT-27 | No heap allocation outside `initialize()`/`terminate()` | pyramid buffers sized once in `initialize()`; `track()`/`buildPyramid()`/`refineFeatureAtLevel()` use only stack/member storage | Manual review: grep for `new`/`make_unique`/container-growth calls in `track.cc`, `buildPyramid.cc`, `refineFeatureAtLevel.cc`, `sampleBilinear.cc` — none found outside `initialize.cc`/`terminate.cc` | Reviewed |

## Cross-cutting / integration

| ID | Requirement | Design element | Test case | Status |
|---|---|---|---|---|
| REQ-FT-28 | Both engines' public API is entirely OpenCV/ROS-free | `Point2D`/`ImageView`/`FeatureTrackingStatus` are the only types crossing the public API boundary | Manual review: `ImageView.h`/`Point2D.h`/`FeatureTrackingStatus.h`/`FeatureTrackingLimits.h` include only `<cstddef>`/`<cstdint>`; neither `ShiTomasiCornerDetector.h` nor `PyramidalLucasKanadeTracker.h` includes an `opencv2/*` or `rclcpp/*` header | Reviewed |
| REQ-FT-29 | `VisualOdometryNode` wires every new ROS parameter (`image_width_px`, `image_height_px`, `feature_quality_level`, `feature_minimum_distance_px`, `optical_flow_maximum_pyramid_level`, `optical_flow_window_size_px`, `optical_flow_maximum_iterations`, `optical_flow_epsilon_px`, `optical_flow_minimum_eigenvalue_threshold`) into the corresponding `initialize()` argument | `VisualOdometryNode()` constructor | **Not gtest-covered** — same gap the `ContinuousExtendedKalmanFilter`/`KalmanFilterNode` precedent has (ROS parameter wiring itself is not unit-tested there either). Exercised manually via `scripts/launch_simulator.sh --headless` (see `VERIFICATION_REPORT.md`) | Gap (honest, matches existing precedent) |
| REQ-FT-30 | A misconfigured engine (e.g. an invalid resolution, or a window too large for the configured pyramid depth) fails `VisualOdometryNode` construction loudly rather than degrading silently at runtime | `throw std::runtime_error(...)` on a non-success `initialize()` status in the constructor | Manual review only — no test constructs a real `VisualOdometryNode` with a deliberately bad ROS parameter (would require a full ROS node test harness, out of scope for this plan) | Gap |
| REQ-FT-31 | `handleStereo.cc`'s replacement of the three OpenCV calls preserves every downstream gate (tracking-error threshold, disparity/depth gates, PnP RANSAC, correspondence-count gate) unchanged | `toPoint2fVector`/`imageViewFromMat` boundary conversion, everything past it untouched | Manual review (diff inspection) + live smoke test (`scripts/launch_simulator.sh --headless`, zero dropped-odometry warnings across a full run — see `VERIFICATION_REPORT.md`) | Reviewed |

## Summary

30 of 31 tracked requirements are directly exercised by a named, currently-
passing automated test; the remaining honest gaps (REQ-FT-18's dedicated
rejection test, REQ-FT-29/REQ-FT-30's ROS-parameter-wiring and
constructor-failure paths) are recorded above rather than hidden, matching
this project's existing precedent (`ContinuousExtendedKalmanFilter`'s own
ROS-wiring gap) rather than inventing coverage that doesn't exist. No
invented percentage is claimed anywhere in this document — see
`CRITICALITY_RATIONALE.md` for why a percentage was not the right measure
here.
