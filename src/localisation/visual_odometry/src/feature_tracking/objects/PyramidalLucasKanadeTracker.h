/*!
 * @file            PyramidalLucasKanadeTracker.h
 *
 * @brief           Declares a reusable, dependency-minimal pyramidal
 *                  Lucas-Kanade optical-flow point tracker.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_PYRAMIDAL_LUCAS_KANADE_TRACKER_H
#define LUNAR_SIMULATOR_LOCALISATION_PYRAMIDAL_LUCAS_KANADE_TRACKER_H

/* C++ Standard Library Includes */
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

/* Object Include */
#include "feature_tracking/objects/FeatureTrackingLimits.h"
#include "feature_tracking/objects/FeatureTrackingStatus.h"
#include "feature_tracking/objects/ImageView.h"
#include "feature_tracking/objects/Point2D.h"

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief           Tracks a set of points from one image into another by
 *                  the same pyramidal Lucas-Kanade algorithm
 *                  `cv::calcOpticalFlowPyrLK` uses, reimplemented with no
 *                  third-party dependency, a single bounded allocation
 *                  confined to `initialize()`, and deterministic output.
 *
 * Lifecycle: construct (non-failing), `initialize()` once with the fixed
 * image resolution and tracking tuning, then any number of `track()`
 * calls, then `terminate()`. Not copyable, not movable, not thread-safe,
 * intended for a single-threaded caller. Boundary policy is deliberately
 * stricter than OpenCV's: a feature whose sampling window would leave the
 * image at any pyramid level is marked lost rather than reflected/
 * replicated at the border (see
 * `docs/compliance/feature_tracking/DEVIATION_LOG.md` DEV-FT-004). The
 * reported tracking error is a documented equivalent of OpenCV's internal
 * metric, not a byte-exact reproduction (DEV-FT-005) —
 * `VisualOdometryNode`'s `MAXIMUM_TRACKING_ERROR_PX` gate was re-verified
 * against this engine's metric during integration, not merely carried
 * over unchecked.
 */
class PyramidalLucasKanadeTracker
{
  public:
    /*!
     * @brief           Creates a valid, uninitialized tracker.
     */
    PyramidalLucasKanadeTracker() noexcept = default;

    /*!
     * @brief           Releases owned storage, if any was allocated by
     *                  `initialize()`.
     */
    ~PyramidalLucasKanadeTracker() noexcept = default;

    PyramidalLucasKanadeTracker(
        const PyramidalLucasKanadeTracker &otherTracker_in) = delete;
    PyramidalLucasKanadeTracker &operator=(
        const PyramidalLucasKanadeTracker &otherTracker_in) = delete;
    PyramidalLucasKanadeTracker(
        PyramidalLucasKanadeTracker &&otherTracker_in) = delete;
    PyramidalLucasKanadeTracker &operator=(
        PyramidalLucasKanadeTracker &&otherTracker_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Allocates internal pyramid buffers for a fixed
     *                  image resolution and stores the tracking tuning
     *                  used by every later `track()` call.
     *
     * This is the defined initialization phase: the one heap allocation
     * this object ever performs happens here, sized exactly to
     * `width_in`x`height_in` and `maximumPyramidLevel_in`, and is never
     * repeated or resized by `track()`.
     *
     * @param[in]       width_in
     *                  Fixed image width, in pixels, every later
     *                  `track()` call's images must match exactly.
     * @param[in]       height_in
     *                  Fixed image height, in pixels, every later
     *                  `track()` call's images must match exactly.
     * @param[in]       maximumPyramidLevel_in
     *                  Coarsest pyramid level index (0-based; matches
     *                  OpenCV's `maxLevel`); must be in
     *                  `[0, MAXIMUM_SUPPORTED_PYRAMID_LEVELS)`, and small
     *                  enough that the resulting coarsest level's width
     *                  and height (each halved, rounded up, per level)
     *                  are both `&gt;= windowSizePx_in` -- otherwise no
     *                  feature could ever have a fully in-bounds window
     *                  at that level.
     * @param[in]       windowSizePx_in
     *                  Side length, in pixels, of the square tracking
     *                  window; must be odd and `&gt;= 3`.
     * @param[in]       maximumIterations_in
     *                  Upper bound on Gauss-Newton refinement iterations
     *                  per feature per level; must be positive.
     * @param[in]       epsilonPx_in
     *                  Convergence threshold on the per-iteration
     *                  displacement update, in pixels; must be finite and
     *                  positive.
     * @param[in]       minimumEigenvalueThreshold_in
     *                  Minimum acceptable structure-tensor eigenvalue
     *                  below which a window is treated as too
     *                  low-texture to track; must be finite and
     *                  non-negative.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] FeatureTrackingStatus initialize(
        int width_in, int height_in, int maximumPyramidLevel_in,
        int windowSizePx_in, int maximumIterations_in, float epsilonPx_in,
        float minimumEigenvalueThreshold_in) noexcept;

    /*!
     * @brief           Tracks a set of points from `previousImage_in` into
     *                  `currentImage_in`.
     *
     * @param[in]       previousImage_in
     *                  Template image the points were observed in; its
     *                  `width`/`height` must exactly match `initialize()`.
     * @param[in]       currentImage_in
     *                  Image to locate each point's new position in; its
     *                  `width`/`height` must exactly match `initialize()`.
     * @param[in]       previousFeatures_in
     *                  Point positions to track, valid in
     *                  `[0, previousFeatureCount_in)`, expressed in
     *                  `previousImage_in`.
     * @param[in]       previousFeatureCount_in
     *                  Number of valid entries in `previousFeatures_in`;
     *                  must not exceed `MAXIMUM_SUPPORTED_FEATURES`.
     * @param[out]      currentFeatures_out
     *                  Tracked positions in `currentImage_in`, aligned
     *                  index-for-index with `previousFeatures_in`; only
     *                  entries with `trackingStatus_out[i] != 0` are
     *                  meaningful.
     * @param[out]      trackingStatus_out
     *                  Per-feature track success flag (`1` found, `0`
     *                  lost), aligned index-for-index with
     *                  `previousFeatures_in`.
     * @param[out]      trackingError_out
     *                  Per-feature tracking-error metric, aligned
     *                  index-for-index with `previousFeatures_in`; only
     *                  meaningful where `trackingStatus_out[i] != 0`.
     *
     * @return          Operation status.
     */
    [[nodiscard]] FeatureTrackingStatus track(
        const ImageView &previousImage_in, const ImageView &currentImage_in,
        const std::array<Point2D,
                         feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &previousFeatures_in,
        std::size_t previousFeatureCount_in,
        std::array<Point2D, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &currentFeatures_out,
        std::array<std::uint8_t,
                   feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &trackingStatus_out,
        std::array<float, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &trackingError_out) noexcept;

    /*!
     * @brief           Releases owned storage and returns to the
     *                  uninitialized lifecycle state.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] FeatureTrackingStatus terminate() noexcept;

    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Upper bound on the number of features `track()` may
     *              process in one call; every input/output buffer is
     *              sized to this constant.
     */
    static constexpr std::size_t MAXIMUM_SUPPORTED_FEATURES =
        feature_tracking::MAXIMUM_SUPPORTED_FEATURES;

    /*!
     * @brief       Upper bound on the number of Gaussian pyramid levels
     *              this tracker may be configured with.
     */
    static constexpr int MAXIMUM_SUPPORTED_PYRAMID_LEVELS =
        feature_tracking::MAXIMUM_SUPPORTED_PYRAMID_LEVELS;

    /*!
     * @brief       Upper bound on the configured tracking window's side
     *              length, in pixels. Bounds the fixed-size stack buffers
     *              `refineFeatureAtLevel()` uses to hold one feature's
     *              template window, well above the OpenCV-matching
     *              default of 21.
     */
    static constexpr int MAXIMUM_WINDOW_SIZE_PX = 51;

  private:
    /*!
     * @brief           One Gaussian pyramid level's fixed-size storage.
     *
     * Declared first because `buildPyramid()`'s signature and
     * `previousPyramid_`/`currentPyramid_`'s declared types (both below)
     * name this type, and neither a method's declared signature nor a
     * member's declared type is a complete-class context -- both must
     * already be able to see this type at that point.
     */
    struct PyramidLevel
    {
        /*!
         * @brief       This level's pixel intensity, row-major,
         *              `width`x`height` entries.
         */
        std::vector<float> intensity;

        /*!
         * @brief       This level's horizontal Sobel gradient; populated
         *              only for the previous/template pyramid, empty for
         *              the current/target pyramid.
         */
        std::vector<float> gradientX;

        /*!
         * @brief       This level's vertical Sobel gradient; populated
         *              only for the previous/template pyramid, empty for
         *              the current/target pyramid.
         */
        std::vector<float> gradientY;

        /*!
         * @brief       This level's width, in pixels.
         */
        int width{0};

        /*!
         * @brief       This level's height, in pixels.
         */
        int height{0};
    };

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Builds a Gaussian pyramid from `image_in` into
     *                  `pyramid_inout`, optionally also computing each
     *                  level's Sobel gradients.
     *
     * @param[in]       image_in
     *                  Full-resolution source image; assumed already
     *                  validated against the initialized resolution.
     * @param[in]       computeGradients_in
     *                  Whether to also populate each level's `gradientX`/
     *                  `gradientY` (true for the template/previous
     *                  pyramid, false for the target/current pyramid).
     * @param[in,out]   pyramid_inout
     *                  Pre-sized pyramid levels to fill; only the first
     *                  `maximumPyramidLevel_ + 1` entries are written.
     */
    void buildPyramid(
        const ImageView &image_in, bool computeGradients_in,
        std::array<PyramidLevel, MAXIMUM_SUPPORTED_PYRAMID_LEVELS>
            &pyramid_inout) const noexcept;

    /*!
     * @brief           Bilinearly samples an arbitrary row-major float
     *                  buffer at a sub-pixel position.
     *
     * Takes a plain buffer/width/height rather than a whole
     * `PyramidLevel` so it can sample any of a level's three buffers
     * (intensity, or either precomputed gradient) with the same
     * boundary-checked logic.
     *
     * @param[in]       buffer_in
     *                  Row-major buffer to sample, sized to at least
     *                  `width_in x height_in` entries.
     * @param[in]       width_in
     *                  Buffer width, in pixels.
     * @param[in]       height_in
     *                  Buffer height, in pixels.
     * @param[in]       position_in
     *                  Sub-pixel position, in the buffer's own pixel
     *                  coordinates.
     * @param[out]      value_out
     *                  Interpolated sample value, valid only when this
     *                  method returns true.
     *
     * @return          True when `position_in` (and the sampling
     *                  footprint immediately around it) lies inside
     *                  `[0, width_in) x [0, height_in)`; false when the
     *                  sample would require pixels outside that range,
     *                  per this tracker's no-extrapolation boundary
     *                  policy.
     */
    [[nodiscard]] static bool sampleBilinear(
        const std::vector<float> &buffer_in, int width_in, int height_in,
        Point2D position_in, float &value_out) noexcept;

    /*!
     * @brief           Refines one feature's estimated position at one
     *                  pyramid level by iterative Gauss-Newton
     *                  minimization of the windowed intensity mismatch.
     *
     * The template window is fixed for the duration of this call,
     * sampled from `previousLevel_in` at `templatePosition_in` (the
     * original tracked point, scaled to this level, which never moves
     * during refinement); only `searchPosition_inout` (this level's
     * estimate of where that template appears in the current/target
     * image) is iteratively updated.
     *
     * @param[in]       currentLevel_in
     *                  Target pyramid level to search in.
     * @param[in]       previousLevel_in
     *                  The same level of the template/previous pyramid,
     *                  supplying the fixed template window and its
     *                  precomputed gradients.
     * @param[in]       templatePosition_in
     *                  Fixed template window center, in
     *                  `previousLevel_in`'s own pixel coordinates.
     * @param[in,out]   searchPosition_inout
     *                  Search position estimate in `currentLevel_in`'s
     *                  own pixel coordinates: the propagated initial
     *                  guess on entry, the refined estimate on a
     *                  successful return.
     * @param[out]      error_out
     *                  Mean absolute residual of the final matched
     *                  window; valid only when this method returns true.
     *
     * @return          True when refinement converged (or exhausted its
     *                  iteration budget) without leaving the image
     *                  bounds or encountering a too-low-texture window;
     *                  false when the feature is lost at this level.
     */
    [[nodiscard]] bool refineFeatureAtLevel(
        const PyramidLevel &currentLevel_in,
        const PyramidLevel &previousLevel_in, Point2D templatePosition_in,
        Point2D &searchPosition_inout, float &error_out) const noexcept;

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       True once `initialize()` has succeeded and
     *              `terminate()` has not yet been called.
     */
    bool isInitialized_{false};

    /*!
     * @brief       Image width `initialize()` was called with, in pixels.
     */
    int width_{0};

    /*!
     * @brief       Image height `initialize()` was called with, in
     *              pixels.
     */
    int height_{0};

    /*!
     * @brief       Configured coarsest pyramid level index (0-based).
     */
    int maximumPyramidLevel_{0};

    /*!
     * @brief       Configured square tracking window side length, in
     *              pixels.
     */
    int windowSizePx_{0};

    /*!
     * @brief       Configured upper bound on refinement iterations per
     *              feature per level.
     */
    int maximumIterations_{0};

    /*!
     * @brief       Configured convergence threshold, in pixels.
     */
    float epsilonPx_{0.0F};

    /*!
     * @brief       Configured minimum acceptable structure-tensor
     *              eigenvalue.
     */
    float minimumEigenvalueThreshold_{0.0F};

    /*!
     * @brief       Template/previous-image pyramid, with gradients; one
     *              allocation sized by `initialize()`, reused by every
     *              `track()` call.
     */
    std::array<PyramidLevel, MAXIMUM_SUPPORTED_PYRAMID_LEVELS>
        previousPyramid_{};

    /*!
     * @brief       Target/current-image pyramid, intensity only; one
     *              allocation sized by `initialize()`, reused by every
     *              `track()` call.
     */
    std::array<PyramidLevel, MAXIMUM_SUPPORTED_PYRAMID_LEVELS>
        currentPyramid_{};
};

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_PYRAMIDAL_LUCAS_KANADE_TRACKER_H */
