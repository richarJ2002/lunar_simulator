/*!
 * @file            ShiTomasiCornerDetector.h
 *
 * @brief           Declares a reusable, dependency-minimal Shi-Tomasi
 *                  (minimum-eigenvalue) corner detector.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_SHI_TOMASI_CORNER_DETECTOR_H
#define LUNAR_SIMULATOR_LOCALISATION_SHI_TOMASI_CORNER_DETECTOR_H

/* C++ Standard Library Includes */
#include <array>
#include <cstddef>
#include <vector>

/* Object Include */
#include "feature_tracking/objects/FeatureTrackingLimits.h"
#include "feature_tracking/objects/FeatureTrackingStatus.h"
#include "feature_tracking/objects/ImageView.h"
#include "feature_tracking/objects/Point2D.h"

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief           Detects corner-like features by the same minimum
 *                  structure-tensor-eigenvalue algorithm OpenCV's
 *                  `cv::goodFeaturesToTrack(..., useHarrisDetector=false)`
 *                  uses, reimplemented with no third-party dependency, a
 *                  single bounded allocation confined to `initialize()`,
 *                  and deterministic output ordering.
 *
 * Lifecycle: construct (non-failing), `initialize()` once with the fixed
 * image resolution and detection tuning, then any number of `detect()`
 * calls, then `terminate()`. Not copyable, not movable (owns fixed
 * internal storage sized by `initialize()`), not thread-safe, and intended
 * for a single-threaded caller, matching `VisualOdometryNode`'s own
 * documented executor assumption. See
 * `docs/compliance/feature_tracking/JSF_AV_APPLICABILITY_PROFILE.md` for
 * the applicable coding profile and
 * `docs/compliance/feature_tracking/DEVIATION_LOG.md` for recorded design
 * deviations (DEV-FT-002 covers the single `initialize()`-time
 * allocation).
 */
class ShiTomasiCornerDetector
{
  public:
    /*!
     * @brief           Creates a valid, uninitialized detector.
     */
    ShiTomasiCornerDetector() noexcept = default;

    /*!
     * @brief           Releases owned storage, if any was allocated by
     *                  `initialize()`.
     */
    ~ShiTomasiCornerDetector() noexcept = default;

    ShiTomasiCornerDetector(const ShiTomasiCornerDetector &otherDetector_in) =
        delete;
    ShiTomasiCornerDetector &operator=(
        const ShiTomasiCornerDetector &otherDetector_in) = delete;
    ShiTomasiCornerDetector(ShiTomasiCornerDetector &&otherDetector_in) =
        delete;
    ShiTomasiCornerDetector &operator=(
        ShiTomasiCornerDetector &&otherDetector_in) = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Allocates internal buffers for a fixed image
     *                  resolution and stores the detection tuning used by
     *                  every later `detect()` call.
     *
     * This is the defined initialization phase: the one heap allocation
     * this object ever performs happens here, sized exactly to
     * `width_in`x`height_in`, and is never repeated or resized by
     * `detect()`. Calling `initialize()` again after a prior successful
     * call re-allocates (still not from within `detect()`) and discards
     * any prior state.
     *
     * @param[in]       width_in
     *                  Fixed image width, in pixels, every later
     *                  `detect()` call's `ImageView` must match exactly.
     * @param[in]       height_in
     *                  Fixed image height, in pixels, every later
     *                  `detect()` call's `ImageView` must match exactly.
     * @param[in]       maximumFeatures_in
     *                  Upper bound on features accepted per `detect()`
     *                  call; must not exceed `MAXIMUM_SUPPORTED_FEATURES`.
     * @param[in]       qualityLevel_in
     *                  Acceptance threshold as a fraction of the frame's
     *                  strongest response; must be finite and in `(0,1]`.
     * @param[in]       minimumDistancePx_in
     *                  Minimum Euclidean separation enforced between
     *                  accepted features, in pixels; must be finite and
     *                  positive.
     *
     * @return          Lifecycle status.
     */
    [[nodiscard]] FeatureTrackingStatus initialize(
        int width_in, int height_in, std::size_t maximumFeatures_in,
        float qualityLevel_in, float minimumDistancePx_in) noexcept;

    /*!
     * @brief           Detects corner-like features in one image.
     *
     * @param[in]       image_in
     *                  Image to detect features in; its `width`/`height`
     *                  must exactly match the values `initialize()` was
     *                  called with.
     * @param[out]      features_out
     *                  Detected feature positions, valid only in
     *                  `[0, featureCount_out)`, in deterministic
     *                  descending-response acceptance order.
     * @param[out]      featureCount_out
     *                  Number of valid entries written to `features_out`.
     *
     * @return          Operation status.
     */
    [[nodiscard]] FeatureTrackingStatus detect(
        const ImageView &image_in,
        std::array<Point2D, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &features_out,
        std::size_t &featureCount_out) noexcept;

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
     * @brief       Upper bound on the number of features `detect()` may
     *              report in one call; every output buffer is sized to
     *              this constant.
     */
    static constexpr std::size_t MAXIMUM_SUPPORTED_FEATURES =
        feature_tracking::MAXIMUM_SUPPORTED_FEATURES;

    /*!
     * @brief       Upper bound on how many local-maxima candidates one
     *              `detect()` call retains before ranking and
     *              min-distance selection, bounding worst-case candidate
     *              count independently of image content.
     */
    static constexpr std::size_t MAXIMUM_CANDIDATE_POOL =
        MAXIMUM_SUPPORTED_FEATURES * 8U;

  private:
    /*!
     * @brief           One ranked local-maxima candidate awaiting
     *                  min-distance selection.
     *
     * Declared first because `candidatePool_`'s own declared type (under
     * PRIVATE MEMBERS below) names this type, and a member's declared type
     * is not a complete-class context -- it must already be visible at
     * that point, the same way `MAXIMUM_SUPPORTED_FEATURES`/
     * `MAXIMUM_CANDIDATE_POOL` above must precede `detect()`'s signature.
     */
    struct Candidate
    {
        /*!
         * @brief       Candidate's pixel position.
         */
        Point2D position;

        /*!
         * @brief       Candidate's minimum-eigenvalue response.
         */
        float response{0.0F};

        /*!
         * @brief       Zero-based row index, used only to break exact
         *              response ties in a fixed, deterministic order.
         */
        int row{0};

        /*!
         * @brief       Zero-based column index, used only to break exact
         *              response ties in a fixed, deterministic order.
         */
        int column{0};
    };

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Computes the horizontal and vertical Sobel
     *                  gradients of `image_in` into `gradientX_`/
     *                  `gradientY_`.
     *
     * @param[in]       image_in
     *                  Image to differentiate; assumed already validated
     *                  against the initialized resolution by the caller.
     */
    void computeGradients(const ImageView &image_in) noexcept;

    /*!
     * @brief           Forms the local structure tensor from
     *                  `gradientX_`/`gradientY_` and writes each pixel's
     *                  minimum eigenvalue into `response_`, tracking the
     *                  frame's maximum response.
     *
     * @return          The frame's maximum response value.
     */
    [[nodiscard]] float computeStructureTensorAndResponse() noexcept;

    /*!
     * @brief           Extracts thresholded 3x3 local maxima from
     *                  `response_` into the fixed-capacity candidate pool,
     *                  bounded to `MAXIMUM_CANDIDATE_POOL` entries by a
     *                  min-heap on response.
     *
     * @param[in]       responseThreshold_in
     *                  Minimum response, exclusive, a pixel must exceed to
     *                  be considered a candidate.
     *
     * @return          Number of candidates collected, always
     *                  `&lt;= MAXIMUM_CANDIDATE_POOL`.
     */
    [[nodiscard]] std::size_t collectCandidates(
        float responseThreshold_in) noexcept;

    /*!
     * @brief           Sorts the collected candidates by descending
     *                  response (with a deterministic tie-break) and
     *                  greedily accepts them subject to the configured
     *                  minimum separation.
     *
     * The all-pairs distance check against already-accepted features is
     * `O(candidateCount x acceptedCount)`, which is already small and
     * fixed (bounded above by `MAXIMUM_CANDIDATE_POOL x
     * MAXIMUM_SUPPORTED_FEATURES`, both compile-time constants) because
     * `collectCandidates()` already bounds `candidateCount_in`; a spatial
     * partitioning scheme would only help for an unbounded candidate
     * count, which does not occur here.
     *
     * @param[in]       candidateCount_in
     *                  Number of valid entries in the candidate pool.
     * @param[out]      features_out
     *                  Accepted feature positions, valid only in
     *                  `[0, return value)`.
     *
     * @return          Number of accepted features, always
     *                  `&lt;= maximumFeatures`.
     */
    [[nodiscard]] std::size_t selectByMinimumDistance(
        std::size_t candidateCount_in,
        std::array<Point2D, MAXIMUM_SUPPORTED_FEATURES> &features_out)
        noexcept;

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       True once `initialize()` has succeeded and `terminate()`
     *              has not yet been called.
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
     * @brief       Configured cap on features accepted per `detect()`
     *              call.
     */
    std::size_t maximumFeatures_{0U};

    /*!
     * @brief       Configured acceptance threshold as a fraction of the
     *              frame's strongest response.
     */
    float qualityLevel_{0.0F};

    /*!
     * @brief       Configured minimum Euclidean separation between
     *              accepted features, in pixels.
     */
    float minimumDistancePx_{0.0F};

    /*!
     * @brief       Horizontal Sobel gradient, one allocation sized by
     *              `initialize()`, reused by every `detect()` call.
     */
    std::vector<float> gradientX_;

    /*!
     * @brief       Vertical Sobel gradient, one allocation sized by
     *              `initialize()`, reused by every `detect()` call.
     */
    std::vector<float> gradientY_;

    /*!
     * @brief       Per-pixel minimum structure-tensor eigenvalue, one
     *              allocation sized by `initialize()`, reused by every
     *              `detect()` call.
     */
    std::vector<float> response_;

    /*!
     * @brief       Fixed-capacity candidate pool, indices
     *              `[0, MAXIMUM_CANDIDATE_POOL)`, reused by every
     *              `detect()` call; never resized after `initialize()`.
     */
    std::array<Candidate, MAXIMUM_CANDIDATE_POOL> candidatePool_{};
};

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_SHI_TOMASI_CORNER_DETECTOR_H */
