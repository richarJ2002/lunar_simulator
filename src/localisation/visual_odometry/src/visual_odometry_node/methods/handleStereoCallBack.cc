/*!
 * @File:         handleStereoCallBack.cc
 *
 * @Brief:        Implements the per-frame stereo visual-odometry pipeline.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>

namespace localisation::visual_odometry
{

namespace
{

/*!
 * Wraps a single-channel 8-bit `cv::Mat` in a non-owning `ImageView`,
 * honoring OpenCV's own row stride (`step`) rather than assuming a
 * contiguous, unpadded buffer -- this is the boundary conversion
 * `feature_tracking::ImageView`'s own documentation describes.
 */
feature_tracking::ImageView imageViewFromMat(const cv::Mat &image_in)
{
    feature_tracking::ImageView view;

    /* Borrow, never copy, the Mat's own pixel buffer. */
    view.p_pixels = image_in.data;

    view.width = image_in.cols;
    view.height = image_in.rows;

    /* OpenCV's row stride, in bytes; may exceed width for a padded
     * buffer. */
    view.strideBytes = image_in.step[0];

    return view;
}

/*!
 * Converts a fixed-capacity engine output (position array plus a valid
 * count) into the `std::vector<cv::Point2f>` form the rest of this
 * pipeline (disparity lookup, PnP, visualization) already operates on.
 */
std::vector<cv::Point2f> toPoint2fVector(
    const std::array<feature_tracking::Point2D,
                     feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        &points_in,
    std::size_t count_in)
{
    std::vector<cv::Point2f> result;

    /* Reserve exactly what will be inserted; no growth reallocation. */
    result.reserve(count_in);

    for (std::size_t index = 0U; index < count_in; ++index)
    {
        result.emplace_back(points_in[index].x, points_in[index].y);
    }

    return result;
}

} /* anonymous namespace */

/* This function's cognitive-complexity finding is accepted: it is the
 * documented per-frame pipeline orchestrator (see its own Doxygen and
 * the numbered Step comments below), already delegating each
 * self-contained concern (visualization, pose update, point-cloud
 * publish) to its own method; the count is additionally inflated by
 * RCLCPP_*_THROTTLE macro expansion, not by genuinely nested human
 * logic. VisualOdometryNode.h/handleStereoCallBack.cc are outside the JSF AV
 * profile scope (see docs/compliance/feature_tracking/
 * JSF_AV_APPLICABILITY_PROFILE.md), so no further exception process
 * applies here. */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void VisualOdometryNode::handleStereoCallBack(
    const sensor_msgs::msg::Image::ConstSharedPtr &p_left_in,
    const sensor_msgs::msg::Image::ConstSharedPtr &p_right_in)
{
    /* Step 1 output: the current left frame as an OpenCV matrix. */
    cv::Mat currentLeft;

    /* Step 1 output: the current right frame as an OpenCV matrix. */
    cv::Mat currentRight;

    /*!
     * Step 1: convert both ROS images to OpenCV greyscale matrices. A
     * conversion failure (unexpected encoding, corrupt buffer) is not fatal
     * to the node; drop this frame and let the next one succeed instead of
     * crashing the estimator.
     */
    try
    {
        /* Decode the left image into an 8-bit greyscale matrix. */
        currentLeft = cv_bridge::toCvCopy(p_left_in, "mono8")->image;

        /* Decode the right image into an 8-bit greyscale matrix. */
        currentRight = cv_bridge::toCvCopy(p_right_in, "mono8")->image;
    }
    catch (const cv_bridge::Exception &error)
    {
        /* Log the failure (throttled) and give up on this frame only. */
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "LocCam conversion failed: %s", error.what());
        return;
    }

    /* Detection output: current-frame corner positions, valid only in
     * [0, currentCornerCount). */
    std::array<feature_tracking::Point2D,
              feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        currentCorners{};

    /* Number of valid entries written to currentCorners. */
    std::size_t currentCornerCount = 0U;

    /*!
     * Detect corner-like features in the CURRENT left image. These are not
     * used this callback; they become "previous frame features" the next
     * time this callback runs, and are also published now purely for visual
     * debugging (blue circles with no track yet). A detection failure here
     * is not fatal to the frame; treat it the same as "zero features
     * found" rather than aborting the callback.
     */
    static_cast<void>(cornerDetector.detect(imageViewFromMat(currentLeft),
                                            currentCorners,
                                            currentCornerCount));

    /* Current-frame corner pixel coordinates, converted for the
     * disparity-lookup/visualization code below. */
    const std::vector<cv::Point2f> detectedFeatures =
        toPoint2fVector(currentCorners, currentCornerCount);

    /* True only on the very first callback, before any frame has been
     * retained as "previous". */
    if (previousLeft.empty())
    {
        /*!
         * First callback ever: there is no previous frame to track from or
         * compute a delta pose against. Publish diagnostics, seed the
         * identity pose at the highest possible confidence (a single
         * stationary sample, inlierCount 1U), and store this frame as
         * "previous" for next time.
         */

        /* Publish the detected corners with no track, for diagnostics. */
        publishFeatureImage(p_left_in->header, currentLeft, detectedFeatures,
                            {}, {}, {});

        /* Publish an empty cloud so subscribers see a message every
         * frame. */
        publishPointCloud(p_left_in->header.stamp, {}, {}, worldFromOptical);

        /* Retain this frame so the next callback has a "previous" pair. */
        storePrevious(currentLeft, currentRight, p_left_in->header.stamp);

        /* The rover has not moved yet by definition; seed both poses at
         * identity. */
        const cv::Matx44d initialPose = cv::Matx44d::eye();

        /* Publish a single, maximally confident sample so the fusing EKF
         * has an initial visual measurement to accept. */
        publishOdometry(p_left_in->header.stamp, 1.0, 1U, initialPose,
                        initialPose);
        return;
    }

    /* Current frame's timestamp, in seconds, for the elapsed-time
     * calculation below. */
    const double currentStampS = stampToSeconds(p_left_in->header.stamp);

    /* Elapsed time since the previous accepted frame, in seconds. */
    const double dtS = currentStampS - previousStampS;

    /* Reject non-positive or implausibly large time steps. */
    if (!(dtS > 0.0) || dtS > 1.0)
    {
        /*!
         * Reject non-positive or implausibly large time steps (clock jumps,
         * simulation resets, dropped frames) rather than dividing by a bad
         * dtS later when computing twist. Diagnostics are still published
         * so a viewer can see that a frame arrived, even though odometry is
         * skipped.
         */

        /* Publish the detected corners with no track, for diagnostics. */
        publishFeatureImage(p_left_in->header, currentLeft, detectedFeatures,
                            {}, {}, {});

        /* Publish an empty cloud so subscribers see a message every
         * frame. */
        publishPointCloud(p_left_in->header.stamp, {}, {}, worldFromOptical);

        /* Still advance "previous" so the next callback measures dtS from
         * this frame instead of repeating the same bad gap. */
        storePrevious(currentLeft, currentRight, p_left_in->header.stamp);
        return;
    }

    /* Step 2 output: fixed-point disparity, scaled by 16 as StereoBM
     * returns it. */
    cv::Mat disparityFixed;

    /*!
     * Step 2: stereo disparity on the PREVIOUS frame pair. Disparity is
     * computed on the previous frame (not the current one) because it is
     * paired with previousFeatures below to reconstruct each feature's 3-D
     * position at the time it was first detected; StereoBM returns
     * fixed-point disparity scaled by 16, so divide back down to true pixel
     * disparity.
     */
    p_stereoMatcher->compute(previousLeft, previousRight, disparityFixed);

    /* True floating-point pixel disparity. */
    cv::Mat disparity;

    /* Undo StereoBM's internal 16x fixed-point scaling. */
    disparityFixed.convertTo(disparity, CV_32F, 1.0 / 16.0);

    /* Step 3 output: previous-frame corner positions, valid only in
     * [0, previousCornerCount); this is also exactly the input shape
     * track() below requires, so it is used directly rather than
     * round-tripped through a vector first. */
    std::array<feature_tracking::Point2D,
              feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        previousCorners{};

    /* Number of valid entries written to previousCorners. */
    std::size_t previousCornerCount = 0U;

    /*!
     * Step 3: re-detect features on the PREVIOUS left image (independent of
     * detectedFeatures above, which is the current frame's detection for the
     * *next* callback) and track them forward into the current frame with
     * pyramidal Lucas-Kanade optical flow. A detection failure here is
     * treated the same as "zero features found" rather than aborting the
     * frame.
     */
    static_cast<void>(cornerDetector.detect(imageViewFromMat(previousLeft),
                                            previousCorners,
                                            previousCornerCount));

    /* Step 3 output: each previous feature's tracked current-frame
     * position, aligned by index with previousCorners. */
    std::array<feature_tracking::Point2D,
              feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        trackedCorners{};

    /* Step 3 output: per-feature Lucas-Kanade track success flags,
     * aligned by index with previousCorners. */
    std::array<std::uint8_t, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        trackedStatus{};

    /* Step 3 output: per-feature Lucas-Kanade tracking error (mean
     * absolute intensity residual over the tracking window, matching
     * OpenCV's own `calcOpticalFlowPyrLK` error definition), aligned by
     * index with previousCorners. */
    std::array<float, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        trackedError{};

    /* Optical flow requires at least one feature to track. */
    if (previousCornerCount > 0U)
    {
        /* Track every previous-frame feature forward into the current
         * frame. */
        static_cast<void>(opticalFlowTracker.track(
            imageViewFromMat(previousLeft), imageViewFromMat(currentLeft),
            previousCorners, previousCornerCount, trackedCorners,
            trackedStatus, trackedError));
    }

    /* Previous-frame corner pixel coordinates, converted for the
     * disparity-lookup/visualization code below. */
    const std::vector<cv::Point2f> previousFeatures =
        toPoint2fVector(previousCorners, previousCornerCount);

    /* Tracked current-frame pixel coordinates, converted for the
     * disparity-lookup/visualization code below; only entries in
     * [0, previousCornerCount) are meaningful, matching previousFeatures'
     * size. */
    const std::vector<cv::Point2f> currentFeatures =
        toPoint2fVector(trackedCorners, previousCornerCount);

    /* Per-feature track success flags, aligned by index with
     * previousFeatures/currentFeatures. */
    const std::vector<unsigned char> trackingStatus(
        trackedStatus.begin(),
        trackedStatus.begin() +
            static_cast<std::ptrdiff_t>(previousCornerCount));

    /* Per-feature tracking error, aligned by index with
     * previousFeatures/currentFeatures. */
    const std::vector<float> trackingError(
        trackedError.begin(),
        trackedError.begin() +
            static_cast<std::ptrdiff_t>(previousCornerCount));

    /* Step 4 output: reconstructed previous-frame 3-D points that survive
     * every gate below. */
    std::vector<cv::Point3f> previousPoints3d;

    /* Step 4 output: each surviving point's tracked current-frame pixel
     * position, aligned with previousPoints3d. */
    std::vector<cv::Point2f> currentPoints2d;

    /*!
     * Step 4: for every successfully tracked feature, reconstruct its 3-D
     * position in the PREVIOUS optical frame from stereo disparity, then
     * keep only the ones passing a chain of physically motivated gates:
     * a Lucas-Kanade "lost track" flag, a large tracking error (likely a
     * mismatch rather than the true corresponding point), disparity too
     * small to be reliable (large or infinite depth), and depth outside the
     * plausible near/far bounds. Surviving pairs feed PnP RANSAC below.
     */
    for (std::size_t index = 0U; index < previousFeatures.size(); ++index)
    {
        /* Reject a lost track or an untrustworthy match. */
        if (trackingStatus[index] == 0U ||
            trackingError[index] > MAXIMUM_TRACKING_ERROR_PX)
        {
            continue;
        }

        /* Round the feature's previous-frame pixel column to an integer
         * disparity-lookup index; u/v is standard pinhole pixel-
         * coordinate notation. */
        // NOLINTNEXTLINE(readability-identifier-length)
        const int u =
            static_cast<int>(std::lround(previousFeatures[index].x));

        /* Round the feature's previous-frame pixel row to an integer
         * disparity-lookup index. */
        // NOLINTNEXTLINE(readability-identifier-length)
        const int v =
            static_cast<int>(std::lround(previousFeatures[index].y));

        /* Reject a feature whose rounded pixel falls outside the disparity
         * image. */
        if (u < 0 || v < 0 || u >= disparity.cols || v >= disparity.rows)
        {
            continue;
        }

        /* Look up this pixel's previously computed stereo disparity. */
        const float disparityPx = disparity.at<float>(v, u);

        /* Reject disparity too small to be reliable (implies very large or
         * infinite depth). */
        if (!(disparityPx > 1.0F))
        {
            continue;
        }

        /* Pinhole stereo depth: depth = focalLength * baseline / disparity. */
        const double depthM =
            fxPx * baselineM / static_cast<double>(disparityPx);

        /* Reject depth outside the plausible near/far bounds. */
        if (!(depthM > 0.08) || depthM > maximumDepthM)
        {
            continue;
        }

        /*!
         * Back-project the pixel to a 3-D point in the previous optical
         * frame using the standard pinhole inverse-projection equations:
         *   x = (u - cx) * depth / fx
         *   y = (v - cy) * depth / fy
         *   z = depth
         */

        /* Back-projected x coordinate in the previous optical frame. */
        const double positionXM =
            (previousFeatures[index].x - cxPx) * depthM / fxPx;

        /* Back-projected y coordinate in the previous optical frame. */
        const double positionYM =
            (previousFeatures[index].y - cyPx) * depthM / fyPx;

        /* Record the reconstructed 3-D point for PnP RANSAC below. */
        previousPoints3d.emplace_back(static_cast<float>(positionXM),
                                      static_cast<float>(positionYM),
                                      static_cast<float>(depthM));

        /* Record this point's tracked current-frame pixel, aligned by
         * index with previousPoints3d. */
        currentPoints2d.push_back(currentFeatures[index]);
    }

    /*!
     * Publish the visualization for this frame regardless of whether a pose
     * update succeeds below, so a viewer can always see current tracking
     * quality.
     */
    publishFeatureImage(p_left_in->header, currentLeft, previousFeatures,
                        currentFeatures, trackingStatus, currentPoints2d);

    /* Whether Step 5 below actually produced and accepted a pose
     * update. */
    bool estimated = false;

    /* Only attempt PnP RANSAC when there are enough correspondences to
     * make it meaningful. */
    if (previousPoints3d.size() >=
        static_cast<std::size_t>(minimumCorrespondences))
    {
        /* Step 5 output: Rodrigues rotation vector from PnP RANSAC. */
        cv::Mat rotationVector;

        /* Step 5 output: translation vector from PnP RANSAC. */
        cv::Mat translationVector;

        /* Step 5 output: indices, into previousPoints3d/currentPoints2d,
         * that PnP RANSAC accepted as inliers. */
        std::vector<int> inliers;

        /*!
         * Step 5: estimate the previous-to-current camera motion with PnP
         * RANSAC using the reconstructed 3-D points (previous frame)
         * against their tracked 2-D pixel locations (current frame). Only
         * attempt this when there are enough correspondences to make RANSAC
         * meaningful, and only accept the result if enough of them also
         * survive as RANSAC inliers -- both gates exist because a
         * confident-looking but underdetermined or outlier-dominated solve
         * is worse than skipping the update entirely.
         */
        estimated = cv::solvePnPRansac(
            previousPoints3d, currentPoints2d, cameraMatrix, cv::noArray(),
            rotationVector, translationVector, false,
            PNP_RANSAC_MAXIMUM_ITERATIONS, PNP_RANSAC_REPROJECTION_ERROR_PX,
            PNP_RANSAC_CONFIDENCE, inliers, cv::SOLVEPNP_ITERATIVE);

        /* Additionally require enough surviving inliers, not just a
         * reported success. */
        estimated = estimated &&
                    inliers.size() >=
                        static_cast<std::size_t>(minimumCorrespondences);

        /* Only apply the estimate if it passed both gates above. */
        if (estimated)
        {
            /* Fold the accepted rotation/translation into the accumulated
             * pose and publish it. */
            updatePose(rotationVector, translationVector,
                       p_left_in->header.stamp, dtS, previousPoints3d,
                       inliers);
        }
    }

    /* No pose update was produced or accepted this frame. */
    if (!estimated)
    {
        /*!
         * When no pose update was possible, still publish the reconstructed
         * (but unfiltered-by-RANSAC) 3-D points so downstream consumers and
         * visualization keep seeing feature depth even during a dropout,
         * and warn (throttled, since this can repeat every frame in a bad
         * patch of terrain) that odometry itself did not advance this
         * cycle.
         */

        /* Every reconstructed point is published here, since none were
         * filtered by RANSAC; build the identity index mapping PnP would
         * otherwise have provided. */
        std::vector<int> correlatedIndices(previousPoints3d.size());

        /* Fill 0, 1, 2, ... so every reconstructed point is included. */
        std::iota(correlatedIndices.begin(), correlatedIndices.end(), 0);

        /* Publish every reconstructed point, expressed in the current
         * (not yet advanced) accumulated pose. */
        publishPointCloud(p_left_in->header.stamp, previousPoints3d,
                          correlatedIndices, worldFromOptical);

        /* Let an operator know odometry did not advance this cycle. */
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Visual odometry lacks stereo/temporal correspondences");
    }

    /* Retain this frame as "previous" for the next callback, whether or
     * not a pose update was accepted this cycle. */
    storePrevious(currentLeft, currentRight, p_left_in->header.stamp);
}

} /* namespace localisation::visual_odometry */
