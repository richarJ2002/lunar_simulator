/*!
 * @File:         VisualOdometryNode.h
 *
 * @Brief:        Declares the stereo visual-odometry ROS node.
 *
 * @Date:         15/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_VISUAL_ODOMETRY_NODE_H
#define LUNAR_SIMULATOR_LOCALISATION_VISUAL_ODOMETRY_NODE_H

/* Function Includes */
/* None */

/* Object Include */
#include "feature_tracking/objects/FeatureTrackingStatus.h"
#include "feature_tracking/objects/ImageView.h"
#include "feature_tracking/objects/Point2D.h"
#include "feature_tracking/objects/PyramidalLucasKanadeTracker.h"
#include "feature_tracking/objects/ShiTomasiCornerDetector.h"

/* Data include */
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

namespace localisation::visual_odometry
{

/*!
 * @brief           Estimates rover motion from the Alpha LocCam stereo pair.
 *
 * The estimator computes disparity between the previous stereo pair with
 * `StereoBM`, tracks image corners between LocCam frames with pyramidal
 * Lucas-Kanade optical flow, reconstructs the previous 3-D feature positions
 * from disparity, and estimates the inter-frame rigid transform with PnP
 * RANSAC. Estimated pose is accumulated into `worldFromOptical`, a rigid
 * transform from the optical frame at node start-up to the current optical
 * frame. Odometry withholds a pose update and keeps the previous accumulated
 * pose whenever too few reliable correspondences are available; it is
 * intended for a single-threaded executor.
 */
class VisualOdometryNode final : public rclcpp::Node
{
  public:
    /** @brief Action applied to the retained stereo keyframe after a frame. */
    enum class KeyframeAction : std::uint8_t
    {
        RETAIN_KEYFRAME       = 0U,
        ADVANCE_KEYFRAME      = 1U,
        MARK_POSE_UNAVAILABLE = 2U
    };

    /** @brief Six-dimensional covariance ordered translation then rotation. */
    using PoseCovariance = cv::Matx<double, 6, 6>;

    /** @brief Tunable thresholds used to assess one PnP solution. */
    struct VisualQualityConfiguration
    {
        int    imageWidthPx{1024};
        int    imageHeightPx{1024};
        int    occupancyGridRows{4};
        int    occupancyGridColumns{4};
        double fxPx{800.0};
        double baselineM{0.15};
        double nearMidDepthM{12.0};
        double minimumCoverageRatio{0.25};
        double minimumNearMidRatio{0.25};
        double preferredDisparityPx{8.0};
        double disparityStddevPx{1.0};
        double reprojectionNoiseFloorPx{0.25};
        double maximumNormalCondition{1.0e12};
        double minimumTranslationVarianceM2{1.0e-5};
        double maximumTranslationVarianceM2{4.0};
        double minimumRotationVarianceRad2{1.0e-6};
        double maximumRotationVarianceRad2{0.25};
    };

    /** @brief Geometry diagnostics and covariance for one accepted PnP solve.
     */
    struct VisualPoseQuality
    {
        bool                  isValid{false};
        double                occupancyRatio{0.0};
        double                nearMidRatio{0.0};
        double                reprojectionRmsPx{0.0};
        double                normalCondition{0.0};
        std::array<double, 3> disparityPercentilesPx{};
        std::array<double, 3> depthPercentilesM{};
        PoseCovariance        relativeCovariance{PoseCovariance::zeros()};
    };

    /*!
     * @brief       Approximate-time synchronization policy for the LocCam
     *              stereo pair.
     */
    using StereoPolicy = message_filters::sync_policies::
        ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    /*!
     * @brief           Configures stereo visual odometry.
     *
     * Declares all ROS parameters, builds the pinhole camera matrix and
     * `StereoBM` matcher, derives the fixed camera-to-body transform from the
     * configured mount position and pitch, and subscribes to the
     * approximate-time-synchronized LocCam pair.
     */
    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    VisualOdometryNode() :
        Node("visual_odometry")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: the left LocCam stream. */
        const std::string leftTopic = declare_parameter<std::string>(
            "left_image_topic",
            "/" + systemName + "/drivers/loccam/left");

        /* Input topic: the right LocCam stream. */
        const std::string rightTopic = declare_parameter<std::string>(
            "right_image_topic",
            "/" + systemName + "/drivers/loccam/right");

        /* Output topic: accumulated visual odometry. */
        const std::string outputTopic = declare_parameter<std::string>(
            "odometry_topic",
            "/" + systemName + "/localisation/visual/odometry");

        /* Output topic: the sparse inlier feature point cloud. */
        const std::string pointCloudTopic = declare_parameter<std::string>(
            "point_cloud_topic",
            "/" + systemName + "/localisation/visual/point_cloud");

        /* Output topic: the annotated feature/track visualization. */
        const std::string featureImageTopic = declare_parameter<std::string>(
            "feature_image_topic",
            "/" + systemName + "/localisation/visual/features");

        /* Explicitly starts a new identity-relative visual epoch after an
         * unrecoverable continuity loss. */
        const std::string resetTopic = declare_parameter<std::string>(
            "reset_topic",
            "/" + systemName + "/localisation/visual/reset");

        /* Fixed world frame in which odometry and point clouds publish. */
        odomFrame =
            declare_parameter<std::string>("odom_frame",
                                           systemName + "/startup_fixed");

        /* Child body frame published in odometry messages. */
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");

        /* Horizontal focal length, in pixels, of both LocCam sensors. */
        fxPx = declare_parameter<double>("fx_px", 800.0);

        /* Vertical focal length, in pixels, of both LocCam sensors. */
        fyPx = declare_parameter<double>("fy_px", 800.0);

        /* Principal-point horizontal pixel coordinate. */
        cxPx = declare_parameter<double>("cx_px", 512.0);

        /* Principal-point vertical pixel coordinate. */
        cyPx = declare_parameter<double>("cy_px", 512.0);

        /* Physical distance between the left and right LocCam sensors. */
        baselineM = declare_parameter<double>("stereo_baseline_m", 0.15);

        /* Reconstructed features beyond this depth are discarded as
         * unreliable. */
        maximumDepthM = declare_parameter<double>("maximum_depth_m", 25.0);

        /* Old image pairs are discarded before expensive conversion. */
        maximumInputAgeS =
            declare_parameter<double>("maximum_input_age_s", 0.25);

        /* Long keyframe gaps are declared unavailable, not silently bridged. */
        maximumKeyframeIntervalS =
            declare_parameter<double>("maximum_keyframe_interval_s", 0.75);
        if (maximumInputAgeS <= 0.0 || maximumKeyframeIntervalS <= 0.0)
        {
            throw std::invalid_argument(
                "visual age and keyframe interval limits must be positive");
        }

        /* Maximum number of corner features detected per frame. */
        maximumFeatures = declare_parameter<int>("maximum_features", 640);

        /* Minimum stereo/temporal correspondences required to attempt
         * PnP. */
        minimumCorrespondences =
            declare_parameter<int>("minimum_correspondences", 20);

        /* Fixed LocCam resolution both feature-tracking engines below are
         * sized for; must match model.sdf's configured LocCam sensor
         * resolution. */
        const int imageWidthPx = declare_parameter<int>("image_width_px", 1024);
        const int imageHeightPx =
            declare_parameter<int>("image_height_px", 1024);

        visualQualityConfiguration.imageWidthPx  = imageWidthPx;
        visualQualityConfiguration.imageHeightPx = imageHeightPx;
        visualQualityConfiguration.occupancyGridRows =
            declare_parameter<int>("feature_grid_rows", 4);
        visualQualityConfiguration.occupancyGridColumns =
            declare_parameter<int>("feature_grid_columns", 4);
        maximumFeaturesPerCell =
            declare_parameter<int>("maximum_features_per_cell", 40);
        visualQualityConfiguration.nearMidDepthM =
            declare_parameter<double>("near_mid_depth_m", 12.0);
        visualQualityConfiguration.minimumCoverageRatio =
            declare_parameter<double>("minimum_inlier_coverage_ratio", 0.25);
        visualQualityConfiguration.minimumNearMidRatio =
            declare_parameter<double>("minimum_near_mid_inlier_ratio", 0.25);
        visualQualityConfiguration.preferredDisparityPx =
            declare_parameter<double>("preferred_disparity_px", 8.0);
        visualQualityConfiguration.disparityStddevPx =
            declare_parameter<double>("disparity_stddev_px", 1.0);
        visualQualityConfiguration.reprojectionNoiseFloorPx =
            declare_parameter<double>("reprojection_noise_floor_px", 0.25);
        visualQualityConfiguration.maximumNormalCondition =
            declare_parameter<double>("maximum_pnp_normal_condition", 1.0e12);
        visualQualityConfiguration.minimumTranslationVarianceM2 =
            declare_parameter<double>("minimum_translation_variance_m2",
                                      1.0e-5);
        visualQualityConfiguration.maximumTranslationVarianceM2 =
            declare_parameter<double>("maximum_translation_variance_m2", 4.0);
        visualQualityConfiguration.minimumRotationVarianceRad2 =
            declare_parameter<double>("minimum_rotation_variance_rad2", 1.0e-6);
        visualQualityConfiguration.maximumRotationVarianceRad2 =
            declare_parameter<double>("maximum_rotation_variance_rad2", 0.25);
        visualQualityConfiguration.fxPx      = fxPx;
        visualQualityConfiguration.baselineM = baselineM;

        if (imageWidthPx <= 0 || imageHeightPx <= 0 ||
            visualQualityConfiguration.occupancyGridRows <= 0 ||
            visualQualityConfiguration.occupancyGridColumns <= 0 ||
            maximumFeaturesPerCell <= 0 ||
            !(visualQualityConfiguration.nearMidDepthM > 0.0) ||
            !(visualQualityConfiguration.minimumCoverageRatio > 0.0) ||
            !(visualQualityConfiguration.minimumCoverageRatio <= 1.0) ||
            !(visualQualityConfiguration.minimumNearMidRatio > 0.0) ||
            !(visualQualityConfiguration.minimumNearMidRatio <= 1.0) ||
            !(visualQualityConfiguration.preferredDisparityPx > 0.0) ||
            !(visualQualityConfiguration.disparityStddevPx > 0.0) ||
            !(visualQualityConfiguration.reprojectionNoiseFloorPx > 0.0) ||
            !(visualQualityConfiguration.maximumNormalCondition > 1.0) ||
            !(visualQualityConfiguration.minimumTranslationVarianceM2 > 0.0) ||
            !(visualQualityConfiguration.maximumTranslationVarianceM2 >=
              visualQualityConfiguration.minimumTranslationVarianceM2) ||
            !(visualQualityConfiguration.minimumRotationVarianceRad2 > 0.0) ||
            !(visualQualityConfiguration.maximumRotationVarianceRad2 >=
              visualQualityConfiguration.minimumRotationVarianceRad2))
        {
            throw std::invalid_argument(
                "visual geometry and covariance parameters are invalid");
        }

        /* Corner-detector acceptance threshold, as a fraction of the
         * frame's strongest response. */
        const float featureQualityLevel = static_cast<float>(
            declare_parameter<double>("feature_quality_level", 0.01));

        /* Minimum enforced pixel separation between accepted corners. */
        const float featureMinimumDistancePx = static_cast<float>(
            declare_parameter<double>("feature_minimum_distance_px", 8.0));

        /* Coarsest pyramid level used by optical-flow tracking. */
        const int opticalFlowMaximumPyramidLevel =
            declare_parameter<int>("optical_flow_maximum_pyramid_level", 3);

        /* Side length, in pixels, of the optical-flow tracking window. */
        const int opticalFlowWindowSizePx =
            declare_parameter<int>("optical_flow_window_size_px", 21);

        /* Upper bound on Gauss-Newton refinement iterations per feature
         * per pyramid level. */
        const int opticalFlowMaximumIterations =
            declare_parameter<int>("optical_flow_maximum_iterations", 30);

        /* Convergence threshold on the per-iteration displacement update,
         * in pixels. */
        const float opticalFlowEpsilonPx = static_cast<float>(
            declare_parameter<double>("optical_flow_epsilon_px", 0.01));

        /* Minimum acceptable structure-tensor eigenvalue below which an
         * optical-flow window is treated as too low-texture to track. */
        const float opticalFlowMinimumEigenvalueThreshold =
            static_cast<float>(declare_parameter<double>(
                "optical_flow_minimum_eigenvalue_threshold",
                1.0e-4));

        /* Read the raw disparity-range parameter before rounding it below. */
        int numberOfDisparities =
            declare_parameter<int>("num_disparities", 128);

        /* StereoBM requires the disparity range to be a positive multiple
         * of 16; round the configured value up to satisfy that. */
        numberOfDisparities =
            std::max(16, ((numberOfDisparities + 15) / 16) * 16);

        /* Read the raw block-matching window size before rounding it
         * below. */
        int blockSize = declare_parameter<int>("block_size", 15);

        /* StereoBM requires an odd block size of at least 5; round the
         * configured value up to satisfy that. */
        blockSize = std::max(5, blockSize | 1);

        /* Camera mount x offset from the body origin, in metres. */
        const double cameraXM = declare_parameter<double>("camera_x_m", 0.932);

        /* Camera mount z offset from the body origin, in metres. */
        const double cameraZM = declare_parameter<double>("camera_z_m", 0.60);

        /* Camera mount pitch about the body y axis, in radians. */
        const double cameraPitchRad =
            declare_parameter<double>("camera_pitch_rad", 0.174533);

        /* Derive the fixed camera-to-body transform from the mount
         * geometry just declared. */
        configureCameraTransform(cameraXM, cameraZM, cameraPitchRad);

        /* Build the pinhole intrinsic matrix shared by both LocCam
         * sensors. */
        cameraMatrix = (cv::Mat_<double>(3, 3) << fxPx,
                        0.0,
                        cxPx,
                        0.0,
                        fyPx,
                        cyPx,
                        0.0,
                        0.0,
                        1.0);

        /* Construct the block-matching stereo disparity estimator with the
         * validated disparity range and block size. */
        p_stereoMatcher = cv::StereoBM::create(numberOfDisparities, blockSize);

        /* Reject low-texture regions that would otherwise match
         * unreliably. */
        p_stereoMatcher->setTextureThreshold(5);

        /* Require a match to be clearly better than its runner-up before
         * it is accepted. */
        p_stereoMatcher->setUniquenessRatio(8);

        /* Suppress small, likely-spurious groups of connected disparity
         * values. */
        p_stereoMatcher->setSpeckleWindowSize(50);

        /* Maximum disparity variation allowed within one speckle group. */
        p_stereoMatcher->setSpeckleRange(2);

        /*!
         * Initialize both reusable feature-tracking engines once, here,
         * at construction time -- matching their own documented
         * lifecycle (the one allocation each ever performs) and this
         * node's single-threaded executor assumption. A misconfiguration
         * here (e.g. `image_width_px` not matching model.sdf, or an
         * optical-flow window too large for the configured pyramid
         * depth) is a startup-time defect, so it fails the node's
         * construction outright rather than degrading silently at
         * runtime; see `docs/compliance/feature_tracking/` for the
         * applicable coding profile.
         */
        const feature_tracking::FeatureTrackingStatus cornerDetectorStatus =
            cornerDetector.initialize(imageWidthPx,
                                      imageHeightPx,
                                      static_cast<std::size_t>(maximumFeatures),
                                      featureQualityLevel,
                                      featureMinimumDistancePx);
        if (cornerDetectorStatus != feature_tracking::FeatureTrackingStatus::
                                        FEATURE_TRACKING_STATUS_SUCCESS)
        {
            throw std::runtime_error(
                "ShiTomasiCornerDetector initialization failed");
        }

        const feature_tracking::FeatureTrackingStatus opticalFlowStatus =
            opticalFlowTracker.initialize(
                imageWidthPx,
                imageHeightPx,
                opticalFlowMaximumPyramidLevel,
                opticalFlowWindowSizePx,
                opticalFlowMaximumIterations,
                opticalFlowEpsilonPx,
                opticalFlowMinimumEigenvalueThreshold);
        if (opticalFlowStatus != feature_tracking::FeatureTrackingStatus::
                                     FEATURE_TRACKING_STATUS_SUCCESS)
        {
            throw std::runtime_error(
                "PyramidalLucasKanadeTracker initialization failed");
        }

        /* Publisher for accumulated visual odometry. */
        p_odometryPublisher =
            create_publisher<nav_msgs::msg::Odometry>(outputTopic,
                                                      rclcpp::QoS(10));

        /* Publisher for the sparse inlier feature point cloud. */
        p_pointCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic,
            rclcpp::QoS(10).reliable());

        /* Publisher for the annotated feature/track visualization. */
        p_featureImagePublisher = create_publisher<sensor_msgs::msg::Image>(
            featureImageTopic,
            rclcpp::QoS(10).reliable());

        p_resetSubscription = create_subscription<std_msgs::msg::Empty>(
            resetTopic,
            rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Empty::ConstSharedPtr p_message)
            { handleResetCallBack(*p_message); });

        /* Feed the left LocCam stream into the stereo synchronizer. */
        const rmw_qos_profile_t latestSensorQos =
            rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile();
        leftSubscriber.subscribe(this, leftTopic, latestSensorQos);

        /* Feed the right LocCam stream into the stereo synchronizer. */
        rightSubscriber.subscribe(this, rightTopic, latestSensorQos);

        /* Pair left/right frames whose timestamps fall within a small
         * tolerance of each other. */
        p_synchronizer =
            std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
                StereoPolicy(1),
                leftSubscriber,
                rightSubscriber);

        /* Every synchronized stereo pair triggers handleStereoCallBack(). A
         * lambda does not satisfy message_filters::Synchronizer's own
         * template-deduced callback-signature matching here; std::bind
         * is kept deliberately. */
        p_synchronizer->registerCallback(
            // NOLINTNEXTLINE(modernize-avoid-bind)
            std::bind(&VisualOdometryNode::handleStereoCallBack,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        /* Emit bounded stage/rate diagnostics outside expensive processing. */
        diagnosticsStartTime = std::chrono::steady_clock::now();
        p_diagnosticsTimer =
            create_wall_timer(std::chrono::seconds(5),
                              [this]() { logPipelineDiagnostics(); });

        /* Record the resolved topic names once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(),
                    "Stereo visual odometry: [%s, %s] -> [%s, %s, %s]",
                    leftTopic.c_str(),
                    rightTopic.c_str(),
                    outputTopic.c_str(),
                    pointCloudTopic.c_str(),
                    featureImageTopic.c_str());
    }

    /*!
     * @brief           Terminates both owned feature-tracking engines and
     *                  logs a failure if that lifecycle transition is
     *                  rejected.
     */
    // NOLINTNEXTLINE(readability-function-cognitive-complexity)
    ~VisualOdometryNode() noexcept override
    {
        /* Return each owned engine to its uninitialized lifecycle state;
         * termination only fails if an engine was never successfully
         * initialized in the first place, which cannot happen once the
         * constructor above has completed, but is still reported (not
         * ignored) rather than assumed. */
        const feature_tracking::FeatureTrackingStatus cornerDetectorStatus =
            cornerDetector.terminate();
        if (cornerDetectorStatus != feature_tracking::FeatureTrackingStatus::
                                        FEATURE_TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(),
                         "ShiTomasiCornerDetector termination failed");
        }

        const feature_tracking::FeatureTrackingStatus opticalFlowStatus =
            opticalFlowTracker.terminate();
        if (opticalFlowStatus != feature_tracking::FeatureTrackingStatus::
                                     FEATURE_TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(),
                         "PyramidalLucasKanadeTracker termination failed");
        }
    }

    VisualOdometryNode(const VisualOdometryNode &otherNode_in) = delete;
    VisualOdometryNode &
        operator=(const VisualOdometryNode &otherNode_in)            = delete;
    VisualOdometryNode(VisualOdometryNode &&otherNode_in)            = delete;
    VisualOdometryNode &operator=(VisualOdometryNode &&otherNode_in) = delete;

    /**
     * @brief           Selects keyframe handling for one processed frame.
     *
     * @param[in]       estimationSucceeded_in
     *                  Whether this frame produced an accepted PnP estimate.
     * @param[in]       isPoseAvailable_in
     *                  Whether pose continuity was available before the frame.
     * @param[in]       keyframeIntervalS_in
     *                  Interval from the retained accepted keyframe in seconds.
     * @param[in]       maximumKeyframeIntervalS_in
     *                  Largest interval that may be bridged continuously.
     * @return          Retain, advance, or mark-unavailable action.
     */
    static KeyframeAction
        selectKeyframeAction(bool   estimationSucceeded_in,
                             bool   isPoseAvailable_in,
                             double keyframeIntervalS_in,
                             double maximumKeyframeIntervalS_in);

    /** @brief Returns the fixed optical-to-body transform for a camera mount.
     */
    static cv::Matx44d calculateBodyFromOptical(double cameraXM_in,
                                                double cameraZM_in,
                                                double cameraPitchRad_in);

    /** @brief Evaluates PnP geometry and estimates its local pose covariance.
     */
    static VisualPoseQuality calculateVisualPoseQuality(
        const std::vector<cv::Point3f>   &objectPoints_in,
        const std::vector<cv::Point2f>   &imagePoints_in,
        const std::vector<int>           &inlierIndices_in,
        const cv::Mat                    &rotationVector_in,
        const cv::Mat                    &translationVector_in,
        const cv::Mat                    &cameraMatrix_in,
        const VisualQualityConfiguration &configuration_in);

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Processes one synchronized stereo LocCam pair.
     *
     * Runs the full per-frame pipeline: convert both images to OpenCV
     * matrices, detect corner features in the current left image (published
     * for the next frame's tracking and for visualization), compute
     * disparity on the previous stereo pair, track the previous frame's
     * features into the current frame with pyramidal Lucas-Kanade optical
     * flow, reconstruct each tracked feature's previous 3-D position from
     * disparity, and estimate the inter-frame transform with PnP RANSAC when
     * enough correspondences survive. Always republishes the feature-track
     * visualization and stores the current frame as "previous" for the next
     * callback.
     *
     * @param[in]       p_left_in
     *                  Left LocCam image, synchronized with `p_right_in`.
     * @param[in]       p_right_in
     *                  Right LocCam image, synchronized with `p_left_in`.
     */
    void handleStereoCallBack(
        const sensor_msgs::msg::Image::ConstSharedPtr &p_left_in,
        const sensor_msgs::msg::Image::ConstSharedPtr &p_right_in);

    /**
     * @brief           Clears retained visual history and begins a new epoch.
     */
    void handleResetCallBack(const std_msgs::msg::Empty &message_in);

    /*!
     * @brief           Logs visual publication, processing and fusion inputs.
     */
    void logPipelineDiagnostics();

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Converts a ROS timestamp to seconds.
     *
     * @param[in]       stamp_in
     *                  Timestamp to convert.
     *
     * @return          Timestamp in seconds.
     */
    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp_in);

    /*!
     * @brief           Multiplies two homogeneous 4x4 transforms.
     *
     * @param[in]       left_in
     *                  Left-hand transform.
     * @param[in]       right_in
     *                  Right-hand transform.
     *
     * @return          `left_in * right_in`.
     */
    static cv::Matx44d multiply(const cv::Matx44d &left_in,
                                const cv::Matx44d &right_in);

    /*!
     * @brief           Inverts a rigid-body homogeneous transform.
     *
     * Exploits the rigid-transform structure (orthonormal rotation block) so
     * the inverse rotation is the transpose and the inverse translation is
     * `-Rotation^T * translation`, avoiding a general 4x4 matrix inverse.
     *
     * @param[in]       transform_in
     *                  Rigid transform to invert.
     *
     * @return          Inverse rigid transform.
     */
    static cv::Matx44d invertRigid(const cv::Matx44d &transform_in);

    /*!
     * @brief           Derives the fixed camera-to-body transform.
     *
     * Builds the rotation from the optical frame (x right, y down, z forward)
     * through the camera mount frame to the Alpha body frame, then combines it
     * with the configured mount translation. The result seeds
     * `worldFromOptical` so accumulated pose starts aligned with the body
     * frame at node start-up.
     *
     * @param[in]       cameraXM_in
     *                  Camera mount x offset from the body origin, in metres.
     * @param[in]       cameraZM_in
     *                  Camera mount z offset from the body origin, in metres.
     * @param[in]       cameraPitchRad_in
     *                  Camera mount pitch about the body y axis, in radians.
     */
    void configureCameraTransform(double cameraXM_in,
                                  double cameraZM_in,
                                  double cameraPitchRad_in);

    /*!
     * @brief           Publishes an annotated feature/track visualization.
     *
     * Blue circles mark freshly detected corners with no track. When tracking
     * status is available, a yellow line marks the optical-flow displacement
     * from the previous to the current feature position and a green circle
     * marks the current position. Magenta rings mark features whose stereo
     * correspondence and temporal track both succeeded (PnP RANSAC input).
     *
     * @param[in]       header_in
     *                  Header copied onto the published image, matching the
     *                  source LocCam frame's stamp and frame id.
     * @param[in]       imageMono_in
     *                  Current left LocCam image, single-channel greyscale.
     * @param[in]       previousFeatures_in
     *                  Previous-frame feature pixel coordinates.
     * @param[in]       currentFeatures_in
     *                  Tracked current-frame pixel coordinates; empty when no
     *                  temporal track was attempted.
     * @param[in]       trackingStatus_in
     *                  Per-feature Lucas-Kanade track success flags, aligned
     *                  with `previousFeatures_in`; empty when no track was
     *                  attempted.
     * @param[in]       stereoCorrelations_in
     *                  Current-frame pixel coordinates of features that also
     *                  produced a valid stereo/PnP correspondence.
     */
    void publishFeatureImage(
        const std_msgs::msg::Header      &header_in,
        const cv::Mat                    &imageMono_in,
        const std::vector<cv::Point2f>   &previousFeatures_in,
        const std::vector<cv::Point2f>   &currentFeatures_in,
        const std::vector<unsigned char> &trackingStatus_in,
        const std::vector<cv::Point2f>   &stereoCorrelations_in);

    /*!
     * @brief           Applies an accepted PnP RANSAC pose estimate.
     *
     * Converts the PnP rotation vector to a rotation matrix, forms the
     * current-from-previous optical transform, right-multiplies it into the
     * accumulated `worldFromOptical`, and publishes the resulting odometry and
     * inlier point cloud.
     *
     * @param[in]       rotationVector_in
     *                  Rodrigues rotation vector from `solvePnPRansac`,
     *                  previous-to-current optical frame.
     * @param[in]       translationVector_in
     *                  Translation vector from `solvePnPRansac`, in metres,
     *                  previous-to-current optical frame.
     * @param[in]       stamp_in
     *                  Timestamp of the current stereo frame.
     * @param[in]       dtS_in
     *                  Elapsed time since the previous accepted frame, in
     *                  seconds.
     * @param[in]       correlatedPoints_in
     *                  Reconstructed previous-frame 3-D points in the
     *                  previous optical frame, indexed like
     *                  `inlierIndices_in`.
     * @param[in]       inlierIndices_in
     *                  Indices into `correlatedPoints_in` that PnP RANSAC
     *                  accepted as inliers.
     */
    void updatePose(const cv::Mat                       &rotationVector_in,
                    const cv::Mat                       &translationVector_in,
                    const builtin_interfaces::msg::Time &stamp_in,
                    double                               dtS_in,
                    const std::vector<cv::Point3f>      &correlatedPoints_in,
                    const std::vector<int>              &inlierIndices_in,
                    const VisualPoseQuality             &quality_in);

    /*!
     * @brief           Publishes inlier 3-D features as a sparse point cloud.
     *
     * Transforms each inlier point from the optical frame into `odomFrame`
     * using the supplied optical-to-map transform and packs the result into
     * an `xyz` `PointCloud2`. Publishes an empty cloud when there are no
     * inliers, so subscribers always receive a message for every processed
     * frame.
     *
     * @param[in]       stamp_in
     *                  Timestamp applied to the published cloud header.
     * @param[in]       correlatedPoints_in
     *                  Reconstructed 3-D points in the optical frame named by
     *                  `mapFromOptical_in`'s source, indexed like
     *                  `inlierIndices_in`.
     * @param[in]       inlierIndices_in
     *                  Indices into `correlatedPoints_in` to publish.
     * @param[in]       mapFromOptical_in
     *                  Rigid transform from the optical frame of
     *                  `correlatedPoints_in` into `odomFrame`.
     */
    void publishPointCloud(const builtin_interfaces::msg::Time &stamp_in,
                           const std::vector<cv::Point3f> &correlatedPoints_in,
                           const std::vector<int>         &inlierIndices_in,
                           const cv::Matx44d              &mapFromOptical_in);

    /*!
     * @brief           Converts a rotation matrix to a ROS quaternion.
     *
     * Uses the standard largest-diagonal-term branch selection for numerical
     * stability near all rotation angles.
     *
     * @param[in]       transform_in
     *                  Homogeneous transform whose upper-left 3x3 block is
     *                  the rotation to convert.
     *
     * @return          Equivalent unit quaternion.
     */
    static geometry_msgs::msg::Quaternion
        rotationToQuaternion(const cv::Matx44d &transform_in);

    /*!
     * @brief           Publishes one odometry message.
     *
     * Pose is the accumulated `currentPose_in` expressed in `odomFrame`.
     * Twist is the finite-difference body-frame relative motion between
     * `previousPose_in` and `currentPose_in` divided by `dtS_in`. Diagonal
     * pose covariance is the accumulated geometry-aware covariance supplied by
     * `updatePose`; diagnostic twist covariance is derived from the current
     * relative solve only.
     *
     * @param[in]       stamp_in
     *                  Timestamp applied to the published message header.
     * @param[in]       dtS_in
     *                  Elapsed time between `previousPose_in` and
     *                  `currentPose_in`, in seconds.
     * @param[in]       poseCovariance_in
     *                  Accumulated pose covariance in the fixed frame.
     * @param[in]       relativeCovariance_in
     *                  Current relative-pose covariance used for diagnostic
     *                  finite-difference twist covariance.
     * @param[in]       previousPose_in
     *                  Previous accumulated world-from-body transform.
     * @param[in]       currentPose_in
     *                  Current accumulated world-from-body transform.
     */
    void publishOdometry(const builtin_interfaces::msg::Time &stamp_in,
                         double                               dtS_in,
                         const cv::Matx44d                   &previousPose_in,
                         const cv::Matx44d                   &currentPose_in,
                         const PoseCovariance                &poseCovariance_in,
                         const PoseCovariance &relativeCovariance_in);

    /*!
     * @brief           Stores the current stereo frame as "previous".
     *
     * @param[in]       left_in
     *                  Current left LocCam image to retain.
     * @param[in]       right_in
     *                  Current right LocCam image to retain.
     * @param[in]       stamp_in
     *                  Current frame timestamp to retain.
     */
    void storePrevious(const cv::Mat                       &left_in,
                       const cv::Mat                       &right_in,
                       const builtin_interfaces::msg::Time &stamp_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Maximum Lucas-Kanade tracking error, in pixels, before a
     *              feature is treated as a lost/mismatched track rather
     *              than a genuine correspondence (see `handleStereoCallBack`).
     */
    static constexpr float MAXIMUM_TRACKING_ERROR_PX = 30.0F;

    /*!
     * @brief       Maximum reprojection error, in pixels, `solvePnPRansac`
     *              may accept when classifying a correspondence as an
     *              inlier (see `handleStereoCallBack`).
     */
    static constexpr double PNP_RANSAC_REPROJECTION_ERROR_PX = 2.5;

    /*!
     * @brief       RANSAC confidence probability passed to
     *              `solvePnPRansac` (see `handleStereoCallBack`).
     */
    static constexpr double PNP_RANSAC_CONFIDENCE = 0.99;

    /*!
     * @brief       Maximum RANSAC iterations passed to `solvePnPRansac`
     *              (see `handleStereoCallBack`).
     */
    static constexpr int PNP_RANSAC_MAXIMUM_ITERATIONS = 100;

    /*!
     * @brief       Publishes accumulated visual odometry.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odometryPublisher;

    /*!
     * @brief       Publishes the sparse inlier feature point cloud.
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        p_pointCloudPublisher;

    /*!
     * @brief       Publishes the annotated feature/track visualization image.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        p_featureImagePublisher;

    /** @brief Receives explicit visual-epoch reset requests. */
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr p_resetSubscription;

    /*!
     * @brief       Left LocCam image subscriber feeding the stereo
     *              synchronizer.
     */
    message_filters::Subscriber<sensor_msgs::msg::Image> leftSubscriber;

    /*!
     * @brief       Right LocCam image subscriber feeding the stereo
     *              synchronizer.
     */
    message_filters::Subscriber<sensor_msgs::msg::Image> rightSubscriber;

    /*!
     * @brief       Approximate-time synchronizer pairing left and right LocCam
     *              frames.
     */
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> p_synchronizer;

    /*!
     * @brief       Periodically reports bounded pipeline diagnostics.
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;

    /*!
     * @brief       Owned block-matching stereo disparity estimator.
     */
    cv::Ptr<cv::StereoBM> p_stereoMatcher;

    /*!
     * @brief       Owned, reusable Shi-Tomasi corner detector, initialized
     *              once at construction for the fixed LocCam resolution
     *              (see `docs/compliance/feature_tracking/`).
     */
    feature_tracking::ShiTomasiCornerDetector cornerDetector;

    /*!
     * @brief       Owned, reusable pyramidal Lucas-Kanade optical-flow
     *              tracker, initialized once at construction for the
     *              fixed LocCam resolution (see
     *              `docs/compliance/feature_tracking/`).
     */
    feature_tracking::PyramidalLucasKanadeTracker opticalFlowTracker;

    /*!
     * @brief       Pinhole camera intrinsic matrix shared by both LocCam
     *              sensors.
     */
    cv::Mat cameraMatrix;

    /*!
     * @brief       Previous-frame left LocCam image retained for the next
     *              callback.
     */
    cv::Mat previousLeft;

    /*!
     * @brief       Previous-frame right LocCam image retained for the next
     *              callback.
     */
    cv::Mat previousRight;

    /*!
     * @brief       Fixed rigid transform from the camera optical frame to the
     *              body frame.
     */
    cv::Matx44d bodyFromOptical{cv::Matx44d::eye()};

    /*!
     * @brief       Accumulated rigid transform from the start-up optical
     *              frame to the current optical frame.
     */
    cv::Matx44d worldFromOptical{cv::Matx44d::eye()};

    /** @brief Accumulated fixed-frame body-pose covariance. */
    PoseCovariance accumulatedPoseCovariance{PoseCovariance::zeros()};

    /** @brief Geometry-quality thresholds for visual covariance. */
    VisualQualityConfiguration visualQualityConfiguration;

    /*!
     * @brief       Fixed frame in which published odometry and point clouds are
     *              expressed.
     */
    std::string odomFrame;

    /*!
     * @brief       Child body frame published in odometry messages.
     */
    std::string baseFrame;

    /*!
     * @brief       Horizontal focal length in pixels.
     */
    double fxPx{800.0};

    /*!
     * @brief       Vertical focal length in pixels.
     */
    double fyPx{800.0};

    /*!
     * @brief       Principal-point horizontal pixel coordinate.
     */
    double cxPx{512.0};

    /*!
     * @brief       Principal-point vertical pixel coordinate.
     */
    double cyPx{512.0};

    /*!
     * @brief       Stereo baseline distance between LocCam sensors, in metres.
     */
    double baselineM{0.15};

    /*!
     * @brief       Maximum accepted reconstructed feature depth, in metres.
     */
    double maximumDepthM{25.0};

    /** @brief Maximum image age admitted for processing, in seconds. */
    double maximumInputAgeS{0.25};

    /** @brief Maximum recoverable accepted-keyframe gap, in seconds. */
    double maximumKeyframeIntervalS{0.75};

    /*!
     * @brief       Timestamp in seconds of the retained previous stereo frame.
     */
    double previousStampS{0.0};

    /*!
     * @brief       Maximum number of corner features detected per frame.
     */
    int maximumFeatures{640};

    /*!
     * @brief       Minimum stereo/temporal correspondences required to attempt
     *              PnP.
     */
    int minimumCorrespondences{20};

    /** @brief Maximum reconstructed correspondences retained per image cell. */
    int maximumFeaturesPerCell{40};

    /** @brief Whether accumulated visual pose remains continuous. */
    bool isVisualPoseAvailable{true};

    /** @brief Steady-clock origin for diagnostic rates. */
    std::chrono::steady_clock::time_point diagnosticsStartTime;

    /** @brief Number of synchronized pairs admitted. */
    std::uint64_t receivedPairCount{0U};

    /** @brief Number of accepted visual pose updates. */
    std::uint64_t acceptedPoseCount{0U};

    /** @brief Number of callbacks that failed to produce a pose. */
    std::uint64_t failedPoseCount{0U};

    /** @brief Pair count at the previous diagnostic report. */
    std::uint64_t previousReportedPairCount{0U};

    /** @brief Accepted count at the previous diagnostic report. */
    std::uint64_t previousReportedAcceptedCount{0U};

    /** @brief Consecutive callbacks without an accepted pose. */
    std::uint64_t consecutiveFailureCount{0U};

    /** @brief Latest PnP input correspondence count. */
    std::size_t latestCorrespondenceCount{0U};

    /** @brief Latest accepted PnP inlier count. */
    std::size_t latestInlierCount{0U};

    /** @brief Latest current-frame detected feature count. */
    std::size_t latestDetectedCount{0U};

    /** @brief Latest successfully tracked feature count. */
    std::size_t latestTrackedCount{0U};

    /** @brief Latest valid stereo reconstruction count before PnP. */
    std::size_t latestStereoValidCount{0U};

    /** @brief Geometry quality for the latest accepted PnP solve. */
    VisualPoseQuality latestVisualQuality;

    /** @brief Latest accepted keyframe interval in seconds. */
    double latestAcceptedInterval_s{0.0};

    /** @brief Latest image age at callback admission in seconds. */
    double latestAdmissionAge_s{0.0};

    /** @brief Latest full callback duration in milliseconds. */
    double latestProcessingDuration_ms{0.0};

    /** @brief Latest image-conversion duration in milliseconds. */
    double latestConversionDuration_ms{0.0};

    /** @brief Latest disparity duration in milliseconds. */
    double latestDisparityDuration_ms{0.0};

    /** @brief Latest corner-detection duration in milliseconds. */
    double latestDetectionDuration_ms{0.0};

    /** @brief Latest feature-tracking duration in milliseconds. */
    double latestTrackingDuration_ms{0.0};

    /** @brief Latest reconstruction duration in milliseconds. */
    double latestReconstructionDuration_ms{0.0};

    /** @brief Latest PnP duration in milliseconds. */
    double latestPnpDuration_ms{0.0};
};

} /* namespace localisation::visual_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_VISUAL_ODOMETRY_NODE_H */
