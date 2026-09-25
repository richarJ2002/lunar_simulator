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
#include "console/console.h"

/* Object Include */
#include "feature_tracking/objects/FeatureTrackingStatus.h"
#include "feature_tracking/objects/ImageView.h"
#include "feature_tracking/objects/Point2D.h"
#include "feature_tracking/objects/PyramidalLucasKanadeTracker.h"
#include "feature_tracking/objects/ShiTomasiCornerDetector.h"

/* Data include */
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
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
 * The estimator matches the keyframe's corners along their rows in the
 * right image once, when the keyframe is stored (or, with stereo_matching
 * "block", reads a dense `StereoBM` map of the keyframe pair), tracks those
 * corners into later LocCam frames with pyramidal Lucas-Kanade optical
 * flow, reconstructs their 3-D positions from disparity, and estimates the
 * keyframe-to-frame rigid transform with PnP RANSAC. The keyframe is kept
 * until the increment spans enough parallax (see KeyframePolicy). Estimated
 * pose is accumulated into `worldFromOptical`, a rigid
 * transform from the optical frame at node start-up to the current optical
 * frame. Odometry withholds a pose update and keeps the previous accumulated
 * pose whenever too few reliable correspondences are available; it is
 * intended for a single-threaded executor.
 */
class VisualOdometryNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Action applied to the retained stereo keyframe after
     *                  a frame.
     */
    enum class KeyframeAction : std::uint8_t
    {
        /*! Keep the keyframe so the next solve spans all motion since it. */
        RETAIN_KEYFRAME = 0U,

        /*! Replace the keyframe with the current frame. */
        ADVANCE_KEYFRAME = 1U,

        /*! Declare the accumulated pose discontinuous. */
        MARK_POSE_UNAVAILABLE = 2U
    };

    /*!
     * @brief           Six-dimensional covariance ordered translation then
     *                  rotation.
     */
    using PoseCovariance = cv::Matx<double, 6, 6>;

    /*!
     * @brief           Tunable thresholds used to assess one PnP solution;
     *                  each field mirrors the ROS parameter of the same
     *                  meaning declared in the constructor.
     */
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

    /*!
     * @brief           Geometry diagnostics and covariance for one accepted
     *                  PnP solve.
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
     * @brief           When an accepted estimate may keep the keyframe
     *                  instead of advancing it.
     *
     *                  Holding the keyframe until features have moved far
     *                  enough lets each published increment span more
     *                  parallax. Thresholds of zero advance on every
     *                  accepted estimate.
     */
    struct KeyframePolicy
    {
        /*!
         * @brief           Median tracked-feature image motion from the
         *                  keyframe at which it advances.
         *
         * @frame           Image
         * @units           pixels
         */
        double minimumParallaxPx{3.0};

        /*!
         * @brief           Fraction of keyframe corners still tracked below
         *                  which it advances, before tracking thins out.
         *
         * @frame           N/A
         * @units           ratio
         */
        double minimumSurvivalRatio{0.5};

        /*!
         * @brief           Longest time an accepted estimate may keep the
         *                  keyframe; at most maximum_keyframe_interval_s.
         *
         * @frame           N/A
         * @units           seconds
         */
        double maximumRetentionS{0.5};
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

        /* Output topic: periodic pipeline diagnostics, shared by every node
         * of the owning system and recorded with every run. */
        const std::string diagnosticsTopic =
            declare_parameter<std::string>("diagnostics_topic",
                                           "/" + systemName + "/diagnostics");

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

        /* Parallax keyframes (WP-01 Phase 5.2): an accepted estimate keeps
         * the keyframe until its features have moved far enough, too few
         * survive, or it has been kept for the retention limit. The
         * retention limit stays within the bridging limit so a kept
         * keyframe is always advanced before a failure could invalidate
         * the pose. */
        keyframePolicy.minimumParallaxPx =
            declare_parameter<double>("keyframe_minimum_parallax_px", 3.0);
        keyframePolicy.minimumSurvivalRatio =
            declare_parameter<double>("keyframe_minimum_survival_ratio", 0.5);
        keyframePolicy.maximumRetentionS =
            declare_parameter<double>("keyframe_maximum_retention_s", 0.5);
        if (!(keyframePolicy.minimumParallaxPx >= 0.0) ||
            !(keyframePolicy.minimumSurvivalRatio >= 0.0) ||
            keyframePolicy.minimumSurvivalRatio > 1.0 ||
            !(keyframePolicy.maximumRetentionS > 0.0) ||
            keyframePolicy.maximumRetentionS > maximumKeyframeIntervalS)
        {
            throw std::invalid_argument("keyframe policy limits are invalid");
        }

        /* Readiness reported to the start-up supervisor: a run of
         * consecutive accepted poses proves tracking has started, and a
         * long gap without one withdraws it. */
        const std::int64_t configuredReadinessPoses =
            declare_parameter<std::int64_t>("readiness_consecutive_poses", 3);
        readinessMaximumPoseGapS =
            declare_parameter<double>("readiness_maximum_pose_gap_s", 3.0);
        if (configuredReadinessPoses <= 0 || !(readinessMaximumPoseGapS > 0.0))
        {
            throw std::invalid_argument(
                "visual readiness limits must be positive");
        }
        readinessConsecutivePoses =
            static_cast<std::uint64_t>(configuredReadinessPoses);

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

        /* Stereo depth source (WP-01 Phase 5.3): "block" reads the dense
         * StereoBM map, recomputed on the keyframe pair every frame, at
         * the nearest pixel; "sparse" matches each keyframe corner once,
         * when the keyframe is stored, along its row to sub-pixel
         * precision with a left-right consistency check. Both search
         * num_disparities pixels. */
        const std::string stereoMatching =
            declare_parameter<std::string>("stereo_matching", "sparse");
        if (stereoMatching != "block" && stereoMatching != "sparse")
        {
            throw std::invalid_argument(
                "stereo_matching must be block or sparse");
        }
        isSparseStereo                 = stereoMatching == "sparse";
        sparseStereoMaximumDisparityPx = numberOfDisparities;
        sparseStereoHalfWindowPx =
            declare_parameter<int>("sparse_stereo_half_window_px", 5);
        sparseStereoMaximumLeftRightDifferencePx = declare_parameter<double>(
            "sparse_stereo_maximum_left_right_difference_px",
            1.0);
        if (sparseStereoHalfWindowPx < 2 || sparseStereoHalfWindowPx > 15 ||
            !(sparseStereoMaximumLeftRightDifferencePx > 0.0))
        {
            throw std::invalid_argument("sparse stereo limits are invalid");
        }

        /* Rows above this are not searched for corners, so the feature
         * budget and the detector's relative quality threshold go to the
         * near ground instead of sky and distant terrain. Zero searches
         * the whole image. */
        detectionMinimumRowPx =
            declare_parameter<int>("detection_minimum_row_px", 0);
        if (detectionMinimumRowPx < 0 ||
            detectionMinimumRowPx > imageHeightPx - 64)
        {
            throw std::invalid_argument(
                "detection_minimum_row_px must leave at least 64 rows");
        }

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
                                      imageHeightPx - detectionMinimumRowPx,
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

        /* Publish bounded stage/rate diagnostics outside expensive
         * processing, once per simulated second, so their stamps and rates
         * share the clock of every other recorded topic. */
        p_diagnosticsPublisher =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnosticsTopic,
                rclcpp::QoS(10));
        p_diagnosticsTimer =
            create_timer(std::chrono::seconds(1),
                         [this]() { publishPipelineDiagnosticsCallBack(); });

        /* Topic wiring is already captured by the run's parameter snapshot,
         * so it is debug detail rather than operator output. */
        LUNAR_LOG_DEBUG(get_logger(),
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
            LUNAR_LOG_ERROR(get_logger(), "Corner detector termination failed");
        }

        const feature_tracking::FeatureTrackingStatus opticalFlowStatus =
            opticalFlowTracker.terminate();
        if (opticalFlowStatus != feature_tracking::FeatureTrackingStatus::
                                     FEATURE_TRACKING_STATUS_SUCCESS)
        {
            LUNAR_LOG_ERROR(get_logger(), "LK tracker termination failed");
        }
    }

    VisualOdometryNode(const VisualOdometryNode &otherNode_in) = delete;
    VisualOdometryNode &
        operator=(const VisualOdometryNode &otherNode_in)            = delete;
    VisualOdometryNode(VisualOdometryNode &&otherNode_in)            = delete;
    VisualOdometryNode &operator=(VisualOdometryNode &&otherNode_in) = delete;

    /*!
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
     * @param[in]       medianParallaxPx_in
     *                  Median image motion of the tracked keyframe corners
     *                  in pixels; non-finite when nothing was tracked.
     * @param[in]       survivalRatio_in
     *                  Fraction of keyframe corners tracked into this frame.
     * @param[in]       policy_in
     *                  When an accepted estimate may keep the keyframe.
     * @return          Retain, advance, or mark-unavailable action. An
     *                  accepted estimate returns RETAIN_KEYFRAME only while
     *                  it is below every policy threshold; the caller then
     *                  publishes nothing and solves against the same
     *                  keyframe next frame.
     */
    static KeyframeAction
        selectKeyframeAction(bool                  estimationSucceeded_in,
                             bool                  isPoseAvailable_in,
                             double                keyframeIntervalS_in,
                             double                maximumKeyframeIntervalS_in,
                             double                medianParallaxPx_in,
                             double                survivalRatio_in,
                             const KeyframePolicy &policy_in);

    /*!
     * @brief           Matches each corner of a left image along its row in
     *                  the right image to sub-pixel precision.
     *
     *                  A (2h+1)-square patch around each corner is compared
     *                  by sum of squared differences at every integer
     *                  disparity up to the search range, the minimum is
     *                  refined by a parabola through its neighbours, and the
     *                  right-image patch at the match must find its way back
     *                  to within the left-right tolerance. A match at the
     *                  edge of the search range or that is not clearly
     *                  better than the next-best non-adjacent disparity is
     *                  rejected.
     *
     * @param[in]       left_in
     *                  Rectified left image, mono8.
     * @param[in]       right_in
     *                  Rectified right image, mono8, same size.
     * @param[in]       corners_in
     *                  Corners detected on left_in.
     * @param[in]       cornerCount_in
     *                  Number of valid entries in corners_in.
     * @param[in]       maximumDisparityPx_in
     *                  Largest disparity searched, pixels.
     * @param[in]       halfWindowPx_in
     *                  Half-width of the square patch, pixels.
     * @param[in]       maximumLeftRightDifferencePx_in
     *                  Largest left-to-right versus right-to-left
     *                  disparity difference accepted, pixels.
     * @param[out]      disparityPx_out
     *                  Disparity of each corner in pixels, aligned with
     *                  corners_in; NaN where no reliable match was found.
     */
    static void calculateSparseDisparity(
        const cv::Mat &left_in,
        const cv::Mat &right_in,
        const std::array<feature_tracking::Point2D,
                         feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
                   &corners_in,
        std::size_t cornerCount_in,
        int         maximumDisparityPx_in,
        int         halfWindowPx_in,
        double      maximumLeftRightDifferencePx_in,
        std::array<float, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
            &disparityPx_out);

    /*!
     * @brief           Returns the fixed optical-to-body transform for a
     *                  camera mount.
     *
     * @param[in]       cameraXM_in
     *                  Camera mount x offset from the body origin, metres.
     *
     * @param[in]       cameraZM_in
     *                  Camera mount z offset from the body origin, metres.
     *
     * @param[in]       cameraPitchRad_in
     *                  Camera mount pitch about the body y axis, radians.
     *
     * @return          Rigid transform from the optical frame (x right,
     *                  y down, z forward) to the body frame.
     */
    static cv::Matx44d calculateBodyFromOptical(double cameraXM_in,
                                                double cameraZM_in,
                                                double cameraPitchRad_in);

    /*!
     * @brief           Evaluates PnP geometry and estimates its local pose
     *                  covariance.
     *
     * @param[in]       objectPoints_in
     *                  Reconstructed previous-frame points, optical frame,
     *                  metres.
     *
     * @param[in]       imagePoints_in
     *                  Tracked current-frame pixels aligned with
     *                  objectPoints_in.
     *
     * @param[in]       inlierIndices_in
     *                  PnP RANSAC inlier indices into both point lists.
     *
     * @param[in]       rotationVector_in
     *                  PnP Rodrigues rotation, previous to current optical.
     *
     * @param[in]       translationVector_in
     *                  PnP translation, previous to current optical, metres.
     *
     * @param[in]       cameraMatrix_in
     *                  Pinhole intrinsic matrix, pixels.
     *
     * @param[in]       configuration_in
     *                  Geometry-quality thresholds.
     *
     * @return          Quality diagnostics and the relative covariance;
     *                  isValid is false when a gate fails.
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

    /*!
     * @brief           Clears retained visual history and begins a new epoch.
     *
     * @param[in]       message_in
     *                  Reset request; it carries no fields.
     */
    void handleResetCallBack(const std_msgs::msg::Empty &message_in);

    /*!
     * @brief           Publishes one pipeline diagnostics record on the
     *                  diagnostics topic and, every fifth call, a compact
     *                  console health line.
     *
     *                  Runs from a simulation-time timer once per second.
     *                  Rates are computed over the simulated interval since
     *                  the previous record (and, for the console line,
     *                  since the previous console line).
     */
    void publishPipelineDiagnosticsCallBack();

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
     * @brief           Stores the current stereo frame and its detected
     *                  corners as the retained keyframe.
     *
     * @param[in]       left_in
     *                  Current left LocCam image to retain.
     *
     * @param[in]       right_in
     *                  Current right LocCam image to retain.
     *
     * @param[in]       stamp_in
     *                  Current frame timestamp to retain.
     *
     * @param[in]       corners_in
     *                  Corners detected on left_in; entries at and beyond
     *                  cornerCount_in are unused.
     *
     * @param[in]       cornerCount_in
     *                  Number of valid entries in corners_in.
     */
    void storePrevious(
        const cv::Mat                       &left_in,
        const cv::Mat                       &right_in,
        const builtin_interfaces::msg::Time &stamp_in,
        const std::array<feature_tracking::Point2D,
                         feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
                   &corners_in,
        std::size_t cornerCount_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Maximum Lucas-Kanade tracking error before a feature
     *                  is treated as a lost or mismatched track rather than
     *                  a genuine correspondence (see handleStereoCallBack).
     *
     * @frame           Image
     * @units           pixels (mean absolute intensity residual)
     */
    static constexpr float MAXIMUM_TRACKING_ERROR_PX = 30.0F;

    /*!
     * @brief           Maximum reprojection error solvePnPRansac may accept
     *                  when classifying a correspondence as an inlier.
     *
     * @frame           Image
     * @units           pixels
     */
    static constexpr double PNP_RANSAC_REPROJECTION_ERROR_PX = 2.5;

    /*!
     * @brief           RANSAC confidence probability passed to
     *                  solvePnPRansac.
     *
     * @frame           N/A
     * @units           probability
     */
    static constexpr double PNP_RANSAC_CONFIDENCE = 0.99;

    /*!
     * @brief           Maximum RANSAC iterations passed to solvePnPRansac.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr int PNP_RANSAC_MAXIMUM_ITERATIONS = 100;

    /*!
     * @brief           Number of diagnostics records between console health
     *                  lines: one line every five simulated seconds.
     *
     * @frame           N/A
     * @units           count
     */
    static constexpr std::uint64_t CONSOLE_HEALTH_PERIOD_TICKS = 5U;

    /*!
     * @brief           Publishes accumulated visual odometry.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odometryPublisher;

    /*!
     * @brief           Publishes the sparse inlier feature point cloud.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        p_pointCloudPublisher;

    /*!
     * @brief           Publishes the annotated feature/track visualization
     *                  image.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        p_featureImagePublisher;

    /*!
     * @brief           Publishes periodic pipeline diagnostics on the
     *                  system's shared diagnostics topic.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        p_diagnosticsPublisher;

    /*!
     * @brief           Receives explicit visual-epoch reset requests.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr p_resetSubscription;

    /*!
     * @brief           Left LocCam image subscriber feeding the stereo
     *                  synchronizer.
     *
     * @frame           N/A
     * @units           N/A
     */
    message_filters::Subscriber<sensor_msgs::msg::Image> leftSubscriber;

    /*!
     * @brief           Right LocCam image subscriber feeding the stereo
     *                  synchronizer.
     *
     * @frame           N/A
     * @units           N/A
     */
    message_filters::Subscriber<sensor_msgs::msg::Image> rightSubscriber;

    /*!
     * @brief           Approximate-time synchronizer pairing left and right
     *                  LocCam frames.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> p_synchronizer;

    /*!
     * @brief           Simulation-time timer driving
     *                  publishPipelineDiagnosticsCallBack() once per second.
     *
     * @frame           N/A
     * @units           N/A
     */
    rclcpp::TimerBase::SharedPtr p_diagnosticsTimer;

    /*!
     * @brief           Owned block-matching stereo disparity estimator.
     *
     * @frame           N/A
     * @units           N/A
     */
    cv::Ptr<cv::StereoBM> p_stereoMatcher;

    /*!
     * @brief           Owned, reusable Shi-Tomasi corner detector,
     *                  initialized once at construction for the fixed
     *                  LocCam resolution (see
     *                  docs/compliance/feature_tracking/).
     *
     * @frame           N/A
     * @units           N/A
     */
    feature_tracking::ShiTomasiCornerDetector cornerDetector;

    /*!
     * @brief           Owned, reusable pyramidal Lucas-Kanade optical-flow
     *                  tracker, initialized once at construction for the
     *                  fixed LocCam resolution (see
     *                  docs/compliance/feature_tracking/).
     *
     * @frame           N/A
     * @units           N/A
     */
    feature_tracking::PyramidalLucasKanadeTracker opticalFlowTracker;

    /*!
     * @brief           Pinhole camera intrinsic matrix shared by both LocCam
     *                  sensors.
     *
     * @frame           Optical
     * @units           pixels
     */
    cv::Mat cameraMatrix;

    /*!
     * @brief           Previous-frame left LocCam image retained for the
     *                  next callback.
     *
     * @frame           Image
     * @units           8-bit intensity
     */
    cv::Mat previousLeft;

    /*!
     * @brief           Previous-frame right LocCam image retained for the
     *                  next callback.
     *
     * @frame           Image
     * @units           8-bit intensity
     */
    cv::Mat previousRight;

    /*!
     * @brief           Corners detected on previousLeft when it was the
     *                  current frame; entries at and beyond
     *                  previousKeyframeCornerCount are unused.
     *
     * @frame           Image
     * @units           pixels
     */
    std::array<feature_tracking::Point2D,
               feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        previousKeyframeCorners{};

    /*!
     * @brief           Number of valid entries in previousKeyframeCorners.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t previousKeyframeCornerCount{0U};

    /*!
     * @brief           Sparse-stereo disparity of each keyframe corner,
     *                  aligned with previousKeyframeCorners; NaN where no
     *                  reliable match was found. Used only when
     *                  isSparseStereo.
     *
     * @frame           Image
     * @units           pixels
     */
    std::array<float, feature_tracking::MAXIMUM_SUPPORTED_FEATURES>
        previousKeyframeDisparityPx{};

    /*!
     * @brief           True to take depth from sparse keyframe-corner
     *                  matching instead of the dense StereoBM map.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isSparseStereo{true};

    /*!
     * @brief           Largest disparity the sparse matcher searches.
     *
     * @frame           Image
     * @units           pixels
     */
    int sparseStereoMaximumDisparityPx{128};

    /*!
     * @brief           Half-width of the sparse matcher's square patch.
     *
     * @frame           Image
     * @units           pixels
     */
    int sparseStereoHalfWindowPx{5};

    /*!
     * @brief           Largest left-to-right versus right-to-left disparity
     *                  difference a sparse match may have.
     *
     * @frame           Image
     * @units           pixels
     */
    double sparseStereoMaximumLeftRightDifferencePx{1.0};

    /*!
     * @brief           First image row searched for corners; the detector
     *                  is initialized for the band below it.
     *
     * @frame           Image
     * @units           pixels
     */
    int detectionMinimumRowPx{0};

    /*!
     * @brief           Fixed rigid transform from the camera optical frame
     *                  to the body frame.
     *
     * @frame           Optical to body
     * @units           metres (translation)
     */
    cv::Matx44d bodyFromOptical{cv::Matx44d::eye()};

    /*!
     * @brief           Accumulated rigid transform from the start-up optical
     *                  frame to the current optical frame.
     *
     * @frame           Current optical to startup-fixed
     * @units           metres (translation)
     */
    cv::Matx44d worldFromOptical{cv::Matx44d::eye()};

    /*!
     * @brief           Accumulated body-pose covariance, translation then
     *                  rotation.
     *
     * @frame           startup-fixed
     * @units           m^2 and rad^2
     */
    PoseCovariance accumulatedPoseCovariance{PoseCovariance::zeros()};

    /*!
     * @brief           Geometry-quality thresholds for visual covariance.
     *
     * @frame           N/A
     * @units           Mixed; see VisualQualityConfiguration
     */
    VisualQualityConfiguration visualQualityConfiguration;

    /*!
     * @brief           Fixed frame in which published odometry and point
     *                  clouds are expressed.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string odomFrame;

    /*!
     * @brief           Child body frame published in odometry messages.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::string baseFrame;

    /*!
     * @brief           Horizontal focal length.
     *
     * @frame           Image
     * @units           pixels
     */
    double fxPx{800.0};

    /*!
     * @brief           Vertical focal length.
     *
     * @frame           Image
     * @units           pixels
     */
    double fyPx{800.0};

    /*!
     * @brief           Principal-point horizontal coordinate.
     *
     * @frame           Image
     * @units           pixels
     */
    double cxPx{512.0};

    /*!
     * @brief           Principal-point vertical coordinate.
     *
     * @frame           Image
     * @units           pixels
     */
    double cyPx{512.0};

    /*!
     * @brief           Stereo baseline distance between the LocCam sensors.
     *
     * @frame           N/A
     * @units           metres
     */
    double baselineM{0.15};

    /*!
     * @brief           Maximum accepted reconstructed feature depth.
     *
     * @frame           Optical
     * @units           metres
     */
    double maximumDepthM{25.0};

    /*!
     * @brief           Maximum image age admitted for processing.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumInputAgeS{0.25};

    /*!
     * @brief           Maximum recoverable accepted-keyframe gap.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumKeyframeIntervalS{0.75};

    /*!
     * @brief           When an accepted estimate keeps the keyframe.
     *
     * @frame           N/A
     * @units           Mixed; see KeyframePolicy
     */
    KeyframePolicy keyframePolicy;

    /*!
     * @brief           Timestamp of the retained previous stereo frame.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double previousStampS{0.0};

    /*!
     * @brief           Maximum number of corner features detected per frame.
     *
     * @frame           N/A
     * @units           count
     */
    int maximumFeatures{640};

    /*!
     * @brief           Minimum stereo/temporal correspondences required to
     *                  attempt PnP.
     *
     * @frame           N/A
     * @units           count
     */
    int minimumCorrespondences{20};

    /*!
     * @brief           Maximum reconstructed correspondences retained per
     *                  image cell.
     *
     * @frame           N/A
     * @units           count
     */
    int maximumFeaturesPerCell{40};

    /*!
     * @brief           Whether the accumulated visual pose remains
     *                  continuous.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isVisualPoseAvailable{true};

    /*!
     * @brief           Number of synchronized pairs admitted.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t receivedPairCount{0U};

    /*!
     * @brief           Number of accepted visual pose updates.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t acceptedPoseCount{0U};

    /*!
     * @brief           Number of callbacks that failed to produce a pose.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t failedPoseCount{0U};

    /*!
     * @brief           Pair count at the previous diagnostics record.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousReportedPairCount{0U};

    /*!
     * @brief           Accepted count at the previous diagnostics record.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousReportedAcceptedCount{0U};

    /*!
     * @brief           Simulation time of the previous diagnostics record;
     *                  zero before the first record.
     *
     * @frame           N/A
     * @units           ROS time
     */
    rclcpp::Time previousReportTime{0, 0, RCL_ROS_TIME};

    /*!
     * @brief           Number of diagnostics records published.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t diagnosticsRecordCount{0U};

    /*!
     * @brief           Accepted count at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleAcceptedCount{0U};

    /*!
     * @brief           Failed count at the previous console health line.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t previousConsoleFailedCount{0U};

    /*!
     * @brief           Simulation time of the previous console health line;
     *                  zero before the first line.
     *
     * @frame           N/A
     * @units           ROS time
     */
    rclcpp::Time previousConsoleTime{0, 0, RCL_ROS_TIME};

    /*!
     * @brief           Consecutive callbacks without an accepted pose.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t consecutiveFailureCount{0U};

    /*!
     * @brief           Consecutive callbacks that produced an accepted pose.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t consecutiveAcceptedCount{0U};

    /*!
     * @brief           Consecutive accepted poses needed before this node
     *                  reports itself ready.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t readinessConsecutivePoses{3U};

    /*!
     * @brief           Longest time without an accepted pose for which this
     *                  node still reports itself ready.
     *
     * @frame           N/A
     * @units           seconds
     */
    double readinessMaximumPoseGapS{3.0};

    /*!
     * @brief           Whether the current visual epoch has reached the
     *                  consecutive-pose streak; cleared by an epoch reset
     *                  or a lost pose.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasReadinessStreak{false};

    /*!
     * @brief           Node time of the latest accepted pose; negative
     *                  before the first.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double latestAcceptedPoseTime_s{-1.0};

    /*!
     * @brief           Latest PnP input correspondence count.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t latestCorrespondenceCount{0U};

    /*!
     * @brief           Latest accepted PnP inlier count.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t latestInlierCount{0U};

    /*!
     * @brief           Latest current-frame detected feature count.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t latestDetectedCount{0U};

    /*!
     * @brief           Latest successfully tracked feature count.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t latestTrackedCount{0U};

    /*!
     * @brief           Latest median image motion of the tracked keyframe
     *                  corners; NaN when nothing was tracked.
     *
     * @frame           Image
     * @units           pixels
     */
    double latestMedianParallaxPx{0.0};

    /*!
     * @brief           Number of accepted estimates that kept the keyframe
     *                  under the parallax policy instead of publishing.
     *
     * @frame           N/A
     * @units           count
     */
    std::uint64_t keyframeRetainedCount{0U};

    /*!
     * @brief           Latest valid stereo reconstruction count before PnP.
     *
     * @frame           N/A
     * @units           count
     */
    std::size_t latestStereoValidCount{0U};

    /*!
     * @brief           Geometry quality for the latest accepted PnP solve.
     *
     * @frame           N/A
     * @units           Mixed; see VisualPoseQuality
     */
    VisualPoseQuality latestVisualQuality;

    /*!
     * @brief           Latest accepted keyframe interval.
     *
     * @frame           N/A
     * @units           seconds
     */
    double latestAcceptedInterval_s{0.0};

    /*!
     * @brief           Latest image age at callback admission.
     *
     * @frame           N/A
     * @units           seconds
     */
    double latestAdmissionAge_s{0.0};

    /*!
     * @brief           Latest full callback duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestProcessingDuration_ms{0.0};

    /*!
     * @brief           Latest image-conversion duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestConversionDuration_ms{0.0};

    /*!
     * @brief           Latest disparity duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestDisparityDuration_ms{0.0};

    /*!
     * @brief           Latest corner-detection duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestDetectionDuration_ms{0.0};

    /*!
     * @brief           Latest feature-tracking duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestTrackingDuration_ms{0.0};

    /*!
     * @brief           Latest reconstruction duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestReconstructionDuration_ms{0.0};

    /*!
     * @brief           Latest PnP duration.
     *
     * @frame           N/A
     * @units           wall milliseconds
     */
    double latestPnpDuration_ms{0.0};
};

} /* namespace localisation::visual_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_VISUAL_ODOMETRY_NODE_H */
