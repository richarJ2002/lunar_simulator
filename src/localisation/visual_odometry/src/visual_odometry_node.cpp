/*!
 * @File:         visual_odometry_node.cpp
 *
 * @Brief:        Estimates rover motion from the Alpha LocCam stereo pair.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

/* Generic Libraries */
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lunar_simulator::localisation
{

class VisualOdometryNode final : public rclcpp::Node
{
  public:
    using StereoPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    VisualOdometryNode() : Node("visual_odometry")
    {
        const std::string leftTopic = declare_parameter<std::string>(
            "left_image_topic", "/alpha/loccam/left");
        const std::string rightTopic = declare_parameter<std::string>(
            "right_image_topic", "/alpha/loccam/right");
        const std::string outputTopic = declare_parameter<std::string>(
            "odometry_topic", "/localisation/visual/odometry");
        const std::string pointCloudTopic = declare_parameter<std::string>(
            "point_cloud_topic", "/localisation/visual/point_cloud");
        const std::string featureImageTopic = declare_parameter<std::string>(
            "feature_image_topic", "/localisation/visual/features");
        odomFrame = declare_parameter<std::string>("odom_frame", "map");
        baseFrame =
            declare_parameter<std::string>("base_frame", "alpha/base_link");
        fxPx = declare_parameter<double>("fx_px", 800.0);
        fyPx = declare_parameter<double>("fy_px", 800.0);
        cxPx = declare_parameter<double>("cx_px", 512.0);
        cyPx = declare_parameter<double>("cy_px", 512.0);
        baselineM = declare_parameter<double>("stereo_baseline_m", 0.15);
        maximumDepthM = declare_parameter<double>("maximum_depth_m", 25.0);
        maximumFeatures = declare_parameter<int>("maximum_features", 500);
        minimumCorrespondences =
            declare_parameter<int>("minimum_correspondences", 20);
        int numberOfDisparities =
            declare_parameter<int>("num_disparities", 128);
        numberOfDisparities =
            std::max(16, ((numberOfDisparities + 15) / 16) * 16);
        int blockSize = declare_parameter<int>("block_size", 15);
        blockSize = std::max(5, blockSize | 1);

        const double cameraXM = declare_parameter<double>("camera_x_m", 0.932);
        const double cameraZM = declare_parameter<double>("camera_z_m", 0.60);
        const double cameraPitchRad =
            declare_parameter<double>("camera_pitch_rad", 0.174533);
        configureCameraTransform(cameraXM, cameraZM, cameraPitchRad);

        cameraMatrix = (cv::Mat_<double>(3, 3) << fxPx, 0.0, cxPx, 0.0, fyPx,
                        cyPx, 0.0, 0.0, 1.0);
        stereoMatcher = cv::StereoBM::create(numberOfDisparities, blockSize);
        stereoMatcher->setTextureThreshold(5);
        stereoMatcher->setUniquenessRatio(8);
        stereoMatcher->setSpeckleWindowSize(50);
        stereoMatcher->setSpeckleRange(2);

        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>(
            outputTopic, rclcpp::QoS(10));
        pointCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic, rclcpp::QoS(10).reliable());
        featureImagePublisher = create_publisher<sensor_msgs::msg::Image>(
            featureImageTopic, rclcpp::QoS(10).reliable());
        leftSubscriber.subscribe(this, leftTopic, rmw_qos_profile_sensor_data);
        rightSubscriber.subscribe(this, rightTopic,
                                  rmw_qos_profile_sensor_data);
        synchronizer =
            std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
                StereoPolicy(5), leftSubscriber, rightSubscriber);
        synchronizer->registerCallback(
            std::bind(&VisualOdometryNode::handleStereo, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "Stereo visual odometry: [%s, %s] -> [%s, %s, %s]",
                    leftTopic.c_str(), rightTopic.c_str(), outputTopic.c_str(),
                    pointCloudTopic.c_str(), featureImageTopic.c_str());
    }

  private:
    static double stampToSeconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(stamp.sec) +
               1.0e-9 * static_cast<double>(stamp.nanosec);
    }

    static cv::Matx44d multiply(const cv::Matx44d &left,
                                const cv::Matx44d &right)
    {
        return left * right;
    }

    static cv::Matx44d invertRigid(const cv::Matx44d &transform)
    {
        cv::Matx44d inverse = cv::Matx44d::eye();
        cv::Matx33d rotation;
        cv::Vec3d translation;
        for (int row = 0; row < 3; ++row)
        {
            translation[row] = transform(row, 3);
            for (int column = 0; column < 3; ++column)
            {
                rotation(row, column) = transform(row, column);
            }
        }
        const cv::Matx33d rotationTranspose = rotation.t();
        const cv::Vec3d inverseTranslation = -(rotationTranspose * translation);
        for (int row = 0; row < 3; ++row)
        {
            inverse(row, 3) = inverseTranslation[row];
            for (int column = 0; column < 3; ++column)
            {
                inverse(row, column) = rotationTranspose(row, column);
            }
        }
        return inverse;
    }

    void configureCameraTransform(double cameraXM, double cameraZM,
                                  double cameraPitchRad)
    {
        const double cosine = std::cos(cameraPitchRad);
        const double sine = std::sin(cameraPitchRad);
        const cv::Matx33d baseFromMount(cosine, 0.0, sine, 0.0, 1.0, 0.0, -sine,
                                        0.0, cosine);
        const cv::Matx33d mountFromOptical(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0,
                                           -1.0, 0.0);
        const cv::Matx33d baseFromOptical = baseFromMount * mountFromOptical;
        bodyFromOptical = cv::Matx44d::eye();
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                bodyFromOptical(row, column) = baseFromOptical(row, column);
            }
        }
        bodyFromOptical(0, 3) = cameraXM;
        bodyFromOptical(2, 3) = cameraZM;
        worldFromOptical = bodyFromOptical;
    }

    void handleStereo(const sensor_msgs::msg::Image::ConstSharedPtr &p_left,
                      const sensor_msgs::msg::Image::ConstSharedPtr &p_right)
    {
        cv::Mat currentLeft;
        cv::Mat currentRight;
        try
        {
            currentLeft = cv_bridge::toCvCopy(p_left, "mono8")->image;
            currentRight = cv_bridge::toCvCopy(p_right, "mono8")->image;
        }
        catch (const cv_bridge::Exception &error)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "LocCam conversion failed: %s", error.what());
            return;
        }

        std::vector<cv::Point2f> detectedFeatures;
        cv::goodFeaturesToTrack(currentLeft, detectedFeatures, maximumFeatures,
                                0.01, 8.0);

        if (previousLeft.empty())
        {
            publishFeatureImage(p_left->header, currentLeft, detectedFeatures,
                                {}, {}, {});
            publishPointCloud(p_left->header.stamp, {}, {}, worldFromOptical);
            storePrevious(currentLeft, currentRight, p_left->header.stamp);
            const cv::Matx44d initialPose = cv::Matx44d::eye();
            publishOdometry(p_left->header.stamp, 1.0, 1U, initialPose,
                            initialPose);
            return;
        }

        const double currentStampS = stampToSeconds(p_left->header.stamp);
        const double dtS = currentStampS - previousStampS;
        if (!(dtS > 0.0) || dtS > 1.0)
        {
            publishFeatureImage(p_left->header, currentLeft, detectedFeatures,
                                {}, {}, {});
            publishPointCloud(p_left->header.stamp, {}, {}, worldFromOptical);
            storePrevious(currentLeft, currentRight, p_left->header.stamp);
            return;
        }

        cv::Mat disparityFixed;
        stereoMatcher->compute(previousLeft, previousRight, disparityFixed);
        cv::Mat disparity;
        disparityFixed.convertTo(disparity, CV_32F, 1.0 / 16.0);

        std::vector<cv::Point2f> previousFeatures;
        cv::goodFeaturesToTrack(previousLeft, previousFeatures, maximumFeatures,
                                0.01, 8.0);
        std::vector<cv::Point2f> currentFeatures;
        std::vector<unsigned char> trackingStatus;
        std::vector<float> trackingError;
        if (!previousFeatures.empty())
        {
            cv::calcOpticalFlowPyrLK(previousLeft, currentLeft,
                                     previousFeatures, currentFeatures,
                                     trackingStatus, trackingError);
        }

        std::vector<cv::Point3f> previousPoints3d;
        std::vector<cv::Point2f> currentPoints2d;
        for (std::size_t index = 0U; index < previousFeatures.size(); ++index)
        {
            if (trackingStatus[index] == 0U || trackingError[index] > 30.0F)
            {
                continue;
            }
            const int u =
                static_cast<int>(std::lround(previousFeatures[index].x));
            const int v =
                static_cast<int>(std::lround(previousFeatures[index].y));
            if (u < 0 || v < 0 || u >= disparity.cols || v >= disparity.rows)
            {
                continue;
            }
            const float disparityPx = disparity.at<float>(v, u);
            if (!(disparityPx > 1.0F))
            {
                continue;
            }
            const double depthM =
                fxPx * baselineM / static_cast<double>(disparityPx);
            if (!(depthM > 0.08) || depthM > maximumDepthM)
            {
                continue;
            }
            const double xM =
                (previousFeatures[index].x - cxPx) * depthM / fxPx;
            const double yM =
                (previousFeatures[index].y - cyPx) * depthM / fyPx;
            previousPoints3d.emplace_back(static_cast<float>(xM),
                                          static_cast<float>(yM),
                                          static_cast<float>(depthM));
            currentPoints2d.push_back(currentFeatures[index]);
        }

        publishFeatureImage(p_left->header, currentLeft, previousFeatures,
                            currentFeatures, trackingStatus, currentPoints2d);

        bool estimated = false;
        if (previousPoints3d.size() >=
            static_cast<std::size_t>(minimumCorrespondences))
        {
            cv::Mat rotationVector;
            cv::Mat translationVector;
            std::vector<int> inliers;
            estimated = cv::solvePnPRansac(
                previousPoints3d, currentPoints2d, cameraMatrix, cv::noArray(),
                rotationVector, translationVector, false, 100, 2.5, 0.99,
                inliers, cv::SOLVEPNP_ITERATIVE);
            estimated = estimated &&
                        inliers.size() >=
                            static_cast<std::size_t>(minimumCorrespondences);
            if (estimated)
            {
                updatePose(rotationVector, translationVector,
                           p_left->header.stamp, dtS, previousPoints3d,
                           inliers);
            }
        }

        if (!estimated)
        {
            std::vector<int> correlatedIndices(previousPoints3d.size());
            std::iota(correlatedIndices.begin(), correlatedIndices.end(), 0);
            publishPointCloud(p_left->header.stamp, previousPoints3d,
                              correlatedIndices, worldFromOptical);
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Visual odometry lacks stereo/temporal correspondences");
        }
        storePrevious(currentLeft, currentRight, p_left->header.stamp);
    }

    void
    publishFeatureImage(const std_msgs::msg::Header &header_in,
                        const cv::Mat &imageMono_in,
                        const std::vector<cv::Point2f> &previousFeatures_in,
                        const std::vector<cv::Point2f> &currentFeatures_in,
                        const std::vector<unsigned char> &trackingStatus_in,
                        const std::vector<cv::Point2f> &stereoCorrelations_in)
    {
        cv::Mat annotatedImage;
        cv::cvtColor(imageMono_in, annotatedImage, cv::COLOR_GRAY2BGR);
        const bool hasTracks =
            currentFeatures_in.size() == previousFeatures_in.size() &&
            trackingStatus_in.size() == previousFeatures_in.size();
        if (hasTracks)
        {
            for (std::size_t index = 0U; index < previousFeatures_in.size();
                 ++index)
            {
                if (trackingStatus_in[index] == 0U)
                {
                    continue;
                }
                cv::line(annotatedImage, previousFeatures_in[index],
                         currentFeatures_in[index], cv::Scalar(0, 255, 255), 1,
                         cv::LINE_AA);
                cv::circle(annotatedImage, currentFeatures_in[index], 3,
                           cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            }
        }
        else
        {
            for (const cv::Point2f &feature : previousFeatures_in)
            {
                cv::circle(annotatedImage, feature, 3, cv::Scalar(255, 128, 0),
                           -1, cv::LINE_AA);
            }
        }
        for (const cv::Point2f &correlation : stereoCorrelations_in)
        {
            cv::circle(annotatedImage, correlation, 5, cv::Scalar(255, 0, 255),
                       1, cv::LINE_AA);
        }

        cv_bridge::CvImage output(header_in, "bgr8", annotatedImage);
        featureImagePublisher->publish(*output.toImageMsg());
    }

    void updatePose(const cv::Mat &rotationVector,
                    const cv::Mat &translationVector,
                    const builtin_interfaces::msg::Time &stamp, double dtS,
                    const std::vector<cv::Point3f> &correlatedPoints_in,
                    const std::vector<int> &inlierIndices_in)
    {
        cv::Mat rotationMatrix;
        cv::Rodrigues(rotationVector, rotationMatrix);
        cv::Matx44d currentFromPrevious = cv::Matx44d::eye();
        for (int row = 0; row < 3; ++row)
        {
            currentFromPrevious(row, 3) = translationVector.at<double>(row);
            for (int column = 0; column < 3; ++column)
            {
                currentFromPrevious(row, column) =
                    rotationMatrix.at<double>(row, column);
            }
        }

        const cv::Matx44d previousWorldFromOptical = worldFromOptical;
        const cv::Matx44d previousWorldFromBody =
            multiply(previousWorldFromOptical, invertRigid(bodyFromOptical));
        worldFromOptical =
            multiply(worldFromOptical, invertRigid(currentFromPrevious));
        const cv::Matx44d worldFromBody =
            multiply(worldFromOptical, invertRigid(bodyFromOptical));
        publishOdometry(stamp, dtS, inlierIndices_in.size(),
                        previousWorldFromBody, worldFromBody);
        publishPointCloud(stamp, correlatedPoints_in, inlierIndices_in,
                          previousWorldFromOptical);
    }

    void publishPointCloud(const builtin_interfaces::msg::Time &stamp,
                           const std::vector<cv::Point3f> &correlatedPoints_in,
                           const std::vector<int> &inlierIndices_in,
                           const cv::Matx44d &mapFromOptical_in)
    {
        std::vector<cv::Point3f> mapPoints;
        mapPoints.reserve(inlierIndices_in.size());
        for (const int inlierIndex : inlierIndices_in)
        {
            if (inlierIndex < 0 || static_cast<std::size_t>(inlierIndex) >=
                                       correlatedPoints_in.size())
            {
                continue;
            }
            const cv::Point3f &point =
                correlatedPoints_in[static_cast<std::size_t>(inlierIndex)];
            const cv::Vec4d pointOptical(point.x, point.y, point.z, 1.0);
            const cv::Vec4d pointMap = mapFromOptical_in * pointOptical;
            mapPoints.emplace_back(static_cast<float>(pointMap[0]),
                                   static_cast<float>(pointMap[1]),
                                   static_cast<float>(pointMap[2]));
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = odomFrame;
        cloud.height = 1U;
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(mapPoints.size());
        sensor_msgs::PointCloud2Iterator<float> xIterator(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> yIterator(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> zIterator(cloud, "z");
        for (const cv::Point3f &point : mapPoints)
        {
            *xIterator = point.x;
            *yIterator = point.y;
            *zIterator = point.z;
            ++xIterator;
            ++yIterator;
            ++zIterator;
        }
        pointCloudPublisher->publish(cloud);
    }

    static geometry_msgs::msg::Quaternion
    rotationToQuaternion(const cv::Matx44d &transform)
    {
        const double trace =
            transform(0, 0) + transform(1, 1) + transform(2, 2);
        geometry_msgs::msg::Quaternion result;
        if (trace > 0.0)
        {
            const double scale = 2.0 * std::sqrt(trace + 1.0);
            result.w = 0.25 * scale;
            result.x = (transform(2, 1) - transform(1, 2)) / scale;
            result.y = (transform(0, 2) - transform(2, 0)) / scale;
            result.z = (transform(1, 0) - transform(0, 1)) / scale;
        }
        else if (transform(0, 0) > transform(1, 1) &&
                 transform(0, 0) > transform(2, 2))
        {
            const double scale =
                2.0 * std::sqrt(1.0 + transform(0, 0) - transform(1, 1) -
                                transform(2, 2));
            result.w = (transform(2, 1) - transform(1, 2)) / scale;
            result.x = 0.25 * scale;
            result.y = (transform(0, 1) + transform(1, 0)) / scale;
            result.z = (transform(0, 2) + transform(2, 0)) / scale;
        }
        else if (transform(1, 1) > transform(2, 2))
        {
            const double scale =
                2.0 * std::sqrt(1.0 + transform(1, 1) - transform(0, 0) -
                                transform(2, 2));
            result.w = (transform(0, 2) - transform(2, 0)) / scale;
            result.x = (transform(0, 1) + transform(1, 0)) / scale;
            result.y = 0.25 * scale;
            result.z = (transform(1, 2) + transform(2, 1)) / scale;
        }
        else
        {
            const double scale =
                2.0 * std::sqrt(1.0 + transform(2, 2) - transform(0, 0) -
                                transform(1, 1));
            result.w = (transform(1, 0) - transform(0, 1)) / scale;
            result.x = (transform(0, 2) + transform(2, 0)) / scale;
            result.y = (transform(1, 2) + transform(2, 1)) / scale;
            result.z = 0.25 * scale;
        }
        return result;
    }

    void publishOdometry(const builtin_interfaces::msg::Time &stamp, double dtS,
                         std::size_t inlierCount,
                         const cv::Matx44d &previousPose,
                         const cv::Matx44d &currentPose)
    {
        nav_msgs::msg::Odometry output;
        output.header.stamp = stamp;
        output.header.frame_id = odomFrame;
        output.child_frame_id = baseFrame;
        output.pose.pose.position.x = currentPose(0, 3);
        output.pose.pose.position.y = currentPose(1, 3);
        output.pose.pose.position.z = currentPose(2, 3);
        output.pose.pose.orientation = rotationToQuaternion(currentPose);

        const cv::Matx44d previousFromCurrent =
            multiply(invertRigid(previousPose), currentPose);
        const cv::Vec3d translationBody(previousFromCurrent(0, 3),
                                        previousFromCurrent(1, 3),
                                        previousFromCurrent(2, 3));
        output.twist.twist.linear.x = translationBody[0] / dtS;
        output.twist.twist.linear.y = translationBody[1] / dtS;
        output.twist.twist.linear.z = translationBody[2] / dtS;
        cv::Mat relativeRotation(3, 3, CV_64F);
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                relativeRotation.at<double>(row, column) =
                    previousFromCurrent(row, column);
            }
        }
        cv::Mat relativeRotationVector;
        cv::Rodrigues(relativeRotation, relativeRotationVector);
        output.twist.twist.angular.x =
            relativeRotationVector.at<double>(0) / dtS;
        output.twist.twist.angular.y =
            relativeRotationVector.at<double>(1) / dtS;
        output.twist.twist.angular.z =
            relativeRotationVector.at<double>(2) / dtS;

        const double variance =
            std::max(0.002, 0.20 / static_cast<double>(inlierCount));
        for (int index = 0; index < 6; ++index)
        {
            output.pose
                .covariance[static_cast<std::size_t>(index * 6 + index)] =
                variance;
            output.twist
                .covariance[static_cast<std::size_t>(index * 6 + index)] =
                2.0 * variance;
        }
        odometryPublisher->publish(output);
    }

    void storePrevious(const cv::Mat &left, const cv::Mat &right,
                       const builtin_interfaces::msg::Time &stamp)
    {
        previousLeft = left.clone();
        previousRight = right.clone();
        previousStampS = stampToSeconds(stamp);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr featureImagePublisher;
    message_filters::Subscriber<sensor_msgs::msg::Image> leftSubscriber;
    message_filters::Subscriber<sensor_msgs::msg::Image> rightSubscriber;
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> synchronizer;
    cv::Ptr<cv::StereoBM> stereoMatcher;
    cv::Mat cameraMatrix;
    cv::Mat previousLeft;
    cv::Mat previousRight;
    cv::Matx44d bodyFromOptical{cv::Matx44d::eye()};
    cv::Matx44d worldFromOptical{cv::Matx44d::eye()};
    std::string odomFrame;
    std::string baseFrame;
    double fxPx{800.0};
    double fyPx{800.0};
    double cxPx{512.0};
    double cyPx{512.0};
    double baselineM{0.15};
    double maximumDepthM{25.0};
    double previousStampS{0.0};
    int maximumFeatures{500};
    int minimumCorrespondences{20};
};

} // namespace lunar_simulator::localisation

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<lunar_simulator::localisation::VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
