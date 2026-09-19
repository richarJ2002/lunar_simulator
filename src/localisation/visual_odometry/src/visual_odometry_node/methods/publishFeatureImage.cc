/*!
 * @File:         publishFeatureImage.cc
 *
 * @Brief:        Implements the annotated feature/track visualization.
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
#include <cstddef>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

namespace localisation::visual_odometry
{

void
VisualOdometryNode::publishFeatureImage(
    const std_msgs::msg::Header &header_in, const cv::Mat &imageMono_in,
    const std::vector<cv::Point2f> &previousFeatures_in,
    const std::vector<cv::Point2f> &currentFeatures_in,
    const std::vector<unsigned char> &trackingStatus_in,
    const std::vector<cv::Point2f> &stereoCorrelations_in)
{
    /* Colour image to draw the annotations into. */
    cv::Mat annotatedImage;

    /*!
     * Convert to colour so tracks and correlations can be drawn in distinct
     * hues over the original greyscale imagery.
     */
    cv::cvtColor(imageMono_in, annotatedImage, cv::COLOR_GRAY2BGR);

    /*!
     * Tracking arrays are only populated once optical flow has actually been
     * attempted (see handleStereoCallBack); on the very first frame, or whenever the
     * caller passes empty tracking arrays, fall back to drawing plain
     * detections with no track line.
     */
    const bool hasTracks =
        currentFeatures_in.size() == previousFeatures_in.size() &&
        trackingStatus_in.size() == previousFeatures_in.size();

    /* Draw a track line/dot per feature when tracking data is available. */
    if (hasTracks)
    {
        /* Visit every previous-frame feature that has tracking data. */
        for (std::size_t index = 0U; index < previousFeatures_in.size();
             ++index)
        {
            /* Skip a feature whose track was lost this frame. */
            if (trackingStatus_in[index] == 0U)
            {
                continue;
            }

            /*!
             * Yellow line: optical-flow displacement from the previous pixel
             * position to the current one. Green dot: current position.
             */
            cv::line(annotatedImage, previousFeatures_in[index],
                     currentFeatures_in[index], cv::Scalar(0, 255, 255), 1,
                     cv::LINE_AA);

            /* Mark the tracked feature's current position. */
            cv::circle(annotatedImage, currentFeatures_in[index], 3,
                       cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        }
    }
    else
    {
        /* Visit every detected feature (no track available yet). */
        for (const cv::Point2f &feature : previousFeatures_in)
        {
            /*!
             * Orange dot: a freshly detected corner with no established
             * track yet (first frame, or diagnostics-only calls).
             */
            cv::circle(annotatedImage, feature, 3, cv::Scalar(255, 128, 0),
                       -1, cv::LINE_AA);
        }
    }

    /* Visit every feature that also produced a valid stereo/PnP
     * correspondence. */
    for (const cv::Point2f &correlation : stereoCorrelations_in)
    {
        /*!
         * Magenta ring: a feature that also produced a valid stereo/PnP
         * correspondence, i.e. it fed the pose estimate this frame.
         */
        cv::circle(annotatedImage, correlation, 5, cv::Scalar(255, 0, 255),
                   1, cv::LINE_AA);
    }

    /* Wrap the annotated image with the source frame's header so viewers
     * can align it with the LocCam stream it was drawn from. */
    const cv_bridge::CvImage output(header_in, "bgr8", annotatedImage);

    /* Publish the finished visualization. */
    p_featureImagePublisher->publish(*output.toImageMsg());
}

} /* namespace localisation::visual_odometry */
