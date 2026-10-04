/*!
 * @file            Point2D.h
 *
 * @brief           Declares a plain pixel-coordinate point, independent of
 *                  any third-party image or ROS message type.
 *
 * @date            16/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_POINT_2D_H
#define LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_POINT_2D_H

namespace localisation::visual_odometry::feature_tracking
{

/*!
 * @brief           A single pixel-plane coordinate.
 *
 * Deliberately independent of `cv::Point2f` (or any other third-party
 * type) so neither feature-tracking engine's public API depends on OpenCV;
 * `VisualOdometryNode` converts to/from `cv::Point2f` at its own boundary.
 */
struct Point2D
{
    /*!
     * @brief       Horizontal pixel coordinate, in pixels, image-left
     *              origin.
     */
    float x{0.0F};

    /*!
     * @brief       Vertical pixel coordinate, in pixels, image-top origin.
     */
    float y{0.0F};
};

} /* namespace localisation::visual_odometry::feature_tracking */

#endif /* LUNAR_SIMULATOR_LOCALISATION_FEATURE_TRACKING_POINT_2D_H */
