/*!
 * @file            FusedMeasurement.h
 *
 * @brief           Declares one fused odometry measurement retained for
 *                  rollback replay.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_FUSED_MEASUREMENT_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_FUSED_MEASUREMENT_H

/* C++ Standard Library Includes */
/* None */

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <Eigen/Dense>

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/MeasurementKind.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/*!
 * @brief           Holds the state-independent part of one odometry update,
 *                  so it can be applied again after a rollback.
 *
 *                  The innovation and observation matrix depend on the
 *                  nominal state at the measurement epoch, which changes
 *                  when an older measurement is inserted before this one.
 *                  Only the measured values, their noise and any inputs
 *                  taken from other buffers are stored; the model is
 *                  re-evaluated against the replayed state.
 */
struct FusedMeasurement
{
  public:
    /*!
     * @brief           Source that produced the measurement.
     *
     * @frame           N/A
     * @units           N/A
     */
    MeasurementKind kind{MeasurementKind::MEASUREMENT_KIND_WHEEL};

    /*!
     * @brief           True when a visual measurement is fused as a
     *                  body-velocity and yaw-rate increment rather than as
     *                  an absolute pose.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool isVisualIncrement{false};

    /*!
     * @brief           Measurement epoch from the source header stamp.
     *
     * @frame           N/A
     * @units           ROS seconds
     */
    double timestamp_s{0.0};

    /*!
     * @brief           Measured values, packed per source.
     *
     *                  Wheel: body vx, vy. Visual increment: body vx, vy,
     *                  vz, yaw rate. Visual pose: position x, y, z, then
     *                  the body-to-fixed quaternion x, y, z, w. Unused
     *                  trailing entries are zero.
     *
     * @frame           body for velocities; startup_fixed for pose
     * @units           m/s and rad/s for velocities; m for position;
     *                  unitless quaternion
     */
    Eigen::Matrix<double, 7, 1> measuredValues{
        Eigen::Matrix<double, 7, 1>::Zero()};

    /*!
     * @brief           Diagonal measurement-noise variances, one per active
     *                  observation row in the order the observation model
     *                  produces them.
     *
     *                  Wheel uses two rows, visual increment three or four
     *                  and visual pose six (position, then body attitude).
     *                  Unused trailing entries are zero.
     *
     * @frame           Matches measuredValues
     * @units           Squared measurement units
     */
    Eigen::Matrix<double, 6, 1> noiseVariances{
        Eigen::Matrix<double, 6, 1>::Zero()};

    /*!
     * @brief           True when raw gyroscope samples covered the visual
     *                  increment interval, enabling the yaw-rate row.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasYawRate{false};

    /*!
     * @brief           Mean raw gyroscope z over the visual increment
     *                  interval, captured at arrival so a replay does not
     *                  depend on the IMU buffer still holding the samples.
     *
     * @frame           body
     * @units           radians per second
     */
    double meanRawYawRate_radPerS{0.0};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_FUSED_MEASUREMENT_H \
        */
