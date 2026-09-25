/*!
 * @file            public_functions.h
 *
 * @brief           Declares wheel odometry's pure kinematic solve.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_PUBLIC_FUNCTIONS_H
#define LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_PUBLIC_FUNCTIONS_H

/* C++ Standard Library Includes */
#include <array>

/* Object Includes */
#include "objects/BodyTwistSolution.h"

namespace localisation::wheel_odometry
{

/*!
 * @brief           Solves the planar body twist from six steered wheels by
 *                  twelve-row weighted least squares.
 *
 *                  For wheel i at body position (x_i, y_i) with steering
 *                  angle d_i and measured rolling speed s_i, the wheel's
 *                  contact-point velocity (vx - wz y_i, vy + wz x_i) gives
 *                  two constraints on the body twist t = (vx, vy, wz):
 *
 *                  rolling:  cos d_i vx + sin d_i vy
 *                              + (x_i sin d_i - y_i cos d_i) wz = s_i
 *                  no-side-slip:
 *                           -sin d_i vx + cos d_i vy
 *                              + (x_i cos d_i + y_i sin d_i) wz = 0
 *
 *                  Rolling rows carry the encoder noise
 *                  rollingSpeedStddevMps_in. No-side-slip rows carry the
 *                  expected lateral slip lateralSlipStddevMps_in plus the
 *                  steering-encoder noise projected through the row's
 *                  derivative with respect to d_i, which is -s_i; a parked
 *                  rover's steering noise therefore cannot create lateral
 *                  velocity. (The rolling row's derivative is the wheel's
 *                  lateral velocity, zero under the model, so steering noise
 *                  is not added there.) The twist is
 *                  (A^T W A)^-1 A^T W b and its covariance (A^T W A)^-1,
 *                  where W holds the inverse row variances. Unlike the
 *                  six rolling rows alone, the stacked system observes vy
 *                  and wz in every steering layout, including parallel
 *                  wheels.
 *
 * @param[in]       steeringAngle_rad_in
 *                  Steering angle per wheel, radians, in wheel order.
 *
 * @param[in]       rollingSpeed_mps_in
 *                  Signed, slip-adjusted circumferential speed per wheel,
 *                  metres per second, in wheel order.
 *
 * @param[in]       wheelX_m_in
 *                  Body-frame x position per wheel, metres.
 *
 * @param[in]       wheelY_m_in
 *                  Body-frame y position per wheel, metres.
 *
 * @param[in]       rollingSpeedStddevMps_in
 *                  Rolling-speed measurement standard deviation, metres
 *                  per second; must be positive.
 *
 * @param[in]       steeringStddevRad_in
 *                  Steering-angle standard deviation, radians; must not
 *                  be negative.
 *
 * @param[in]       lateralSlipStddevMps_in
 *                  Expected lateral slip standard deviation of a wheel,
 *                  metres per second; must be positive.
 *
 * @return          The twist and covariance; isValid is false for invalid
 *                  standard deviations, non-finite inputs or a normal
 *                  matrix that is not positive definite.
 */
BodyTwistSolution
    solveBodyTwist(const std::array<double, 6> &steeringAngle_rad_in,
                   const std::array<double, 6> &rollingSpeed_mps_in,
                   const std::array<double, 6> &wheelX_m_in,
                   const std::array<double, 6> &wheelY_m_in,
                   double                       rollingSpeedStddevMps_in,
                   double                       steeringStddevRad_in,
                   double                       lateralSlipStddevMps_in);

} /* namespace localisation::wheel_odometry */

#endif /* LUNAR_SIMULATOR_LOCALISATION_WHEEL_ODOMETRY_PUBLIC_FUNCTIONS_H */
