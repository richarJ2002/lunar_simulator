/*!
 * @File:         handleVelocityCommandCallBack.cc
 *
 * @Brief:        Implements conversion of one commanded body velocity into
 *                six wheel steering angles and speeds.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AckermannControllerNodeClass.h"

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <cstddef>

namespace control::ackermann_controller
{

void AckermannControllerNode::handleVelocityCommandCallBack(
    const geometry_msgs::msg::Twist &message_in)
{
    const double vx_mPs        = message_in.linear.x;
    const double vy_mPs        = message_in.linear.y;
    const double yawRate_radPs = message_in.angular.z;

    /*!
     * Each wheel's steering angle and unscaled (pre-limit) required speed,
     * computed once and reused below whether or not scaling ends up applying.
     */
    std::array<double, 6> steeringAngle_rad{};
    std::array<double, 6> unscaledSpeed_radPs{};
    double                maxUnscaledSpeed_radPs = 0.0;

    for (std::size_t wheelIndex = 0; wheelIndex < wheelX_m.size(); wheelIndex++)
    {
        steeringAngle_rad[wheelIndex] =
            computeSteeringAngle(vx_mPs,
                                 vy_mPs,
                                 yawRate_radPs,
                                 wheelX_m[wheelIndex],
                                 wheelY_m[wheelIndex]);

        unscaledSpeed_radPs[wheelIndex] =
            computeWheelSpeed_radPs(vx_mPs,
                                   vy_mPs,
                                   yawRate_radPs,
                                   wheelX_m[wheelIndex],
                                   wheelY_m[wheelIndex],
                                   wheelRadius_m);

        maxUnscaledSpeed_radPs =
            std::max(maxUnscaledSpeed_radPs, unscaledSpeed_radPs[wheelIndex]);
    }

    /*!
     * Proportionally scale every wheel's speed down by the same factor
     * when the fastest-required wheel would exceed Alpha's physical
     * maximum_wheel_speed_radps, so the commanded motion's shape is
     * preserved and only its overall rate is reduced. The steering angles
     * computed above need no corresponding scaling: atan2 of two values
     * scaled by the same positive factor returns the same angle, so a
     * uniform speed scale alone already yields the physically-scaled
     * command.
     */
    const double speedScale =
        (maxUnscaledSpeed_radPs > maximumWheelSpeed_radPs)
            ? maximumWheelSpeed_radPs / maxUnscaledSpeed_radPs
            : 1.0;

    actuator_msgs::msg::Actuators command;
    command.header.stamp = now();
    command.position.resize(wheelX_m.size());
    command.velocity.resize(wheelX_m.size());

    for (std::size_t wheelIndex = 0; wheelIndex < wheelX_m.size(); wheelIndex++)
    {
        command.position[wheelIndex] = steeringAngle_rad[wheelIndex];

        /*!
         * The sign convention flips the always-non-negative rolling speed
         * into each wheel's own raw actuator sign convention, mirroring
         * wheel_odometry's inverse use of the same per-wheel multiplier.
         */
        command.velocity[wheelIndex] = unscaledSpeed_radPs[wheelIndex] *
                                       speedScale *
                                       driveDirectionMultipliers[wheelIndex];
    }

    p_wheelCommandPublisher->publish(command);
}

} /* namespace control::ackermann_controller */
