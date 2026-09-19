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
#include "objects/AckermannControllerNode.h"

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <cstddef>

namespace control::ackermann_controller
{

void AckermannControllerNode::handleVelocityCommandCallBack(
    const geometry_msgs::msg::Twist &message_in)
{
    const double vxMps        = message_in.linear.x;
    const double vyMps        = message_in.linear.y;
    const double yawRateRadps = message_in.angular.z;

    /*!
     * Each wheel's steering angle and unscaled (pre-limit) required speed,
     * computed once and reused below whether or not scaling ends up applying.
     */
    std::array<double, 6> steeringAngleRad{};
    std::array<double, 6> unscaledSpeedRadps{};
    double                maxUnscaledSpeedRadps = 0.0;

    for (std::size_t wheelIndex = 0; wheelIndex < wheelXM.size(); ++wheelIndex)
    {
        steeringAngleRad[wheelIndex] =
            computeSteeringAngle(vxMps,
                                 vyMps,
                                 yawRateRadps,
                                 wheelXM[wheelIndex],
                                 wheelYM[wheelIndex]);

        unscaledSpeedRadps[wheelIndex] =
            computeWheelSpeedRadps(vxMps,
                                   vyMps,
                                   yawRateRadps,
                                   wheelXM[wheelIndex],
                                   wheelYM[wheelIndex],
                                   wheelRadiusM);

        maxUnscaledSpeedRadps =
            std::max(maxUnscaledSpeedRadps, unscaledSpeedRadps[wheelIndex]);
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
        (maxUnscaledSpeedRadps > maximumWheelSpeedRadps)
            ? maximumWheelSpeedRadps / maxUnscaledSpeedRadps
            : 1.0;

    actuator_msgs::msg::Actuators command;
    command.header.stamp = now();
    command.position.resize(wheelXM.size());
    command.velocity.resize(wheelXM.size());

    for (std::size_t wheelIndex = 0; wheelIndex < wheelXM.size(); ++wheelIndex)
    {
        command.position[wheelIndex] = steeringAngleRad[wheelIndex];

        /*!
         * The sign convention flips the always-non-negative rolling speed
         * into each wheel's own raw actuator sign convention, mirroring
         * wheel_odometry's inverse use of the same per-wheel multiplier.
         */
        command.velocity[wheelIndex] = unscaledSpeedRadps[wheelIndex] *
                                       speedScale *
                                       driveDirectionMultipliers[wheelIndex];
    }

    p_wheelCommandPublisher->publish(command);
}

} /* namespace control::ackermann_controller */
