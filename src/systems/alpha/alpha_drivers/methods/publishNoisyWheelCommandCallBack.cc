/*!
 * @file            publishNoisyWheelCommandCallBack.cc
 *
 * @brief           Adds configured noise to one wheel command and forwards
 *                   it onto the raw actuator bridge topic.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNodeClass.h"

/* Generic Libraries */
#include <algorithm>

/* Other Project Module Includes */
#include "console/console.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::publishNoisyWheelCommandCallBack(
    const actuator_msgs::msg::Actuators &message_in)
{
    /*!
     * The gate drops every command while the system is not READY or its
     * heartbeat is stale; a dropped command is never replayed.
     */
    const CommandGateDecision gate =
        evaluateCommandGate(latestSystemState,
                            now().seconds() - latestSystemStateReceipt_s,
                            maximumStateHeartbeatAge_s);

    /* Drop a gated command after counting and reporting it. */
    if (!gate.isOpen)
    {
        blockedCommandCount++;
        SRS_LOG_WARN_THROTTLE(get_logger(),
                                *get_clock(),
                                3000,
                                "CMD blocked: %s",
                                gate.reason.c_str());
        return;
    }

    /* Count the command entering noise injection below. */
    forwardedCommandCount++;
    latestForwardedSteering_rad = message_in.position;

    /* Start from a copy of the commanded, noise-free actuator targets. */
    actuator_msgs::msg::Actuators message = message_in;

    /*!
     * Every steering-position and drive-velocity target gets its own
     * independent noise sample, matching per-actuator command imprecision
     * rather than one shared error across all six wheels.
     */
    for (double &position_rad : message.position)
    {
        /* Add one independent noise sample to this steering target. */
        position_rad += sampleGaussian(wheelCommandPositionStddev_rad);
    }

    /* Perturb every drive-velocity target the same way. */
    for (double &velocity_radPs : message.velocity)
    {
        /* Add one independent noise sample to this drive target. */
        velocity_radPs += sampleGaussian(wheelCommandVelocityStddev_radPs);

        /*!
         * Clamp to Alpha's physical maximum drive-wheel speed -- this is
         * the last software boundary before the raw actuator bridge, so it
         * also catches noise pushing an already-borderline command over
         * the limit, or any commander that does not itself respect it (see
         * this class's own doc comment for the other two layers).
         */
        velocity_radPs = std::clamp(velocity_radPs,
                                      -maximumWheelSpeed_radPs,
                                      maximumWheelSpeed_radPs);
    }

    /* Publish the perturbed command onto the raw actuator bridge topic. */
    p_rawWheelCommandPublisher->publish(message);
}

} /* namespace systems::alpha::alpha_drivers */
