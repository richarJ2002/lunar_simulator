/*!
 * @file            publishNoisyWheelCommandCallBack.cc
 *
 * @brief           Adds configured noise to one wheel command and forwards
 *                   it onto the raw actuator bridge topic.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

/* Generic Libraries */
#include <algorithm>

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::publishNoisyWheelCommandCallBack(
    const actuator_msgs::msg::Actuators &message_in)
{
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
    for (double &velocity_radPerS : message.velocity)
    {
        /* Add one independent noise sample to this drive target. */
        velocity_radPerS += sampleGaussian(wheelCommandVelocityStddev_radPerS);

        /*!
         * Clamp to Alpha's physical maximum drive-wheel speed -- this is
         * the last software boundary before the raw actuator bridge, so it
         * also catches noise pushing an already-borderline command over
         * the limit, or any commander that does not itself respect it (see
         * this class's own doc comment for the other two layers).
         */
        velocity_radPerS = std::clamp(velocity_radPerS, -maximumWheelSpeedRadps,
                                      maximumWheelSpeedRadps);
    }

    /* Publish the perturbed command onto the raw actuator bridge topic. */
    p_rawWheelCommandPublisher->publish(message);
}

} /* namespace systems::alpha::alpha_drivers */
