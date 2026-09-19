/*!
 * @file            publishNoisyJointStateCallBack.cc
 *
 * @brief           Adds configured noise to measured joint states.
 *
 * @date            17/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaDriverNode.h"

namespace systems::alpha::alpha_drivers
{

void AlphaDriverNode::publishNoisyJointStateCallBack(
    const sensor_msgs::msg::JointState &message_in)
{
    /* Start from a copy of the raw, noise-free joint state. */
    sensor_msgs::msg::JointState message = message_in;

    /*!
     * Every steering-joint position and every drive-joint velocity gets its
     * own independent noise sample, matching per-encoder measurement error
     * rather than one shared error across all six wheels.
     */
    for (double &position_rad : message.position)
    {
        /* Add one independent noise sample to this steering position. */
        position_rad += sampleGaussian(wheelPositionStddev_rad);
    }

    /* Perturb every drive-joint velocity the same way. */
    for (double &velocity_radPerS : message.velocity)
    {
        /* Add one independent noise sample to this drive velocity. */
        velocity_radPerS += sampleGaussian(wheelVelocityStddev_radPerS);
    }

    /* Publish the perturbed measurement to the public topic. */
    p_jointStatePublisher->publish(message);
}

} /* namespace systems::alpha::alpha_drivers */
