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
    constexpr std::int64_t NANOSECONDS_PER_SECOND = 1000000000;
    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(message_in.header.stamp.sec) *
            NANOSECONDS_PER_SECOND +
        static_cast<std::int64_t>(message_in.header.stamp.nanosec);

    /* Gazebo 8.11 publishes joint states every physics iteration. Keep the
     * hardware-facing stream at its configured sensor rate before adding
     * noise or triggering the downstream wheel solve. A backwards clock jump
     * starts a new interval immediately. */
    if (hasPreviousJointStateStamp &&
        stamp_ns >= previousJointStateStamp_ns &&
        stamp_ns - previousJointStateStamp_ns < jointStateMinimumPeriod_ns)
    {
        return;
    }
    previousJointStateStamp_ns = stamp_ns;
    hasPreviousJointStateStamp = true;

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
