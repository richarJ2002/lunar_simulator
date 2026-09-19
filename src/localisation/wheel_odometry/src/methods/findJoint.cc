/*!
 * @File:         findJoint.cc
 *
 * @Brief:        Implements named-joint lookup within a joint-state message.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/WheelOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace localisation::wheel_odometry
{

bool WheelOdometryNode::findJoint(const sensor_msgs::msg::JointState &message_in,
                                  const std::string &name_in,
                                  double &position_out, double &velocity_out)
{
    /*!
     * JointState parallels its name/position/velocity arrays by index
     * rather than guaranteeing a fixed ordering, so every wheel's data
     * must be located by name on every callback. A linear scan is fine
     * here: the message only ever carries twelve joints (six drive, six
     * steer).
     */
    for (std::size_t index = 0U; index < message_in.name.size(); ++index)
    {
        /* Skip every entry that is not the joint being searched for. */
        if (message_in.name[index] == name_in)
        {
            /*!
             * Gazebo publishes name/position/velocity as separate
             * vectors that could in principle disagree in length; guard
             * against an out-of-range read rather than trusting message
             * well-formedness.
             */
            if (index >= message_in.position.size() ||
                index >= message_in.velocity.size())
            {
                /* The name matched but the data is incomplete; report
                 * failure rather than reading out of bounds. */
                return false;
            }

            /* Copy out the matched joint's position. */
            position_out = message_in.position[index];

            /* Copy out the matched joint's velocity. */
            velocity_out = message_in.velocity[index];

            /* The named joint was found with both fields present. */
            return true;
        }
    }

    /* No entry in the message matched the requested joint name. */
    return false;
}

} /* namespace localisation::wheel_odometry */
