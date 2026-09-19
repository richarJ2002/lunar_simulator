/*!
 * @File:         main.cpp
 *
 * @Brief:        Entry point that composes and spins every node in Alpha's
 *                stack -- this system's own estimate/sensor/control nodes,
 *                the localisation stack, the fusing Kalman filter, and the
 *                Ackermann controller -- in one process.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    /* Initialize rclcpp before any node is constructed. */
    rclcpp::init(argc, argv);

    /*!
     * A freshly spawned Gazebo model drops the last few millimetres onto the
     * terrain and settles under lunar gravity for a brief moment before
     * physics stabilizes -- a simulation artifact with no real-rover
     * equivalent (a real rover isn't powered on mid-drop). Waiting here,
     * before any node in Alpha's stack exists (hence before any of them can
     * subscribe to anything), means that settling motion finishes before any
     * node ever sees a sensor message, rather than being read as if it were
     * a real, stationary-IMU calibration sample -- inertial_odometry's own
     * startup bias calibration assumes genuine stillness and has no way to
     * distinguish settling from noise. 5 s is a generous multiple of how
     * long that settling actually takes; not a per-deployment tuning knob,
     * so a plain constant here rather than a declared ROS parameter -- Alpha
     * Node itself isn't an rclcpp::Node and no node exists yet to declare
     * one on.
     */
    std::this_thread::sleep_for(std::chrono::seconds(5));

    {
        /* Construct every node in Alpha's stack, then block, dispatching
         * every node's callbacks, until shutdown is requested. */
        systems::alpha::alpha_node::AlphaNode alpha;
        alpha.spin();
    }

    /* Release rclcpp's global resources once spinning has stopped. */
    rclcpp::shutdown();

    /* Report ordinary process exit. */
    return 0;
}
