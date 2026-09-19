/*!
 * @File:         AlphaNode.h
 *
 * @Brief:        Declares the class that composes and spins every node in
 *                Alpha's stack in one process.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_ALPHA_ALPHA_NODE_H
#define LUNAR_SIMULATOR_ALPHA_ALPHA_NODE_H

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AckermannControllerNode.h"
#include "objects/AlphaDriverNode.h"
#include "objects/AlphaKalmanFilterNode.h"
#include "objects/GroundTruthNode.h"
#include "objects/InertialOdometryNode.h"
#include "objects/WheelOdometryNode.h"
#include "visual_odometry_node/objects/VisualOdometryNode.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace systems::alpha::alpha_node
{

/*!
 * @brief           Owns one instance of every node in Alpha's stack --
 *                  this system's own EKF/path/TF node and Gazebo hardware
 *                  interface, the four localisation static libraries, and
 *                  the Ackermann controller -- and spins them together on
 *                  one multi-threaded executor, in one process.
 *
 *                  Alpha is this system's *choice* of stack, not part of
 *                  the localisation/control packages themselves: removing a
 *                  node from Alpha's stack, or picking a different EKF
 *                  model, is a matter of editing this class alone.
 */
class AlphaNode final
{
  public:
    /*!
     * @brief           Constructs every node in Alpha's stack and adds each
     *                  to the shared executor.
     *
     *                  Acts as this class's initialization step: by the time
     *                  the constructor returns, every node's parameters are
     *                  declared and every subscription/publisher/timer is
     *                  wired, exactly as it was when each ran as its own
     *                  process.
     */
    AlphaNode() :
        p_driverNode(std::make_shared<
                     systems::alpha::alpha_drivers::AlphaDriverNode>()),
        p_groundTruth(
            std::make_shared<localisation::ground_truth::GroundTruthNode>()),
        p_inertialOdometry(
            std::make_shared<
                localisation::inertial_odometry::InertialOdometryNode>()),
        p_visualOdometry(std::make_shared<
                         localisation::visual_odometry::VisualOdometryNode>()),
        p_wheelOdometry(std::make_shared<
                        localisation::wheel_odometry::WheelOdometryNode>()),
        p_kalmanFilter(
            std::make_shared<systems::alpha::alpha_localisation::
                                 alpha_kalman_filter::AlphaKalmanFilterNode>()),
        p_ackermannController(
            std::make_shared<
                control::ackermann_controller::AckermannControllerNode>())
    {
        /*!
         * Each node keeps its own default (mutually exclusive) callback
         * group, so this multi-threaded executor lets all seven progress
         * independently of one another -- matching the concurrency they had
         * as separate processes -- while still serializing each node's own
         * callbacks against itself, which e.g. AlphaKalmanFilterNode's
         * internal state requires.
         */
        executor.add_node(p_driverNode);
        executor.add_node(p_groundTruth);
        executor.add_node(p_inertialOdometry);
        executor.add_node(p_visualOdometry);
        executor.add_node(p_wheelOdometry);
        executor.add_node(p_kalmanFilter);
        executor.add_node(p_ackermannController);
    }

    /*!
     * @brief           Releases every owned node.
     *
     *                  Acts as this class's termination step: destroying
     *                  this object drops the executor's last reference to
     *                  every node, releasing each one's ROS interfaces.
     */
    ~AlphaNode() = default;

    AlphaNode(const AlphaNode &otherNode_in)            = delete;
    AlphaNode &operator=(const AlphaNode &otherNode_in) = delete;
    AlphaNode(AlphaNode &&otherNode_in)                 = delete;
    AlphaNode &operator=(AlphaNode &&otherNode_in)      = delete;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Dispatches every owned node's callbacks until
     *                  shutdown is requested.
     */
    void spin();

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Shared executor every owned node is added to.
     */
    rclcpp::executors::MultiThreadedExecutor executor;

    /*!
     * @brief       Interfaces Alpha's stack with Gazebo: raw sensor reads
     *              and raw actuator writes both pass through here.
     */
    std::shared_ptr<systems::alpha::alpha_drivers::AlphaDriverNode>
        p_driverNode;

    /*!
     * @brief       Publishes noise-free ground truth.
     */
    std::shared_ptr<localisation::ground_truth::GroundTruthNode> p_groundTruth;

    /*!
     * @brief       Integrates noisy IMU measurements.
     */
    std::shared_ptr<localisation::inertial_odometry::InertialOdometryNode>
        p_inertialOdometry;

    /*!
     * @brief       Estimates motion from stereo imagery.
     */
    std::shared_ptr<localisation::visual_odometry::VisualOdometryNode>
        p_visualOdometry;

    /*!
     * @brief       Estimates motion from wheel joint states.
     */
    std::shared_ptr<localisation::wheel_odometry::WheelOdometryNode>
        p_wheelOdometry;

    /*!
     * @brief       Fuses every odometry source into one EKF estimate and
     *              exposes it as odometry, a retained path, and TF.
     */
    std::shared_ptr<systems::alpha::alpha_localisation::
                        alpha_kalman_filter::AlphaKalmanFilterNode>
        p_kalmanFilter;

    /*!
     * @brief       Converts commanded body velocity into wheel commands.
     */
    std::shared_ptr<control::ackermann_controller::AckermannControllerNode>
        p_ackermannController;
};

} /* namespace systems::alpha::alpha_node */

#endif /* LUNAR_SIMULATOR_ALPHA_ALPHA_NODE_H */
