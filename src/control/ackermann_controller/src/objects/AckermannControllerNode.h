/*!
 * @File:         AckermannControllerNode.h
 *
 * @Brief:        Declares the node converting a body-frame velocity command
 *                into six independent wheel steering angles and speeds.
 *
 * @Date:         17/09/2026
 *
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ACKERMANN_CONTROLLER_NODE_H
#define LUNAR_SIMULATOR_CONTROL_ACKERMANN_CONTROLLER_NODE_H

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/twist.hpp>

/* Generic Libraries */
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace control::ackermann_controller
{

/*!
 * @brief           Converts a general body-frame velocity command into
 *                   six-wheel Ackermann/crab steering angles and speeds.
 *
 * Wheel order is fixed throughout this node and its configuration
 * (`wheel_x_m`, `wheel_y_m`, `drive_direction_multipliers`) as front-left,
 * front-right, centre-left, centre-right, rear-left, rear-right, matching
 * `wheel_odometry`'s identical convention. All wheel positions are
 * expressed in the rover body frame in metres.
 *
 * The commanded body twist is planar linear velocity (vx, vy) and yaw rate
 * wz, all in the body frame. Treating the rover as rigid, the required
 * velocity of the contact point of wheel i at body-frame offset (x_i, y_i)
 * is the rigid-body composition
 *
 *   velocity_i = (vx - wz * y_i, vy + wz * x_i)
 *
 * so wheel i must steer to angle atan2(vy + wz * x_i, vx - wz * y_i) and
 * roll at speed hypot(vx - wz * y_i, vy + wz * x_i) / wheel_radius_m. This
 * single formula is well-defined for every commanded twist, including
 * wz == 0 (pure crab translation) and vy == 0 (pure Ackermann cornering),
 * with no turn-radius division and therefore no singularity at wz == 0.
 * Each wheel always steers to the angle that lets it roll forward (a
 * non-negative speed); it does not consider the alternative of steering
 * 180 degrees and rolling backward, so a commanded twist that passes
 * through zero can produce a discontinuous steering-angle jump rather than
 * a continuously reversing wheel.
 *
 * Alpha's real-hardware maximum drive-wheel speed (maximum_wheel_speed_radps,
 * matching the ExoMars rover's own physical limit) is enforced here as the
 * first of three layers (see AlphaDriverNode and alpha_model/model.sdf for
 * the other two): when the fastest wheel a commanded twist would require
 * exceeds this limit, every wheel's speed is scaled down by the same
 * factor before publishing, preserving the commanded motion's shape (the
 * steering angles above are unaffected, since atan2 of two values scaled
 * by the same positive factor is unchanged) and only reducing its overall
 * rate -- see handleVelocityCommandCallBack().
 *
 * The node is intended for a single-threaded executor.
 */
class AckermannControllerNode final : public rclcpp::Node
{
  public:
    /*!
     * @brief           Declares parameters, validates them and wires the
     *                   velocity subscription to the wheel-command
     *                   publisher.
     *
     * @throws          std::invalid_argument if `wheel_radius_m` or
     *                   `maximum_wheel_speed_radps` is not positive.
     */
    AckermannControllerNode() : Node("ackermann_controller")
    {
        /*!
         * Declared first so the topic defaults below can be rooted at the
         * owning system's namespace.
         */
        const std::string systemName =
            declare_parameter<std::string>("system_name", "alpha");

        /* Input topic: commanded body-frame velocity. */
        const std::string velocityTopic = declare_parameter<std::string>(
            "velocity_topic", "/" + systemName + "/control/cmd/velocity");

        /* Output topic: the public six-wheel joint command, matching
         * AlphaDriverNode's own default so both derive the same topic
         * from system_name without either hardcoding the other's name. */
        const std::string wheelJointStatesTopic =
            declare_parameter<std::string>(
                "wheel_joint_states_topic",
                "/" + systemName + "/control/cmd/wheel_joint_states");

        /* Common wheel radius used to convert contact-point speed to
         * angular drive rate. */
        wheelRadiusM = declare_parameter<double>("wheel_radius_m", 0.1425);

        /* Reject a configuration that could never produce a physically
         * meaningful wheel speed. */
        if (wheelRadiusM <= 0.0)
        {
            /* Fail fast at construction rather than divide by a
             * non-positive radius later. */
            throw std::invalid_argument("wheel_radius_m must be positive");
        }

        /*!
         * Alpha's real-hardware maximum drive-wheel speed (matching the
         * ExoMars rover's own physical limit, ~0.02 m/s at Alpha's wheel
         * radius) -- see this class's own doc comment for the three layers
         * this is enforced at; this is the first, applied in
         * handleVelocityCommandCallBack().
         */
        maximumWheelSpeedRadps =
            declare_parameter<double>("maximum_wheel_speed_radps", 0.14);

        if (maximumWheelSpeedRadps <= 0.0)
        {
            /* Fail fast at construction rather than silently scale every
             * command down to zero later. */
            throw std::invalid_argument(
                "maximum_wheel_speed_radps must be positive");
        }

        /* Load each wheel's fixed body-frame x position. */
        loadSixValues("wheel_x_m", {0.64, 0.64, 0.0, 0.0, -0.72, -0.72},
                      wheelXM);

        /* Load each wheel's fixed body-frame y position. */
        loadSixValues("wheel_y_m", {0.60, -0.60, 0.60, -0.60, 0.60, -0.60},
                      wheelYM);

        /* Load each wheel's fixed drive-direction sign convention. */
        loadSixValues("drive_direction_multipliers",
                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                      driveDirectionMultipliers);

        /* Publisher for the public six-wheel joint command. */
        p_wheelCommandPublisher =
            create_publisher<actuator_msgs::msg::Actuators>(
                wheelJointStatesTopic, rclcpp::QoS(10));

        /* Every incoming velocity command triggers
         * handleVelocityCommandCallBack(). */
        p_velocityCommandSubscription =
            create_subscription<geometry_msgs::msg::Twist>(
                velocityTopic, rclcpp::QoS(10),
                [this](geometry_msgs::msg::Twist::ConstSharedPtr p_message)
                { handleVelocityCommandCallBack(*p_message); });

        /* Record the resolved topic names once at start-up for operators
         * inspecting the node's log. */
        RCLCPP_INFO(get_logger(), "Ackermann controller: %s -> %s",
                    velocityTopic.c_str(), wheelJointStatesTopic.c_str());
    }

    /*! @brief Releases the node's ROS interfaces. */
    ~AckermannControllerNode() override = default;

    AckermannControllerNode(const AckermannControllerNode &otherNode_in) =
        delete;
    AckermannControllerNode &operator=(
        const AckermannControllerNode &otherNode_in) = delete;
    AckermannControllerNode(AckermannControllerNode &&otherNode_in) = delete;
    AckermannControllerNode &operator=(
        AckermannControllerNode &&otherNode_in) = delete;

  private:
    /* ---------------------------------------------------------------------- *
     * CALLBACK METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Converts one commanded body twist into six-wheel
     *                   steering angles and speeds and publishes them.
     *
     * When the fastest wheel this twist would require exceeds
     * maximumWheelSpeedRadps, every wheel's speed is scaled down by the
     * same factor before publishing (see this class's own doc comment).
     *
     * @param[in]       message_in
     *                   Commanded body-frame velocity: linear.x and
     *                   linear.y in m/s, angular.z in rad/s.
     */
    void handleVelocityCommandCallBack(const geometry_msgs::msg::Twist &message_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Declares a six-element parameter, falling back to the
     *                   provided defaults when the configured value does not
     *                   contain exactly six entries.
     *
     * @param[in]       name_in
     *                   Externally controlled ROS parameter name.
     * @param[in]       defaults_in
     *                   Default six values used verbatim when validation
     *                   fails.
     * @param[out]      values_out
     *                   Six validated (or default) values in wheel order.
     */
    void loadSixValues(const std::string &name_in,
                       const std::vector<double> &defaults_in,
                       std::array<double, 6> &values_out);

    /*!
     * @brief           Computes the steering angle one wheel needs to roll
     *                   without slipping toward its required contact-point
     *                   velocity.
     *
     * @param[in]       vxMps_in
     *                   Commanded body-frame forward velocity in m/s.
     * @param[in]       vyMps_in
     *                   Commanded body-frame lateral velocity in m/s.
     * @param[in]       yawRateRadps_in
     *                   Commanded body-frame yaw rate in rad/s.
     * @param[in]       wheelXM_in
     *                   Wheel's fixed body-frame x position in metres.
     * @param[in]       wheelYM_in
     *                   Wheel's fixed body-frame y position in metres.
     *
     * @return          Steering angle in radians, in (-pi, pi].
     */
    static double computeSteeringAngle(double vxMps_in, double vyMps_in,
                                       double yawRateRadps_in,
                                       double wheelXM_in, double wheelYM_in);

    /*!
     * @brief           Computes the angular drive rate one wheel needs to
     *                   roll without slipping at its required contact-point
     *                   speed.
     *
     * @param[in]       vxMps_in
     *                   Commanded body-frame forward velocity in m/s.
     * @param[in]       vyMps_in
     *                   Commanded body-frame lateral velocity in m/s.
     * @param[in]       yawRateRadps_in
     *                   Commanded body-frame yaw rate in rad/s.
     * @param[in]       wheelXM_in
     *                   Wheel's fixed body-frame x position in metres.
     * @param[in]       wheelYM_in
     *                   Wheel's fixed body-frame y position in metres.
     * @param[in]       wheelRadiusM_in
     *                   Wheel radius in metres.
     *
     * @return          Non-negative angular drive rate in rad/s.
     */
    static double computeWheelSpeedRadps(double vxMps_in, double vyMps_in,
                                         double yawRateRadps_in,
                                         double wheelXM_in, double wheelYM_in,
                                         double wheelRadiusM_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Publishes the public six-wheel joint command.
     */
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
        p_wheelCommandPublisher;

    /*!
     * @brief       Subscribes to the commanded body-frame velocity.
     */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        p_velocityCommandSubscription;

    /*!
     * @brief       Per-wheel body-frame x position in metres.
     */
    std::array<double, 6> wheelXM{};

    /*!
     * @brief       Per-wheel body-frame y position in metres.
     */
    std::array<double, 6> wheelYM{};

    /*!
     * @brief       Per-wheel drive-direction sign convention, dimensionless.
     */
    std::array<double, 6> driveDirectionMultipliers{};

    /*!
     * @brief       Common wheel radius in metres.
     */
    double wheelRadiusM{0.1425};

    /*!
     * @brief       Alpha's real-hardware maximum drive-wheel speed in
     *              rad/s; see its declare_parameter call for the three
     *              layers this is enforced at.
     */
    double maximumWheelSpeedRadps{0.14};
};

} /* namespace control::ackermann_controller */

#endif /* LUNAR_SIMULATOR_CONTROL_ACKERMANN_CONTROLLER_NODE_H */
