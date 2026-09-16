#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("minimal_test");
    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>("test_topic", rclcpp::QoS(rclcpp::KeepLast(10)));
    RCLCPP_INFO(node->get_logger(), "Minimal test node running");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}