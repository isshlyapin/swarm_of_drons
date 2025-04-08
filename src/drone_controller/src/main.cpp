#include <rclcpp/qos.hpp>
#include "rclcpp/executors.hpp"
#include "../include/drone.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::QoS qos1(10);
    auto node = std::make_shared<Drone>("drone-xxx", 1, Point{0.0, 0.0, 0.0}, qos1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}