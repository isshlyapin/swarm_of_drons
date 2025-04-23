#include <rclcpp/rclcpp.hpp>

#include "drone_visual_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<DroneVisualController>("default_drone_visual_controller");

    rclcpp::Executor::SharedPtr executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    executor->add_node(controller);
    for (const auto& node : controller->getDrones()) {
        executor->add_node(node);
    }

    executor->spin();

    rclcpp::shutdown();
    return 0;
}