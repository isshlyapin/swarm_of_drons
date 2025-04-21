#include <rclcpp/rclcpp.hpp>

#include "drone_visual_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<DroneVisualController>("default_drone_visual_controller");
    controller->init(
        "/workspaces/swarm_of_drons/test/test1/drones.csv"
    );

    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}