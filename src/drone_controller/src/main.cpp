#include "rclcpp/executors.hpp"

#include "../include/drone_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<DroneController>("default_drone_controller");
    controller->init(
        "/workspaces/swarm_of_drons/src/drone_controller/image/swarm_data_drons.csv"
    );

    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}