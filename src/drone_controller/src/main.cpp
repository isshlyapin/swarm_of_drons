#include <rclcpp/rclcpp.hpp>

#include "drone_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
     
        auto controller = std::make_shared<DroneController>("default_drone_controller");
     
        executor->add_node(controller);
        for (auto &node : controller->getDrones()) {
            executor->add_node(node);
        }
        executor->spin();

    } catch (const rclcpp::exceptions::InvalidParameterValueException &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "DroneController: %s", e.what());
        return 1;
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "DroneController: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}