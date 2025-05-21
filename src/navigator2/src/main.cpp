#include <rclcpp/rclcpp.hpp>

#include "navigator2/navigator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto navigator_node = std::make_shared<Navigator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(navigator_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
