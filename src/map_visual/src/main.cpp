#include <rclcpp/rclcpp.hpp>

#include "marker_publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MarkerPublisher>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}
