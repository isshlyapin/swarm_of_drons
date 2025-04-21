#include "marker_publisher.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        std::cerr << "Usage: marker_publisher <path_to_csv>" << std::endl;
        return 1;
    }

    auto node = std::make_shared<MarkerPublisher>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
