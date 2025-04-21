#include <cstddef>
#include <string>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mission_publisher.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Получаем параметры командной строки
    std::string mission_file = "";
    int publish_all = 0;
    int mission_index = -1;
    int interval = 1000;
    
    // Обрабатываем аргументы командной строки
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--file" || arg == "-f") {
            if (i + 1 < argc) {
                mission_file = argv[++i];
            }
        } else if (arg == "--all" || arg == "-a") {
            publish_all = 1;
        } else if (arg == "--index" || arg == "-i") {
            if (i + 1 < argc) {
                mission_index = std::stoi(argv[++i]);
            }
        } else if (arg == "--interval" || arg == "-t") {
            if (i + 1 < argc) {
                interval = std::stoi(argv[++i]);
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  -f, --file FILE    Path to the mission JSON file" << std::endl;
            std::cout << "  -a, --all          Publish all missions with interval" << std::endl;
            std::cout << "  -i, --index N      Publish only mission with index N" << std::endl;
            std::cout << "  -t, --interval MS  Interval between missions in milliseconds (default: 1000)" << std::endl;
            std::cout << "  -h, --help         Show this help message" << std::endl;
            return 0;
        }
    }
    
    if (mission_file.empty()) {
        std::cerr << "Error: Mission file not specified. Use --file option." << std::endl;
        return 1;
    }
    
    auto node = std::make_shared<MissionPublisher>("mission_publisher");
    
    // Загружаем миссии из файла
    if (!node->loadMissions(mission_file)) {
        return 1;
    }
    
    // Публикуем миссии
    if (publish_all) {
        node->publishAllMissions(interval);
    } else if (mission_index >= 0) {
        if (!node->publishMission(static_cast<size_t>(mission_index))) {
            return 1;
        }
    } else {
        std::cerr << "Error: Either --all or --index must be specified." << std::endl;
        return 1;
    }
    
    // Запускаем спиннер для обработки сообщений
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}