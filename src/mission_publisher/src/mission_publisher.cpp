#include <rcl/time.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>

#include "mission_publisher.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

MissionPublisher::MissionPublisher(const std::string& node_name)
    : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "MissionPublisher node initialized");
    
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    // Создаем издателя для глобальных миссий
    mission_publisher_ = this->create_publisher<GlobalMissionMsg>(
        "global_mission", 100);
}

bool MissionPublisher::loadMissions(const std::string& mission_file)
{
    RCLCPP_INFO(this->get_logger(), "Loading missions from %s", mission_file.c_str());
    
    try {
        // Открываем файл
        std::ifstream file(mission_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open mission file: %s", mission_file.c_str());
            return false;
        }
        
        // Парсим JSON
        nlohmann::json missions_json;
        file >> missions_json;
        
        // Очищаем предыдущие миссии
        missions_.clear();
        
        // Парсим каждую миссию
        for (const auto& mission_json : missions_json) {
            try {
                Mission mission = parseMissionFromJson(mission_json);
                missions_.push_back(mission);
                RCLCPP_INFO(this->get_logger(), "Loaded mission for drone ID: %d with %zu waypoints",
                           mission.drone_id, mission.poses.size());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse mission: %s", e.what());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu missions", missions_.size());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading missions: %s", e.what());
        return false;
    }
}

bool MissionPublisher::publishMission(size_t mission_index)
{
    if (mission_index >= missions_.size()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid mission index: %zu", mission_index);
        return false;
    }
    
    // Создаем сообщение из миссии
    auto msg = createGlobalMissionMsg(missions_[mission_index]);
    
    // Публикуем сообщение
    mission_publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "Published mission for drone ID: %d", msg.drone_id);
    return true;
}

void MissionPublisher::publishAllMissions(int interval_ms)
{
    RCLCPP_INFO(this->get_logger(), "Publishing all %zu missions with interval %d ms",
               missions_.size(), interval_ms);
    
    rclcpp::Rate rate(1000 / interval_ms, this->get_clock());
    for (size_t i = 0; i < missions_.size(); ++i) {
        publishMission(i);
        if (i < missions_.size() - 1) {
            // Ждем указанный интервал перед следующей публикацией
            rate.sleep();
        }
    }
}

size_t MissionPublisher::getMissionCount() const
{
    return missions_.size();
}

MissionPublisher::Mission MissionPublisher::parseMissionFromJson(const nlohmann::json& mission_json)
{
    Mission mission;
    
    // Парсим ID дрона
    if (!mission_json.contains("drone_id") || !mission_json["drone_id"].is_number_integer()) {
        throw std::runtime_error("Missing or invalid drone_id");
    }
    mission.drone_id = mission_json["drone_id"];
    
    // Парсим время начала
    if (!mission_json.contains("start_time") && !mission_json["start_time"].is_number_integer()) {
        throw std::runtime_error("Missing or invalid start_time");
    }
    int delay_sec = mission_json["start_time"];
    mission.start_time = rclcpp::Time(delay_sec, 0, RCL_ROS_TIME);
    
    // Парсим позиции
    if (!mission_json.contains("poses") || !mission_json["poses"].is_array()) {
        throw std::runtime_error("Missing or invalid waypoints array");
    }
    
    const auto& waypoints = mission_json["poses"];
    for (const auto& wp : waypoints) {
        geometry_msgs::msg::Pose pose;
        
        // Проверяем наличие необходимых полей
        if (!wp.contains("x") || !wp["x"].is_number()) {
            throw std::runtime_error("Missing or invalid position object");
        }
        if (!wp.contains("y") || !wp["y"].is_number()) {
            throw std::runtime_error("Missing or invalid position object");
        }        
        if (!wp.contains("z") || !wp["z"].is_number()) {
            throw std::runtime_error("Missing or invalid position object");
        }
        
        // Позиция
        pose.position.x = wp.value("x", 0.0);
        pose.position.y = wp.value("y", 0.0);
        pose.position.z = wp.value("z", 0.0);
                
        mission.poses.push_back(pose);
    }

    // Парсим скорости
    if (!mission_json.contains("velocities") || !mission_json["velocities"].is_array()) {
        throw std::runtime_error("Missing or invalid velocities array");
    }

    const auto& velocities = mission_json["velocities"];
    for (const auto& v : velocities) {

        mission.velocities.push_back(v.get<float>());
    }
    
    return mission;
}

MissionPublisher::GlobalMissionMsg MissionPublisher::createGlobalMissionMsg(const Mission& mission)
{
    GlobalMissionMsg msg;
    
    msg.poses      = mission.poses;
    msg.drone_id   = mission.drone_id;
    msg.velocities = mission.velocities;
    msg.start_time = mission.start_time;
    
    return msg;
}