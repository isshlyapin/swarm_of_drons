#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <fstream>
#include <thread>

#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

using json = nlohmann::json;

class MissionPublisher : public rclcpp::Node {
public:
    using GlobalMissionMsg = drone_interfaces::msg::GlobalMission;

    /**
     * @brief Конструктор узла публикации миссий
     * @param node_name Имя узла
     */
    explicit MissionPublisher(const std::string& node_name);

    /**
     * @brief Загрузка миссий из JSON-файла
     * @param mission_file Путь к файлу миссий
     * @return true если загрузка успешна, false в противном случае
     */
    bool loadMissions(const std::string& mission_file);

    /**
     * @brief Публикация миссии по индексу
     * @param mission_index Индекс миссии в загруженном списке
     * @return true если миссия была опубликована, false в противном случае
     */
    bool publishMission(size_t mission_index);

    /**
     * @brief Публикация всех загруженных миссий с заданным интервалом
     * @param interval_ms Интервал между публикациями в миллисекундах
     */
    void publishAllMissions(int interval_ms);

    /**
     * @return Количество загруженных миссий
     */
    size_t getMissionCount() const;

private:
    /**
     * @brief Структура для хранения миссии
     */
    struct Mission {
        int drone_id;
        rclcpp::Time start_time;
        std::vector<float> velocities;

        std::vector<geometry_msgs::msg::Pose> poses;
    };

    /**
     * @brief Конвертация JSON в объект миссии
     * @param mission_json JSON-объект с описанием миссии
     * @return Структура Mission
     */
    Mission parseMissionFromJson(const json& mission_json);

    /**
     * @brief Конвертация объекта миссии в сообщение GlobalMission
     * @param mission Объект миссии
     * @return Сообщение GlobalMission
     */
    GlobalMissionMsg createGlobalMissionMsg(const Mission& mission);

private:
    std::vector<Mission> missions_;
    rclcpp::Publisher<GlobalMissionMsg>::SharedPtr mission_publisher_;
};