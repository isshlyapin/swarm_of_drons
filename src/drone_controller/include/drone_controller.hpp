#pragma once

#include <thread>
#include <future>
#include <unordered_map>

#include "rclcpp/node.hpp"
#include <rclcpp/rate.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/duration.hpp>
#include "rclcpp/publisher.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/subscription.hpp"

#include "drone_interfaces/msg/mission.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

#include "csv.hpp"
#include "drone.hpp"

enum class DroneState {
    READY = 0,
    FLY   = 1
};

struct DroneInfo {
    DroneState state;
    Drone::SharedPtr drone;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr       reportSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr    locationSubscription;
    rclcpp::Publisher<drone_interfaces::msg::Mission>::SharedPtr missionPublisher;
};

class DroneController : public rclcpp::Node {
private:
    using MsgGlobalMissionT    = drone_interfaces::msg::GlobalMission;
    using MsgGlobalMissionPtrT = MsgGlobalMissionT::SharedPtr;

public:
    DroneController(const std::string& nodeName) : Node(nodeName) {
        RCLCPP_INFO(this->get_logger(), "DroneController %s: Created", nodeName.c_str());

        set_parameter(rclcpp::Parameter("use_sim_time", true));

        RCLCPP_INFO(this->get_logger(), "DroneController %s: Global Mission Subscription Init", nodeName.c_str());
        globalMissionSubscription = create_subscription<MsgGlobalMissionT>(
            "global_mission",
            100,
            [this](const MsgGlobalMissionPtrT msg) {
                this->globalMissionHandler(msg);
            }
        );
    }

    void init(std::string pathToDronesCSV) {
        io::CSVReader<6> in(pathToDronesCSV);
        in.read_header(io::ignore_extra_column, "model", "id", "state", "x", "y", "z");

        std::string model;
        int id;
        int state;
        double x, y, z;

        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        while (in.read_row(model, id, state, x, y, z)) {
            DroneInfo droneInfo;
            auto drone = std::make_shared<Drone>(model, id, Point{x, y, z}, 100);
            droneInfo.drone = drone;
            droneInfo.state = static_cast<DroneState>(state);
            droneInfo.missionPublisher = this->create_publisher<drone_interfaces::msg::Mission>(
                drone->getMissionTopic(),
                100
            );
            droneInfo.reportSubscription = this->create_subscription<std_msgs::msg::UInt16>(
                drone->getReportTopic(),
                100,
                [this](const std_msgs::msg::UInt16::SharedPtr msg) {
                    this->reportHandler(msg);
                }
            );
            droneInfo.locationSubscription = this->create_subscription<geometry_msgs::msg::Pose>(
                drone->getLocationTopic(),
                100,
                [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
                    this->locationHandler(msg);
                }
            );
            drones[id] = droneInfo;

            executor->add_node(drone);
        }

        executorThread = std::thread(
            [this]() {
                RCLCPP_INFO(this->get_logger(), "DroneController: Executor Thread Started");
                executor->spin();
            }
        );
    }

private:
    void globalMissionHandler(const MsgGlobalMissionPtrT msg) {
        RCLCPP_INFO(this->get_logger(), "DroneController: Global Mission Received");
        if (drones.find(msg->drone_id) != drones.end()) {
            drone_interfaces::msg::Mission mission;
            mission.poses      = msg->poses;
            mission.start_time = msg->start_time;
            mission.velocities = msg->velocities;  
    
            std::thread missionThread(
                [this, msg, mission]() {
                    if (this->now() < msg->start_time) {
                        RCLCPP_INFO(this->get_logger(), "DroneController: Waiting for start time");
                        rclcpp::Duration dtime = rclcpp::Time(msg->start_time) - this->now();
                        rclcpp::Rate rate(dtime, this->get_clock());
                        rate.sleep();
                    }
                    RCLCPP_INFO(this->get_logger(), "DroneController: Mission for Drone published");
                    drones[msg->drone_id].missionPublisher->publish(mission);
                }
            );
            missionThread.detach();
        }
    }

    void reportHandler(const std_msgs::msg::UInt16::SharedPtr msg) {}

    void locationHandler(const geometry_msgs::msg::Pose::SharedPtr msg) {}

private:
    std::thread executorThread;
    rclcpp::Executor::SharedPtr executor;
    std::unordered_map<int, DroneInfo> drones;
    rclcpp::Subscription<MsgGlobalMissionT>::SharedPtr globalMissionSubscription;    
};
