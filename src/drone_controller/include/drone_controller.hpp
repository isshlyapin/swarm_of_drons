#pragma once

#include <string>
#include <thread>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "drone_interfaces/msg/mission.hpp"
#include "drone_interfaces/msg/global_mission.hpp"
#include "drone_interfaces/msg/report.hpp"

#include "drone.hpp"


struct DroneInfo {
    DroneState state;
    int count_missions;
    Drone::SharedPtr drone;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odometrySubscription;
    rclcpp::Publisher<drone_interfaces::msg::Mission>::SharedPtr   missionPublisher;
    rclcpp::Subscription<drone_interfaces::msg::Report>::SharedPtr reportSubscription;
};

class DroneController : public rclcpp::Node {
private:
    using MsgGlobalMissionT    = drone_interfaces::msg::GlobalMission;
    using MsgGlobalMissionPtrT = MsgGlobalMissionT::SharedPtr;

    using MsgMissionT    = drone_interfaces::msg::Mission;
    using MsgMissionPtrT = MsgMissionT::SharedPtr;

    using MsgReportT    = drone_interfaces::msg::Report;
    using MsgReportPtrT = MsgReportT::SharedPtr;    

    using MsgOdometryT    = nav_msgs::msg::Odometry;
    using MsgOdometryPtrT = MsgOdometryT::SharedPtr;

    using Point = Eigen::Vector3d;

public:
    DroneController(const std::string& nodeName);

    std::vector<Drone::SharedPtr> getDrones() const;
private:
    void init(std::string pathToDronesCSV);
    
    void globalMissionHandler(const MsgGlobalMissionPtrT msg);

    void reportHandler(const MsgReportPtrT msg);

    void odometryHandler(const MsgOdometryPtrT msg);

private:
    std::thread executorThread;
    rclcpp::Executor::SharedPtr executor;
    std::unordered_map<int, DroneInfo> drones;
    rclcpp::Subscription<MsgGlobalMissionT>::SharedPtr globalMissionSubscription;
};
