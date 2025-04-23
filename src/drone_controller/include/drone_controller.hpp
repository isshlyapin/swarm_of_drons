#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "drone_interfaces/msg/report.hpp"
#include "drone_interfaces/msg/mission.hpp"
#include "drone_interfaces/msg/global_mission.hpp"
#include "navigator_interfaces/srv/free_drone.hpp"

#include "drone.hpp"

struct DroneInfo {
    std::shared_ptr<Drone> drone;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odometrySubscription;
    rclcpp::Publisher<drone_interfaces::msg::Mission>::SharedPtr   missionPublisher;
    rclcpp::Subscription<drone_interfaces::msg::Report>::SharedPtr reportSubscription;

    // DroneInfo(std::shared_ptr<Drone> dronePtr)
    //     : drone(dronePtr), state(DroneState::READY), count_missions(0) {}

    void decrementMissionCount() {
        // countMissionsMutex.lock();
        --count_missions;
        // countMissionsMutex.unlock();
    }

    void incrementMissionCount() {
        // countMissionsMutex.lock();
        ++count_missions;
        // countMissionsMutex.unlock();
    }

    int getMissionCount() const {
        // countMissionsMutex.lock();
        int currentCount = count_missions;
        // countMissionsMutex.unlock();
        return currentCount;
    }

    void setState(DroneState newState) {
        // stateMutex.lock();
        state = newState;
        // stateMutex.unlock();
    }

    DroneState getState() const {
        // stateMutex.lock();
        DroneState currentState = state;
        // stateMutex.unlock();
        return currentState;
    }
    
private:
    DroneState state;
    int count_missions;
    // mutable std::mutex countMissionsMutex;
    // mutable std::mutex stateMutex;
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

    using SrvFreeDroneT    = navigator_interfaces::srv::FreeDrone;

    using Point = Eigen::Vector3d;

public:
    DroneController(const std::string& nodeName);

    std::vector<Drone::SharedPtr> getDrones() const;
private:
    void init(std::string pathToDronesCSV);
    
    void globalMissionHandler(const MsgGlobalMissionPtrT msg);

    void reportHandler(const MsgReportPtrT msg);

    void odometryHandler(const MsgOdometryPtrT msg);

    void freeDroneHandler(
        const std::shared_ptr<SrvFreeDroneT::Request> request,
              std::shared_ptr<SrvFreeDroneT::Response> response
    );

    std::vector<int> findFreeDrone();

    int getNearDroneID(std::vector<int>& v, Point target);

private:
    std::thread executorThread;
    rclcpp::Executor::SharedPtr executor;
    std::unordered_map<int, DroneInfo> drones;
    rclcpp::Subscription<MsgGlobalMissionT>::SharedPtr globalMissionSubscription;
    rclcpp::Service<SrvFreeDroneT>::SharedPtr freeDroneService;
};
