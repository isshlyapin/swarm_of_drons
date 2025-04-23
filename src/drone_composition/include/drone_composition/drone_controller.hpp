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

#include "drone_composition/drone.hpp"

namespace DroneComposition {

struct DroneContext {
    void decrementMissionCount() {
        std::lock_guard<std::mutex> lock(countMissionsMutex);
        --countMissions;
    }

    void incrementMissionCount() {
        std::lock_guard<std::mutex> lock(countMissionsMutex);
        ++countMissions;
    }

    int getMissionCount() const {
        std::lock_guard<std::mutex> lock(countMissionsMutex);
        return countMissions;
    }

    void setState(DroneState newState) {
        std::lock_guard<std::mutex> lock(stateMutex);
        state = newState;
    }

    DroneState getState() const {
        std::lock_guard<std::mutex> lock(stateMutex);
        return state;
    }

    const Drone::Point& getPosition() const {
        std::lock_guard<std::mutex> lock(positionMutex);
        return position;
    }

    void setPosition(const Drone::Point& newPosition) {
        std::lock_guard<std::mutex> lock(positionMutex);
        position = newPosition;
    }

    bool isFree() const {
        std::lock_guard<std::mutex> lock1(stateMutex);
        std::lock_guard<std::mutex> lock2(countMissionsMutex);
        return state == DroneState::READY && countMissions == 0;
    }

    std::string getFullName() const {
        return model + "_" + std::to_string(id);
    }

public:
    int id;
    std::string model;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odometrySubscription;
    rclcpp::Publisher<drone_interfaces::msg::Mission>::SharedPtr   missionPublisher;
    rclcpp::Subscription<drone_interfaces::msg::Report>::SharedPtr reportSubscription;

private:
    DroneState state;
    int countMissions;
    Drone::Point position;

    mutable std::mutex stateMutex;
    mutable std::mutex positionMutex;
    mutable std::mutex countMissionsMutex;
};

class DroneController : public rclcpp::Node {
public:
    using MsgGlobalMissionT    = drone_interfaces::msg::GlobalMission;
    using MsgGlobalMissionPtrT = MsgGlobalMissionT::SharedPtr;

    using SrvFreeDroneT         = navigator_interfaces::srv::FreeDrone;
    using SrvFreeDroneRequestT  = SrvFreeDroneT::Request;
    using SrvFreeDroneResponseT = SrvFreeDroneT::Response;

    using Point   = Drone::Point;
    using Vector3 = Drone::Vector3;

public:
    explicit DroneController(const rclcpp::NodeOptions& options);

private:
    void communicationInit();

    void dronesContextInit();

    void addDroneContext(int id, const std::string& model, 
                         DroneState state, const Point& position);
    
    void globalMissionHandler(const MsgGlobalMissionPtrT msg);

    void reportHandler(const Drone::MsgReportPtrT msg);

    void odometryHandler(const Drone::MsgOdometryPtrT msg, int droneId);

    void freeDroneSrvHandler(
        const std::shared_ptr<SrvFreeDroneRequestT> request,
              std::shared_ptr<SrvFreeDroneResponseT> response
    );

    void findFreeDrones(std::vector<std::shared_ptr<DroneContext>>& v);

    std::shared_ptr<DroneContext> getNearDroneContext(
        const std::vector<std::shared_ptr<DroneContext>>& v, 
        const Point& target
    );

    void fieldFreeDroneSrvResponse(
        const std::shared_ptr<DroneContext> contextPtr,
              std::shared_ptr<SrvFreeDroneResponseT> response
    );


private:
    std::unordered_map<int, std::shared_ptr<DroneContext>> drones;
    
    rclcpp::Service<SrvFreeDroneT>::SharedPtr freeDroneService;
    rclcpp::Subscription<MsgGlobalMissionT>::SharedPtr globalMissionSubscription;
};

} // namespace DroneComposition
