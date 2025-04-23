#pragma once

#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "drone_interfaces/msg/report.hpp"
#include "drone_interfaces/msg/mission.hpp"

enum class DroneState {
    READY = 0,
    FLY   = 1,
    ERROR = 2
};

namespace DroneComposition {

class Drone : public rclcpp::Node {
public:
    using MsgMissionT     = drone_interfaces::msg::Mission;
    using MsgMissionPtrT  = MsgMissionT::SharedPtr;

    using MsgReportT      = drone_interfaces::msg::Report;
    using MsgReportPtrT   = MsgReportT::SharedPtr;
    
    using MsgOdometryT    = nav_msgs::msg::Odometry;
    using MsgOdometryPtrT = MsgOdometryT::SharedPtr;

    using Point   = Eigen::Vector3d;
    using Vector3 = Eigen::Vector3d;

public:
    explicit Drone(const rclcpp::NodeOptions & options);

    static std::string getReportTopic(const std::string& name) {
        return name + "/report";
    }

    static std::string getMissionTopic(const std::string& name) {
        return name + "/mission";
    }

    static std::string getOdometryTopic(const std::string& name) {
        return name + "/odometry";
    }

    std::string getReportTopic() { return std::string{this->get_name()} + "/report"; }
    std::string getMissionTopic() { return std::string{this->get_name()} + "/mission"; }
    std::string getOdometryTopic() { return std::string{this->get_name()} + "/odometry"; }

    Point getCurrentPosition() const { return currentPosition; }

    std::string getModel() const { return model; }

private:
    void flight(Point targetPoint, Vector3 velocity);

    void missionHandler(const MsgMissionPtrT msg);

    void sendReport(DroneState state);

private:
    int id;
    std::string model;
    std::string realName;
    Point currentPosition;
    rclcpp::Publisher<MsgReportT>::SharedPtr     reportPublisher;
    rclcpp::Publisher<MsgOdometryT>::SharedPtr   odometryPublisher;
    rclcpp::Subscription<MsgMissionT>::SharedPtr missionSubscription;
};

}