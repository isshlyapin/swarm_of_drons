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

class Drone : public rclcpp::Node {
private:
    using MsgMissionT     = drone_interfaces::msg::Mission;
    using MsgMissionPtrT  = MsgMissionT::SharedPtr;

    using MsgReportT      = drone_interfaces::msg::Report;
    using MsgReportPtrT   = MsgReportT::SharedPtr;
    
    using MsgOdometryT    = nav_msgs::msg::Odometry;
    using MsgOdometryPtrT = MsgOdometryT::SharedPtr;

    using Point   = Eigen::Vector3d;
    using Vector3 = Eigen::Vector3d;

public:
    Drone(const std::string &node_name, int id);
    
    Drone(const std::string &model, int id,  Point pos);

    std::string getReportTopic() { return realName + "/report"; }
    std::string getMissionTopic() { return realName + "/mission"; }
    std::string getOdometryTopic() { return realName + "/odometry"; }

private:
    void flight(Point targetPoint, Vector3 velocity);

    void missionHandler(const MsgMissionPtrT msg);

    void sendReport(DroneState state);

private:
    int id;
    std::string realName;
    Point currentPosition;
    rclcpp::Publisher<MsgReportT>::SharedPtr     reportPublisher;
    rclcpp::Publisher<MsgOdometryT>::SharedPtr   odometryPublisher;
    rclcpp::Subscription<MsgMissionT>::SharedPtr missionSubscription;
};
