#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "drone_composition/drone.hpp"

class DroneVisual : public rclcpp::Node {
public:
    using MsgDroneOdometryT    = DroneComposition::Drone::MsgOdometryT;
    using MsgDroneOdometryPtrT = DroneComposition::Drone::MsgOdometryPtrT;
    
    using MsgDroneReportT    = DroneComposition::Drone::MsgReportT;
    using MsgDroneReportPtrT = DroneComposition::Drone::MsgReportPtrT;

    using MsgMarkerT    = visualization_msgs::msg::Marker;
    using MsgMarkerPtrT = MsgMarkerT::SharedPtr;
   
    using MsgPathT    = nav_msgs::msg::Path;
    using MsgPathPtrT = MsgPathT::SharedPtr;
public:
    DroneVisual(const std::string &drone_id);

    std::string getMarkerTopic() const {
        return std::string{drone_id} + "/visual/marker";
    }

    std::string getPathTopic() const {
        return drone_id + "/visual/path";
    }

    std::string getTextMarkersTopic() {
        return std::string{"/text_markers"};
    }

private:
    void communicationInit();

    void initMarker();

    void updateMarker(const MsgDroneOdometryPtrT msg);

    void updatePath(const MsgDroneOdometryPtrT msg);

    void odometryHandler(const MsgDroneOdometryPtrT msg);

    void reportHandler(const MsgDroneReportPtrT msg);

private:
    std::string drone_id;

    MsgPathT path;
    MsgMarkerT marker;
    MsgMarkerT text_marker;
    
    rclcpp::Publisher<MsgPathT>::SharedPtr pathPublisher;
    rclcpp::Publisher<MsgMarkerT>::SharedPtr markerPublisher;
    rclcpp::Subscription<MsgDroneReportT>::SharedPtr reportSubscription;
    rclcpp::Subscription<MsgDroneOdometryT>::SharedPtr odometrySubscription;
};