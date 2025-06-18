#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drone_composition/drone.hpp"

class DroneVisual : public rclcpp::Node {
public:
    using MsgDroneOdometryT    = DroneComposition::Drone::MsgOdometryT;
    using MsgDroneOdometryPtrT = DroneComposition::Drone::MsgOdometryPtrT;
    
    using MsgDroneReportT    = DroneComposition::Drone::MsgReportT;
    using MsgDroneReportPtrT = DroneComposition::Drone::MsgReportPtrT;

    using MsgMarkerT    = visualization_msgs::msg::Marker;
    using MsgMarkerPtrT = MsgMarkerT::SharedPtr;
    
    using MsgMarkerArrayT    = visualization_msgs::msg::MarkerArray;
    using MsgMarkerArrayPtrT = MsgMarkerArrayT::SharedPtr;
   
    using MsgPathT    = nav_msgs::msg::Path;
    using MsgPathPtrT = MsgPathT::SharedPtr;
public:
    DroneVisual(const std::string &drone_id);

    std::string getMarkerTopic() const {
        return std::string{drone_id} + "/visual/markers";
    }

    std::string getPathTopic() const {
        return drone_id + "/visual/path";
    }

    std::string getTextMarkersTopic() {
        return std::string{"/text_markers"};
    }

private:
    void communicationInit();

    void initMarkers();

    void updateMarkers(const MsgDroneOdometryPtrT msg);

    void updatePath(const MsgDroneOdometryPtrT msg);

    void odometryHandler(const MsgDroneOdometryPtrT msg);

    void reportHandler(const MsgDroneReportPtrT msg);

private:
    std::string drone_id;

    MsgPathT path;
    MsgMarkerT sphere_marker;
    MsgMarkerT text_marker;
    MsgMarkerArrayT marker_array;
    
    rclcpp::Publisher<MsgPathT>::SharedPtr pathPublisher;
    rclcpp::Publisher<MsgMarkerArrayT>::SharedPtr markerArrayPublisher;
    rclcpp::Subscription<MsgDroneReportT>::SharedPtr reportSubscription;
    rclcpp::Subscription<MsgDroneOdometryT>::SharedPtr odometrySubscription;
};