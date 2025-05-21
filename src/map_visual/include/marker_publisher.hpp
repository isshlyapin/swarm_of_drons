#pragma once

#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>


class MarkerPublisher : public rclcpp::Node {
public:
    using Point = Eigen::Vector3d;

    using MsgMarkerT = visualization_msgs::msg::Marker;
    
public:
    MarkerPublisher();

private:
    void communicationInit();

    std::string getDronePortsTopic() {
        return std::string{"/drone_ports"};
    }

    std::string getControlPointsTopic() {
        return std::string{"/control_points"};
    }

    std::string getTextMarkersTopic() {
        return std::string{"/text_markers"};
    }

    void mapInit();
    
    void publishMarkers();

    MsgMarkerT createMarker(const std::string &index, const Point &point);
    MsgMarkerT createTextMarker(const std::string &index, const Point &point);

  private:
    std::vector<MsgMarkerT> markers;
    std::vector<MsgMarkerT> textMarkers_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dronePortsPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr controlPointsPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr textPublisher_;
};
