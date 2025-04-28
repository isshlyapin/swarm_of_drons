#include <fastcsv/csv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "marker_publisher.hpp"

using namespace std::chrono_literals;

MarkerPublisher::MarkerPublisher()
: Node("marker_publisher")
{
    declare_parameter("drone_ports_qos", 25);
    declare_parameter("control_points_qos", 25);

    declare_parameter("map_file", "");

    communicationInit();

    mapInit();
}

void MarkerPublisher::communicationInit() {
    int drone_ports_qos = get_parameter("drone_ports_qos").as_int();
    int control_points_qos = get_parameter("control_points_qos").as_int();

    dronePortsPublisher = this->create_publisher<MsgMarkerT>(
        getDronePortsTopic(), 
        drone_ports_qos
    );
    controlPointsPublisher = this->create_publisher<MsgMarkerT>(
        getControlPointsTopic(), 
        control_points_qos
    );
    timer_ = this->create_wall_timer(
        3s,
        [this]() {
            this->publishMarkers();
        }
    );
}

void MarkerPublisher::mapInit() {
    std::string csv_path_ = get_parameter("map_file").as_string();
    if (csv_path_.empty()) {
        RCLCPP_ERROR(get_logger(), "Map file path is empty");
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Map file path is empty"
        );
    }

    io::CSVReader<4> reader(csv_path_);
    reader.read_header(io::ignore_extra_column, "index", "x", "y", "z");
    std::string index;
    double x, y, z;
    while (reader.read_row(index, x, y, z)) {
        markers.push_back(createMarker(index, Point{x, y, z}));
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu entries from CSV", markers.size());
}

MarkerPublisher::MsgMarkerT MarkerPublisher::createMarker(const std::string &index, const Point &point) {
    MsgMarkerT marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = index[0] == 'd' ? "drone_ports" : "control_points";
    marker.id = std::stoi(index.substr(1));
    marker.action = MsgMarkerT::ADD;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();

    if (index[0] == 'd') {
        marker.type = MsgMarkerT::CUBE;
        marker.scale.x = 10.0;
        marker.scale.y = 10.0;
        marker.scale.z = 5.0;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.8f;
    } else if (index[0] == 'p') {
        marker.type = MsgMarkerT::CYLINDER;
        marker.scale.x = 10.0;
        marker.scale.y = 10.0;
        marker.scale.z = 5.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown marker type: %s", index.c_str());
    }

    return marker;
}

void MarkerPublisher::publishMarkers() {
    rclcpp::Time now = this->now();

    for (auto &marker : markers) {
        marker.header.stamp = now;
        if (marker.ns == "drone_ports") {
            dronePortsPublisher->publish(marker);
        } else if (marker.ns == "control_points") {
            controlPointsPublisher->publish(marker);
        }        
    }
}