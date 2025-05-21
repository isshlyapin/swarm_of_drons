#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "drone_visual.hpp"

DroneVisual::DroneVisual(const std::string &drone_id)
: Node(
    drone_id + "_" + "v",
    rclcpp::NodeOptions().use_global_arguments(false)
  ),
  drone_id(drone_id)
{
    declare_parameter("vis_odm_qos", 25);
    declare_parameter("vis_rep_qos", 25);
    declare_parameter("vis_path_qos", 25);
    declare_parameter("vis_marker_qos", 25);

    declare_parameter("flight_rate", 50.0);
    declare_parameter("time_scale", 1.0);

    communicationInit();
    initMarker();
}

void DroneVisual::communicationInit() {
    int odm_qos = get_parameter("vis_odm_qos").as_int();
    int rep_qos = get_parameter("vis_rep_qos").as_int();
    int path_qos = get_parameter("vis_path_qos").as_int();
    int marker_qos = get_parameter("vis_marker_qos").as_int();

    odometrySubscription = this->create_subscription<MsgDroneOdometryT>(
        DroneComposition::Drone::getOdometryTopic(drone_id), 
        odm_qos,
        [this](const MsgDroneOdometryPtrT msg) {
            this->odometryHandler(msg);
        }
    );

    reportSubscription = this->create_subscription<MsgDroneReportT>(
        DroneComposition::Drone::getReportTopic(drone_id),
        rep_qos,
        [this](const MsgDroneReportPtrT msg) {
            this->reportHandler(msg);
        }
    );

    markerPublisher = this->create_publisher<MsgMarkerT>(
        getMarkerTopic(),
        marker_qos
    );

    pathPublisher = this->create_publisher<MsgPathT>(
        getPathTopic(),
        path_qos
    );
}

void DroneVisual::initMarker() {
    marker.ns = "drones";
    marker.id = std::stoi(drone_id.substr(drone_id.rfind('_') + 1));
    marker.type = MsgMarkerT::SPHERE;
    marker.action = MsgMarkerT::ADD;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.scale.z = 10.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);

    // Текстовая метка
    text_marker.ns = "drones";
    text_marker.id = marker.id + 1000; // уникальный id для текста
    text_marker.type = MsgMarkerT::TEXT_VIEW_FACING;
    text_marker.action = MsgMarkerT::ADD;
    text_marker.scale.z = 8.0;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    text_marker.lifetime = rclcpp::Duration(0, 0);
    text_marker.text = drone_id;
}

void DroneVisual::updateMarker(const MsgDroneOdometryPtrT msg) {
    marker.header = msg->header;
    marker.pose.position.x = msg->pose.pose.position.x;
    marker.pose.position.y = msg->pose.pose.position.y;
    marker.pose.position.z = msg->pose.pose.position.z;

    // Текстовая метка — выше сферы
    text_marker.header = msg->header;
    text_marker.pose.position.x = msg->pose.pose.position.x;
    text_marker.pose.position.y = msg->pose.pose.position.y;
    text_marker.pose.position.z = msg->pose.pose.position.z - marker.scale.z * 1.2;
}

void DroneVisual::updatePath(const MsgDroneOdometryPtrT msg) {
    path.header = msg->header;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
}

void DroneVisual::odometryHandler(const MsgDroneOdometryPtrT msg) {
    static const int coeff = std::max(
        1, 
        static_cast<int>(
            get_parameter("flight_rate").as_double() / 30 / 
            get_parameter("time_scale").as_double()
        )
    );

    static int count = 0;

    if (count++ % coeff == 0) {
        updateMarker(msg);
        markerPublisher->publish(marker);
        markerPublisher->publish(text_marker);

        updatePath(msg);
        pathPublisher->publish(path);
    }
}

void DroneVisual::reportHandler(const MsgDroneReportPtrT msg) {
    if (!magic_enum::enum_contains<DroneState>(msg->state)) {
        RCLCPP_ERROR(get_logger(), "DroneVisual: Invalid state");
        return;
    } else {
        if (msg->state == magic_enum::enum_integer(DroneState::READY)) {
            path.poses.clear();
            pathPublisher->publish(path);    
        }
    }
}