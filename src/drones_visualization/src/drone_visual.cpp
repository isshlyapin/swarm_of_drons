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
    initMarkers();
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

    markerArrayPublisher = this->create_publisher<MsgMarkerArrayT>(
        getMarkerTopic(),
        marker_qos
    );

    pathPublisher = this->create_publisher<MsgPathT>(
        getPathTopic(),
        path_qos
    );
}

void DroneVisual::initMarkers() {
    int drone_numeric_id = std::stoi(drone_id.substr(drone_id.rfind('_') + 1));
    
    // Инициализация сферы дрона
    sphere_marker.ns = "drones";
    sphere_marker.id = drone_numeric_id;
    sphere_marker.type = MsgMarkerT::SPHERE;
    sphere_marker.action = MsgMarkerT::ADD;
    sphere_marker.scale.x = 10.0;
    sphere_marker.scale.y = 10.0;
    sphere_marker.scale.z = 10.0;
    sphere_marker.color.r = 1.0f;
    sphere_marker.color.g = 0.0f;
    sphere_marker.color.b = 0.0f;
    sphere_marker.color.a = 1.0f;
    sphere_marker.lifetime = rclcpp::Duration(0, 0);

    // Инициализация текстовой метки
    text_marker.ns = "drone_labels";
    text_marker.id = drone_numeric_id;
    text_marker.type = MsgMarkerT::TEXT_VIEW_FACING;
    text_marker.action = MsgMarkerT::ADD;
    text_marker.scale.z = 8.0;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    text_marker.lifetime = rclcpp::Duration(0, 0);
    text_marker.text = drone_id;

    // Добавляем маркеры в массив
    marker_array.markers.clear();
    marker_array.markers.push_back(sphere_marker);
    marker_array.markers.push_back(text_marker);
}

void DroneVisual::updateMarkers(const MsgDroneOdometryPtrT msg) {
    // Обновляем сферу
    sphere_marker.header = msg->header;
    sphere_marker.pose.position.x = msg->pose.pose.position.x;
    sphere_marker.pose.position.y = msg->pose.pose.position.y;
    sphere_marker.pose.position.z = msg->pose.pose.position.z;

    // Обновляем текстовую метку (размещаем выше сферы)
    text_marker.header = msg->header;
    text_marker.pose.position.x = msg->pose.pose.position.x;
    text_marker.pose.position.y = msg->pose.pose.position.y;
    text_marker.pose.position.z = msg->pose.pose.position.z - sphere_marker.scale.z * 1.2;

    // Обновляем массив маркеров
    marker_array.markers[0] = sphere_marker;
    marker_array.markers[1] = text_marker;
}

void DroneVisual::updatePath(const MsgDroneOdometryPtrT msg) {
    path.header = msg->header;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    path.poses.push_back(pose);
}

void DroneVisual::odometryHandler(const MsgDroneOdometryPtrT msg) {
        updateMarkers(msg);
        markerArrayPublisher->publish(marker_array);

        updatePath(msg);
        pathPublisher->publish(path);
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