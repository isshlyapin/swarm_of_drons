#include <rclcpp/rclcpp.hpp>

#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/subscription.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "drone_interfaces/msg/report.hpp"

#include <magic_enum.hpp>

enum class DroneState {
    READY = 0,
    FLY   = 1,
    ERROR = 2
};

class DroneVisual : public rclcpp::Node {
private:
    using MsgDroneOdometryT = nav_msgs::msg::Odometry;
    using MsgDroneOdometryPtrT = MsgDroneOdometryT::SharedPtr;

    using MsgDroneReportT = drone_interfaces::msg::Report;
    using MsgDroneReportPtrT = MsgDroneReportT::SharedPtr;

    using MsgMarkerT = visualization_msgs::msg::Marker;
    using MsgMarkerPtrT = MsgMarkerT::SharedPtr;
    using MsgPathT = nav_msgs::msg::Path;
    using MsgPathPtrT = MsgPathT::SharedPtr;
public:
    DroneVisual(const std::string &drone_id)
      : Node(
            drone_id + "_" + "v", 
            rclcpp::NodeOptions().use_global_arguments(false)
        ),
        drone_id(drone_id)
    {
        declare_parameter("vis_odm_qos", 25);
        declare_parameter("vis_path_qos", 25);
        declare_parameter("vis_marker_qos", 25);
        declare_parameter("vis_rep_qos", 25);

        int odm_qos = get_parameter("vis_odm_qos").as_int();
        int path_qos = get_parameter("vis_path_qos").as_int();
        int marker_qos = get_parameter("vis_marker_qos").as_int();
        int rep_qos = get_parameter("vis_rep_qos").as_int();

        odometrySubscription = this->create_subscription<MsgDroneOdometryT>(
            drone_id + "/odometry", 
            odm_qos,
            [this](const MsgDroneOdometryPtrT msg) {
                this->odometryHandler(msg);
            }
        );

        reportSubscription = this->create_subscription<MsgDroneReportT>(
            drone_id + "/report", 
            rep_qos,
            [this](const MsgDroneReportPtrT msg) {
                this->reportHandler(msg);
            }
        );

        markerPublisher = this->create_publisher<MsgMarkerT>(
            drone_id + "/visual/marker", 
            marker_qos
        );

        pathPublisher = this->create_publisher<MsgPathT>(
            drone_id + "/visual/path", 
            path_qos
        );
    }

private:
    void publishMarker(const MsgDroneOdometryPtrT msg) {
        // Create a marker message
        MsgMarkerT marker;
        marker.header = msg->header;
        marker.ns = "drones";
        marker.id = std::stoi(drone_id.substr(drone_id.rfind('_') + 1));
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = msg->pose.pose.position.x;
        marker.pose.position.y = msg->pose.pose.position.y;
        marker.pose.position.z = msg->pose.pose.position.z;
        marker.scale.x = 10.0; // Line width
        marker.scale.y = 10.0; // Line width
        marker.scale.z = 10.0; // Line width
        marker.color.r = 1.0f; // Red color
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f; // Fully opaque
        marker.lifetime = rclcpp::Duration(0, 0); // No lifetime

        // Publish the marker
        markerPublisher->publish(marker);
    }

    void odometryHandler(const MsgDroneOdometryPtrT msg) {
        // Handle the odometry message
        // RCLCPP_INFO(get_logger(), "Received odometry data");

        publishMarker(msg);

        path.header = msg->header;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        path.poses.push_back(pose);
        pathPublisher->publish(path);
    }

    void reportHandler(const MsgDroneReportPtrT msg) {
        // Handle the report message
        // RCLCPP_INFO(get_logger(), "Received report data");
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

private:
    std::string drone_id;
    nav_msgs::msg::Path path;
    rclcpp::Subscription<MsgDroneOdometryT>::SharedPtr odometrySubscription;
    rclcpp::Subscription<MsgDroneReportT>::SharedPtr reportSubscription;
    rclcpp::Publisher<MsgMarkerT>::SharedPtr markerPublisher;
    rclcpp::Publisher<MsgPathT>::SharedPtr pathPublisher;
};