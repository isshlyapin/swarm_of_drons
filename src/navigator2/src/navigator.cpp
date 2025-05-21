#include "navigator2/navigator.hpp"
#include "navigator2/drone.hpp"
#include <cstddef>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <string>

Navigator::Navigator(const rclcpp::NodeOptions& options)
: Node("navigator", options), map_(2.0)
{
    RCLCPP_INFO(get_logger(), "Navigator node started.");
    
    declare_parameter("graph_file", "");
    declare_parameter("edges_file", "");
    declare_parameter("missions_file", "");
    
    declare_parameter("global_mission_qos", 10);
    declare_parameter("free_drone_qos", 10);
    
    initCommunication();
    
    const std::string graph_file_path = get_parameter("graph_file").as_string();
    const std::string edges_file_path = get_parameter("edges_file").as_string();
    const std::string missions_file_path = get_parameter("missions_file").as_string();
    
    if (graph_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Graph file path not provided.");
        throw rclcpp::exceptions::InvalidParameterValueException("Graph file path not provided.");
    } 
    if (edges_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Edges file path not provided.");
        throw rclcpp::exceptions::InvalidParameterValueException("Edges file path not provided.");
    }
    if (missions_file_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Missions file path not provided.");
        throw rclcpp::exceptions::InvalidParameterValueException("Missions file path not provided.");
    }
   
    RCLCPP_INFO(get_logger(), "Loading graph");
    map_.loadGraphFromCSV(graph_file_path);

    RCLCPP_INFO(get_logger(), "Loading edges");
    map_.loadEdgesFromCSV(edges_file_path);

    RCLCPP_INFO(get_logger(), "Loading missions");    
    mission_manager_.loadMissionsFromCSV(missions_file_path);
    RCLCPP_INFO(get_logger(), "Navigator node initialized with graph, edges, and missions.");
}

Navigator::Navigator() : Navigator(rclcpp::NodeOptions()) {}

void Navigator::initCommunication()
{
    client_node_ = std::make_shared<rclcpp::Node>(
        "navigator_client_node",
        rclcpp::NodeOptions().use_global_arguments(false)
    );

    free_drone_client_ = client_node_->create_client<SrvFreeDroneT>(
        "free_drone_service",
        get_parameter("free_drone_qos").as_int()
    );

    global_mission_publisher_ = this->create_publisher<MsgGlobalMissionT>(
        "global_mission",
        get_parameter("global_mission_qos").as_int()
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() -> void {
            this->timerCallback();
        }
    );
}

void Navigator::timerCallback()
{
    RCLCPP_DEBUG(get_logger(), "Timer callback triggered.");
    if (mission_manager_.empty()) {
        RCLCPP_DEBUG(get_logger(), "No missions available.");
        return;
    }

    auto mission = mission_manager_.topInputMission();
    if (mission.time_appearance > this->now().seconds()) {
        RCLCPP_DEBUG(get_logger(), "Waits opening mission");
        return;
    }

    // Пытаемся получить информацию о свободном дроне от сервиса
    auto request = std::make_shared<SrvFreeDroneRequestT>();
    auto droneport_pose = map_.getVertexPosition(mission.id_from);
    request->pose_dronport.position.x = droneport_pose.x();
    request->pose_dronport.position.y = droneport_pose.y();
    request->pose_dronport.position.z = droneport_pose.z();

    while (!free_drone_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_DEBUG(this->get_logger(), "Wait for service interrupted");
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
    }

    auto result = free_drone_client_->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(client_node_, result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call free drone service.");
        return;
    }

    auto response = result.get();
    
    auto droneport_id = map_.getVertexId({
        response->pose_drone.position.x,
        response->pose_drone.position.y,
        response->pose_drone.position.z
    });

    if (droneport_id.empty()) {
        RCLCPP_ERROR(get_logger(), "Droneport ID not found.");
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Droneport position: (%lf, %lf, %lf)",
        response->pose_drone.position.x,
        response->pose_drone.position.y,
        response->pose_drone.position.z
    );
    RCLCPP_DEBUG(get_logger(), "Droneport ID: %s", droneport_id.c_str());
    Drone drone{
        droneport_id,
        DroneVMax{30},
        DroneVMin{20},
        DroneFreeTime{this->now().seconds() + 3}
    };

    if (droneport_id != mission.id_from) {
        auto map_mission = map_.generateMission(
            droneport_id, 
            mission.id_from,
            drone, 
            map_cfg::OptimalPathMode::MIN_TIME
        );

        publishGlobalMission(map_mission, response->id, "relocate");
        
        drone.setFreeTime(map_mission.timeFinish);
        drone.setVertexId(mission.id_from);
    }

    auto map_mission = map_.generateMission(
        mission.id_from, 
        mission.id_to,
        drone, 
        map_cfg::OptimalPathMode::MIN_TIME
    );
    publishGlobalMission(map_mission, response->id, "execute");

    mission_manager_.popInputMission();
    RCLCPP_INFO(get_logger(), "Mission published.");
}

void Navigator::publishGlobalMission(const Mission& mission, 
    uint32_t drone_id, const std::string& type)
{
    MsgGlobalMissionT msg;
    msg.start_time.sec = static_cast<int>(mission.timeStart);
    msg.start_time.nanosec = static_cast<int>(
        (mission.timeStart - msg.start_time.sec) * 1e9
    );
    msg.drone_id = drone_id;
    msg.mission_type = type;
    msg.id_from = mission.vertexes.front();
    msg.id_to = mission.vertexes.back();

    for (size_t i = 0; i < mission.vertexes.size(); ++i) {
        auto pose = map_.getVertexPosition(mission.vertexes[i]);
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose.x();
        pose_msg.position.y = pose.y();
        pose_msg.position.z = pose.z();

        msg.poses.push_back(pose_msg);
        if (i < mission.velocities.size()) {
            msg.velocities.push_back(mission.velocities[i]);
        }
    }

    RCLCPP_INFO(get_logger(), "Publishing global mission: %s from %s to %s", 
        type.c_str(), msg.id_from.c_str(), msg.id_to.c_str());

    global_mission_publisher_->publish(msg);
}