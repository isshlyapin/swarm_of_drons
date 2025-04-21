#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "../include/csv.hpp"

#include "drone_interfaces/msg/global_mission.hpp"
#include "navigator_interfaces/srv/free_drone.hpp"
#include "../include/interface.hpp"

bool GraphInterface::CheckEqualPose(double x1, double y1, double x2, double y2, double z1, double z2) {
    double eps = 1e-2;
    if ( abs(x1 - x2) <= eps && abs(y2 - y1) <= eps && 
            abs(z2 - z1) <= eps) return true;
    return false;
}

void GraphInterface::fillMsgMission(Route& path, drone_interfaces::msg::GlobalMission& mission, int32_t drone_id) {
    mission.start_time = path.time_start;
    mission.velocities = path.velocities;
    mission.drone_id = drone_id;

    std::vector<std::vector<double>> poses;
    for (size_t i = 0; i < path.route.size(); i++) {
        std::vector<double> xyz;
        xyz.push_back(static_cast<double>(path.route[i]->getX()));
        xyz.push_back(static_cast<double>(path.route[i]->getX()));
        xyz.push_back(static_cast<double>(0));
        poses.push_back(xyz);
    }

    for (size_t i = 0; i < poses.size(); i++) {
        mission.poses[i].position.x = poses[i][0];
        mission.poses[i].position.y = poses[i][1];
        mission.poses[i].position.z = poses[i][2];
    }
}
void GraphInterface::init(const std::string& pathToGraph, const std::string& pathToEdges) {
    io::CSVReader<4> graphcsv(pathToGraph);
    io::CSVReader<3> edgescsv(pathToEdges);
    graphcsv.read_header(io::ignore_extra_column, "index", "x", "y", "z");
    edgescsv.read_header(io::ignore_extra_column, "index1", "index2", "echelon");

    std::string index; 
    float x; float y; float z;
    bool dronport = true;
    

    while (graphcsv.read_row(index, x, y, z)) {
        if (index.at(0) != 'd') dronport = false;
        allnodes.push_back(std::make_shared<graph_node>(index, x, y, 0, dronport, 2));
        dronport = true;
    }

    std::string index1; std::string index2;
    int echelon;

    while (edgescsv.read_row(index1, index2, echelon)) {
        size_t firstnode = 0; size_t secondnode = 0;
        while (allnodes[firstnode ]->getName() != index1) firstnode++;
        while (allnodes[secondnode]->getName() != index1) secondnode++;
        std::shared_ptr<edge> edge12 = std::make_shared<edge>(allnodes[firstnode], allnodes[secondnode]);
        allnodes[firstnode]->AddNeighbor(allnodes[secondnode], edge12);
    }
}

void GraphInterface::parceMissions(const std::string pathToMissions, float Vmin, float Vmax) {
    io::CSVReader<3> miscsv(pathToMissions);
    miscsv.read_header(io::ignore_extra_column, "index1", "index2", "time_appearance");

    std::string index1; std::string index2;
    float t_start;

    while (miscsv.read_row(index1, index2, t_start)) {
        size_t firstnode = 0; size_t secondnode = 0;
        while (allnodes[firstnode ]->getName() != index1) firstnode++;
        while (allnodes[secondnode]->getName() != index1) secondnode++;
        allmissions.push(std::make_shared<Mission>(allnodes[firstnode], allnodes[secondnode], t_start, Vmin, Vmax));
    }
}

void GraphInterface::publicRoutes(std::string NameOfService, float Vmin, float Vmax) {
    rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedPtr client =
        this->create_client<navigator_interfaces::srv::FreeDrone>(NameOfService);

    auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
    std::shared_ptr<Mission> curMission = allmissions.front();
    allmissions.pop();
    request->pose_dronport.position.x = curMission->start->getX();
    request->pose_dronport.position.y = curMission->start->getY();
    request->pose_dronport.position.z = 5;

    RCLCPP_INFO(this->get_logger(), "Waiting answer at %s", NameOfService.c_str());
    while (!client->wait_for_service()) {}

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call %s", NameOfService.c_str());
    }

    size_t numbernode = 0;
    while (CheckEqualPose(allnodes[numbernode]->getX(), result.get()->pose_drone.position.x, 
                            allnodes[numbernode]->getY(), result.get()->pose_drone.position.y,
                            0, 0)) {
        numbernode++;
    }

    rclcpp::Duration Tdelay = rclcpp::Duration(1, 0);
    rclcpp::Time Tstart = this->now() + Tdelay;
    Route curRoute = allnodes[numbernode]->GenRouteTo(curMission->start, allnodes, Tstart, Vmin, Vmax);
    
    rclcpp::Publisher<drone_interfaces::msg::GlobalMission>::SharedPtr mission_publisher = this->create_publisher<drone_interfaces::msg::GlobalMission>("global_mission", 1000);

    drone_interfaces::msg::GlobalMission sendMission;
    fillMsgMission( curRoute, sendMission, result.get()->id);
    mission_publisher->publish(sendMission);

    curRoute = curMission->start->GenRouteTo(curMission->finish, allnodes, curRoute.time_finish + Tdelay, Vmin, Vmax);
    fillMsgMission(curRoute, sendMission, result.get()->id);
    mission_publisher->publish(sendMission);
}