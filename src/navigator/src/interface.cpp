#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "../include/csv.hpp"

#include "../include/graph.hpp"
#include "drone_interfaces/msg/global_mission.hpp"
#include "navigator_interfaces/srv/free_drone.hpp"
#include "../include/interface.hpp"

bool GraphInterface::CheckEqualPose(double x1, double y1, double x2, double y2, double z1, double z2) {
    double eps = 10;
    if ( abs(x1 - x2) <= eps && abs(y2 - y1) <= eps && 
            abs(z2 - z1) <= eps) return true;
    return false;
}

void GraphInterface::fillMsgMission(Route& path, drone_interfaces::msg::GlobalMission& mission, int32_t drone_id) {
    mission.start_time = path.time_start;
    //RCLCPP_INFO(this->get_logger(), "Tstart: %f", path.time_start.seconds());
    mission.velocities.clear();

    for (size_t i = 0; i < path.velocities.size(); i++) {
        mission.velocities.push_back(static_cast<float>(path.velocities[i]));
        //RCLCPP_INFO(this->get_logger(), "v[%ld]: %f", i, path.velocities[i]);
    }
    mission.drone_id = drone_id;
    std::vector<std::vector<double>> poses;
    for (size_t i = 0; i < path.route.size(); i++) {
        std::vector<double> xyz(3);
        xyz[0] = path.route[i]->getX();
        xyz[1] = path.route[i]->getY();
        xyz[2] = 0;
        poses.push_back(xyz);
    }

    mission.poses.clear();

    for (size_t i = 1; i < poses.size(); i++) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = poses[i][0];
        //RCLCPP_INFO(this->get_logger(), "x: %f y: %f", poses[i][0], poses[i][1]);
        pose.position.y = poses[i][1];
        pose.position.z = poses[i][2];
        mission.poses.push_back(pose);
    }
}
void GraphInterface::init(const std::string& pathToGraph, const std::string& pathToEdges) {
    io::CSVReader<4> graphcsv(pathToGraph);
    io::CSVReader<3> edgescsv(pathToEdges);
    graphcsv.read_header(io::ignore_extra_column, "index", "x", "y", "z");
    edgescsv.read_header(io::ignore_extra_column, "index1", "index2", "echelon");

    std::string index; 
    double x; double y; double z;
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
        while (allnodes[secondnode]->getName() != index2) secondnode++;
        std::shared_ptr<edge> edge12 = std::make_shared<edge>(allnodes[firstnode], allnodes[secondnode]);
        RCLCPP_INFO(this->get_logger(), 
            "Append edge from: %s to: %s",
            allnodes[firstnode]->getName().c_str(),
            allnodes[secondnode]->getName().c_str());
        allnodes[firstnode]->AddNeighbor(allnodes[secondnode], edge12);
    }

    RCLCPP_INFO(this->get_logger(), "Inited graph:");
    for (size_t i = 0; i < allnodes.size(); i++) {
        RCLCPP_INFO(this->get_logger(),
            "Node: %s dronport: %d", 
            allnodes[i]->getName().c_str(), allnodes[i]->getDronport());
    }
}

void GraphInterface::parceMissions(const std::string pathToMissions, double Vmin, double Vmax) {
    io::CSVReader<3> miscsv(pathToMissions);
    miscsv.read_header(io::ignore_extra_column, "index1", "index2", "time_appearance");

    std::string index1; std::string index2;
    double t_start;

    while (miscsv.read_row(index1, index2, t_start)) {
        size_t firstnode = 0; size_t secondnode = 0;
        rclcpp::Time t_s(t_start, 0, RCL_ROS_TIME);
        while (allnodes[firstnode ]->getName() != index1) firstnode++;
        while (allnodes[secondnode]->getName() != index2) secondnode++;
        allmissions.push(std::make_shared<Mission>(allnodes[firstnode], allnodes[secondnode], t_s, Vmin, Vmax));
    }
}

void GraphInterface::publicRoutes(std::string NameOfService, double Vmin, double Vmax) {
    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<navigator_interfaces::srv::FreeDrone>("free_drone_service");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return ;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }

    auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
    std::shared_ptr<Mission> curMission = allmissions.front();
    allmissions.pop();
    request->pose_dronport.position.x = curMission->start->getX();
    request->pose_dronport.position.y = curMission->start->getY();
    request->pose_dronport.position.z = 5;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        client->remove_pending_request(result_future);
        return;
    }
    auto result = result_future.get();

    size_t numbernode = 0;
    while (!CheckEqualPose(allnodes[numbernode]->getX(), allnodes[numbernode]->getY(), 
                        result->pose_drone.position.x, result->pose_drone.position.y,
                            0, 0)) {
        numbernode++;
    }

    rclcpp::Clock clock(RCL_ROS_TIME);
    rclcpp::Duration Tdelay = rclcpp::Duration(5, 0);
    rclcpp::Time Tstart = this->now() + Tdelay;
    Route curRoute;
    curRoute.time_finish = Tstart - Tdelay;
    drone_interfaces::msg::GlobalMission sendMission;

    if (allnodes[numbernode] != curMission->start) {
        curRoute = allnodes[numbernode]->GenRouteTo(curMission->start, allnodes, Tstart, Vmin, Vmax);    
        fillMsgMission( curRoute, sendMission, result->id);
        mission_publisher->publish(sendMission);
    }

    curRoute = curMission->start->GenRouteTo(curMission->finish, allnodes, curRoute.time_finish + Tdelay, Vmin, Vmax);

    fillMsgMission(curRoute, sendMission, result->id);

    mission_publisher->publish(sendMission);
}