#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
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

    // request->pose_dronport.position.x = 41;
    // request->pose_dronport.position.y = 12;
    // request->pose_dronport.position.z = 5;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
      client->remove_pending_request(result_future);
      return;
    }
    auto result = result_future.get();
    // auto result = result_future;
    RCLCPP_INFO(
      node->get_logger(), "Result: %s_%d",
      result->model.c_str(), result->id);

    // auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
    // std::shared_ptr<Mission> curMission = allmissions.front();
    // allmissions.pop();
    // request->pose_dronport.position.x = curMission->start->getX();
    // request->pose_dronport.position.y = curMission->start->getY();
    // request->pose_dronport.position.z = 5;

    // RCLCPP_INFO(this->get_logger(), "Waiting answer at %s", NameOfService.c_str());
    // while (!client->wait_for_service()) {}

    // printf("before spin\n");
    // auto result = client->async_send_request(
    //     request,
    //     [this](rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedFuture result) {
    //         RCLCPP_INFO(this->get_logger(), "Request sent");
    //         RCLCPP_INFO(this->get_logger(), "Result drone %s_%d", result.get()->model.c_str(), result.get()->id);
    //     }
    // );
    
    // printf("after spin\n");
    // RCLCPP_INFO(this->get_logger(), "Result drone %s_%d", result.get()->model.c_str(), result.get()->id);
    size_t numbernode = 0;
    while (CheckEqualPose(allnodes[numbernode]->getX(), result->pose_drone.position.x, 
                            allnodes[numbernode]->getY(), result->pose_drone.position.y,
                            0, 0)) {
        numbernode++;
    }

    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    rclcpp::Duration Tdelay = rclcpp::Duration(100, 0);
    rclcpp::Time Tstart = clock.now() + Tdelay;
    // rclcpp::Time Tfinish(0, 0, RCL_ROS_TIME);
    // printf("times\n");
    // if (Tfinish > Tstart) printf("Tfinish > Tstart\n");
    // printf("times\n");
    Route curRoute = allnodes[numbernode]->GenRouteTo(curMission->start, allnodes, Tstart, Vmin, Vmax);
    
    drone_interfaces::msg::GlobalMission sendMission;
    fillMsgMission( curRoute, sendMission, result->id);
    mission_publisher->publish(sendMission);

    curRoute = curMission->start->GenRouteTo(curMission->finish, allnodes, curRoute.time_finish + Tdelay, Vmin, Vmax);
    fillMsgMission(curRoute, sendMission, result->id);
    mission_publisher->publish(sendMission);
}