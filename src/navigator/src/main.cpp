#include <memory>
#include <string>

#include "../include/interface.hpp"
#include <rclcpp/rclcpp.hpp>

#define absoluteVmin 5
#define absoluteVmax 10

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string nodename = "navigator";

    std::string nGraph    = "/workspaces/swarm_of_drones/tests/test3/graph.csv";
    std::string nEdges    = "/workspaces/swarm_of_drones/tests/test3/edges.csv";
    std::string nMissions = "/workspaces/swarm_of_drones/tests/test3/missions.csv";
    std::string nService  = "free_drone_service";

    std::shared_ptr<GraphInterface> navigator = std::make_shared<GraphInterface>(nodename);

    navigator->init(nGraph, nEdges);
    navigator->parceMissions(nMissions, absoluteVmin, absoluteVmax);
    navigator->SetVmax(absoluteVmax);
    navigator->SetVmin(absoluteVmin);
    navigator->SetName(nService);
    rclcpp::spin(navigator);
    rclcpp::shutdown();
    return 0;
}