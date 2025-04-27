#include <memory>
#include <string>

#include "../include/graph.hpp"
#include "../include/time_table.hpp"
#include "../include/interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <utility>

#define absoluteVmin 5
#define absoluteVmax 10

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string nodename = "navigator";

    std::string nGraph    = "/workspaces/tests/test3/swarm_of_drons/graph.csv";
    std::string nEdges    = "/workspaces/tests/test3/swarm_of_drons/edges.csv";
    std::string nMissions = "/workspaces/tests/test/swarm_of_drons/missions.csv";
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