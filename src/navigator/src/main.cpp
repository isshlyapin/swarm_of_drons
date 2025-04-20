#include "../include/graph.hpp"
#include "../include/time_table.hpp"
#include <rclcpp/rclcpp.hpp>

int main() {

    std::shared_ptr<graph_node> dp1 = std::make_shared<graph_node>(
        "dp1", 0, 0, 1, true,  1);
    std::shared_ptr<graph_node> dp2 = std::make_shared<graph_node>(
        "dp2", 2, 0, 1, true,  1);
    std::shared_ptr<graph_node> kt1 = std::make_shared<graph_node>(
        "kt1", 1, 0, 1, false, 1);

    std::vector<std::shared_ptr<graph_node>> all_nodes;
    
    all_nodes.push_back(dp1);
    all_nodes.push_back(dp2);
    all_nodes.push_back(kt1);

    std::shared_ptr<edge> dp1_kt1 = std::make_shared<edge>(dp1, kt1);
    std::shared_ptr<edge> kt1_dp2 = std::make_shared<edge>(kt1, dp2);

    dp1->AddNeighbor(kt1, dp1_kt1);
    kt1->AddNeighbor(dp2, kt1_dp2);

    Route path1 = dp1->GenRouteTo(dp2, all_nodes, TimeTable::TimeFloatToRCL(0), 1, 2);
    //printf("EDGE PD1->KT1:\n");
    //dp1->getEdges()[0]->getTimes().PrintTimes();
    //printf("EDGE KT1->DP2:\n");
    //kt1->getEdges()[0]->getTimes().PrintTimes();
    Route path2 = dp1->GenRouteTo(dp2, all_nodes, TimeTable::TimeFloatToRCL(0), 1, 2);

    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "path1: %s->%s->%s\n", path1.route[0]->getName().c_str(),
        path1.route[1]->getName().c_str(), path1.route[2]->getName().c_str());
    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "vel1: 1->2: %f; 2->3: %f\n", path1.velocities[0],
    path1.velocities[1]);
    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "t1: %f\n", path1.time_start.seconds());

    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "path2: %s->%s->%s\n", path2.route[0]->getName().c_str(),
        path2.route[1]->getName().c_str(), path2.route[2]->getName().c_str());
    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "vel2: 1->2: %f; 2->3: %f\n", path2.velocities[0],
    path2.velocities[1]);
    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "t2: %f\n", path2.time_start.seconds());

    //printf("EDGE PD1->KT1:\n");
    //dp1->getEdges()[0]->getTimes().PrintTimes();
    //printf("EDGE KT1->DP2:\n");
    //kt1->getEdges()[0]->getTimes().PrintTimes();
    
    return 0;
}