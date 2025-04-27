#include <cstddef>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include <utility>
#include <memory>
#include <string>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include "../include/graph.hpp"
#include "../include/time_table.hpp"

bool graph_node::CalcPossTimeEdgeAndNode(std::shared_ptr<graph_node>& node,
                       std::shared_ptr<edge>& edge, double Vmin, double Vmax) {
    TimeTable new_poss;
    TimeTable PossToDep = poss_times;
    PossToDep -= edge->getTimes();
    rclcpp::Time Tmin = TimeTable::TimeFloatToRCL(edge->Length() / Vmax);
    rclcpp::Time Tmax = TimeTable::TimeFloatToRCL(edge->Length() / Vmin);

    for (size_t i_tmp = 0; i_tmp < PossToDep.getSize(); i_tmp++) {

        rclcpp::Time t0 = PossToDep.getTime(i_tmp).first;
        rclcpp::Time t1 = t0 + (Tmin - zerotime);

        rclcpp::Time t2 = PossToDep.getTime(i_tmp).second;
        if ( !TimeTable::isInfinity(t2) ) t2 = t2 + (Tmax - zerotime);

        rclcpp::Time t3 = TimeTable::TimeFloatToRCL(-1);
        for (size_t i_edge = 0; i_edge < (edge->getTimes()).getSize(); i_edge++) {
            if ( TimeTable::isFirstBiggerOrEq((edge->getTimes()).getTime(i_edge).first, t0) &&
                 TimeTable::isFirstBiggerOrEq(t2, (edge->getTimes()).getTime(i_edge).first) &&
                 TimeTable::isFirstBiggerOrEq(t3, (edge->getTimes()).getTime(i_edge).first)) {

                t3 = (edge->getTimes()).getTime(i_edge).first;
            }
        }

        std::pair<rclcpp::Time, rclcpp::Time> timepair;
        timepair.first = t1;

        if (TimeTable::isFirstBiggerOrEq(t3, t1) &&
            TimeTable::isFirstBiggerOrEq(t3, t2)) {
            timepair.second = t2;
            if (timepair.first.get_clock_type() != 1 || timepair.second.get_clock_type() != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in CalcPossTimeEdgeAndNode");
            }
            new_poss.AppendTime(timepair);
        } else if ( TimeTable::isFirstBiggerOrEq(t3, t1) &&
                    TimeTable::isFirstBiggerOrEq(t2, t3)) {
            timepair.second = t3;
            if (timepair.first.get_clock_type() != 1 || timepair.second.get_clock_type() != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in CalcPossTimeEdgeAndNode");
            }
            new_poss.AppendTime(timepair);
        }
    }

    if (new_poss != node->poss_times) {
        node->AddPossTimes(new_poss);
        return true;
    }
    return false;
}

bool graph_node::CanGoToThisNeighbour(std::shared_ptr<graph_node>& node, size_t numberNeighbour,
    double& vel, rclcpp::Time t_finish, rclcpp::Time& t_start, double Vmin, double Vmax) {

    rclcpp::Time Tmin = TimeTable::TimeFloatToRCL(std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmin));
        
    rclcpp::Time Tmax = TimeTable::TimeFloatToRCL(std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmax));

    TimeTable edgestime = ((node->getEdges())[numberNeighbour])->getTimes();

    for (size_t i = 0; i < edgestime.getSize(); i++) {

        if ( TimeTable::isFirstTimeBigger(t_finish, edgestime.getTime(i).first) &&
             TimeTable::isFirstTimeBigger(edgestime.getTime(i).second, Tmax) ) {

            RCLCPP_INFO(rclcpp::get_logger("graph"), 
                "Cannot go to this route because edge is busy");
            return false;

        } else if ( TimeTable::isFirstTimeBigger(t_finish, edgestime.getTime(i).first) &&
                    TimeTable::isFirstBiggerOrEq(edgestime.getTime(i).second, Tmin) ) {

            Tmin = edgestime.getTime(i).second;
        }
    }
                    
    for (size_t j = 0; j < (node->getPossTimes()).getSize(); j++) {
        if (TimeTable::isFirstBiggerOrEq((node->getPossTimes()).getTime(j).second, Tmin) &&
            TimeTable::isFirstBiggerOrEq(Tmin, (node->getPossTimes()).getTime(j).first)  ){

            t_start = Tmin;
            vel = (node->getEdges())[numberNeighbour]->Length() / ((t_finish - t_start).seconds());
            return true;

        } else if (TimeTable::isFirstBiggerOrEq(Tmax, (node->getPossTimes()).getTime(j).first) &&
                   TimeTable::isFirstBiggerOrEq((node->getPossTimes()).getTime(j).first, Tmin) ){

            t_start = (node->getPossTimes()).getTime(j).first;
            vel = (node->getEdges())[numberNeighbour]->Length() / ((t_finish - t_start).seconds());
            return true;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("graph"), 
                "Cannot go to this route time of this node does not allow");
    return false;
}

std::shared_ptr<graph_node>& graph_node::GetRightNeighbour(std::shared_ptr<graph_node>& goal, 
        std::vector<std::shared_ptr<graph_node>>& allNodes, double& vel,
        rclcpp::Time t_finish, rclcpp::Time& t_start, double Vmin, double Vmax) {

    for (size_t i = 0; i < allNodes.size(); i++) {
        bool isNeighbour = false;
        size_t numberOfNeighbour;

        for (size_t j = 0; j < allNodes[i]->getNeighbors().size(); j++) {
            if ( (allNodes[i]->getNeighbors())[j] == goal) {
                isNeighbour = true;
                numberOfNeighbour = j;
            }
        }

        if (isNeighbour) {
            RCLCPP_INFO(rclcpp::get_logger("graph"), 
                "Trying to generate route from: %s to: %s", 
                allNodes[i]->getName().c_str(), goal->getName().c_str());
            if ( CanGoToThisNeighbour(allNodes[i], numberOfNeighbour, vel, t_finish, t_start, Vmin, Vmax) ) {
                RCLCPP_INFO(rclcpp::get_logger("graph"), 
                    "Go from: %s to: %s", goal->getName().c_str(), allNodes[i]->getName().c_str());
                return allNodes[i];
            }
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("graph"), "No right neighbour");
    return allNodes[0];
}

void graph_node::AddNeighbor(std::shared_ptr<graph_node>& neighbor, std::shared_ptr<edge>& edge) {
    if (edge->second() == neighbor) {
        neighbors.push_back(neighbor);
        edges.push_back(edge);
    } else RCLCPP_ERROR(rclcpp::get_logger("graph"), "Edge and node doesn't match!");
}

void graph_node::CalcAllPossTimes(double Vmin, double Vmax, std::vector<std::shared_ptr<graph_node>>& involved_nodes) {
    bool changes = false;
    bool isInInvNodes = false;

    for (size_t i = 0; i < involved_nodes.size(); i++) {
        if (involved_nodes[i] == shared_from_this()) isInInvNodes = true;
    }

    if (isInInvNodes) curRecDep++;
    else involved_nodes.push_back(shared_from_this());

    for (size_t i = 0; i < neighbors.size(); i++) {
        changes = CalcPossTimeEdgeAndNode(neighbors[i], edges[i], Vmin, Vmax);
        if ( changes && !(neighbors[i]->getDronport()) && (neighbors[i]->getCurRecDep() < depth_recursion) ) {
            neighbors[i]->CalcAllPossTimes(Vmin, Vmax, involved_nodes);
        }
    }
}

void graph_node::UpdateTimetables(Route& route) {
    rclcpp::Time curTime = route.time_start;
    rclcpp::Time prevTime = route.time_start;
    for (size_t i = 0; i < route.route.size() - 1; i++) {
        size_t numCurEdge;
                
        for (size_t j = 0; j < (route.route[i])->getEdges().size(); j++) {
            if ( ( (route.route[i])->getEdges() )[j]->second() == route.route[i+1]) {
                numCurEdge = j;
            }
        }

        rclcpp::Time t1 = curTime + (TimeTable::TimeFloatToRCL(
            ( (route.route[i])->getEdges() )[numCurEdge]->Length() / route.velocities[i]) - zerotime);
        std::pair<rclcpp::Time, rclcpp::Time> time = {curTime, t1};

        if (i != 0) {
            time = {TimeTable::TimeFloatToRCL(
                ((prevTime + (curTime - zerotime)).seconds() ) / 2),
            TimeTable::TimeFloatToRCL(
                ((curTime  + (t1      - zerotime)).seconds() ) / 2)
            };
            RCLCPP_INFO(rclcpp::get_logger("graph"), "Add time to timetable node [%f, %f]",
                                            time.first.seconds(), time.second.seconds());
            (route.route[i])->AddTime(time);
        }
        prevTime = curTime; curTime = t1;
    }
}

Route graph_node::GenRouteTo(std::shared_ptr<graph_node>& goal_node,
                 std::vector<std::shared_ptr<graph_node>>& allNodes,
                 rclcpp::Time t_start, double Vmin, double Vmax) {

    if (t_start.get_clock_type() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in GenRouteTo");
    }

    std::pair<rclcpp::Time, rclcpp::Time> start_time = {t_start, TimeTable::TimeFloatToRCL(-1)};
    poss_times.AppendTime(start_time);
    std::vector<std::shared_ptr<graph_node>> involved_nodes;

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Start generated all poss_times");
    CalcAllPossTimes(Vmin, Vmax, involved_nodes);
    RCLCPP_INFO(rclcpp::get_logger("graph"), "Generated all poss_times:");

    for (size_t i = 0; i < allNodes.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("graph"), "node %s:", allNodes[i]->getName().c_str());
        allNodes[i]->getPossTimes().PrintTimes();
    }

    std::vector<double> velocities;
    std::vector<std::shared_ptr<graph_node>> path;
    double curVel;
    std::shared_ptr<graph_node> new_goal = goal_node;
    path.insert(path.begin(), new_goal);
    Route route;
    rclcpp::Time t_finish = goal_node->getPossTimes().getTime(0).first;
    route.time_finish = t_finish;

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Starting generate route");

    while (new_goal != shared_from_this()) {
        new_goal = GetRightNeighbour(new_goal, allNodes, curVel, t_finish, t_finish, Vmin, Vmax);
        velocities.insert(velocities.begin(), curVel);
        path.insert(path.begin(), new_goal);
    }

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Ending generate route");

    route.route = path;
    route.velocities = velocities;
    route.time_start = t_finish;

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Updating timetables");

    UpdateTimetables(route);

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Clearing poss_times");

    ClearTimeValues(allNodes);

    RCLCPP_INFO(rclcpp::get_logger("graph"), "Returning routes");

    return route;
}

void graph_node::ClearTimeValues(std::vector<std::shared_ptr<graph_node>>& allNodes) {
    for (size_t i = 0; i < allNodes.size(); i++) {
        allNodes[i]->ClearPossTimes();
        allNodes[i]->refCurDepRec();
    }
}