#include <cassert>
#include <cstddef>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
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
            RCLCPP_INFO(rclcpp::get_logger("graph"), "timepair");
            RCLCPP_INFO(rclcpp::get_logger("graph"), "[%f, %f]",
                timepair.first.seconds(), timepair.second.seconds());
            new_poss.AppendTime(timepair);
        } else if ( TimeTable::isFirstBiggerOrEq(t3, t1) &&
                    TimeTable::isFirstBiggerOrEq(t2, t3)) {
            timepair.second = t3;
            if (timepair.first.get_clock_type() != 1 || timepair.second.get_clock_type() != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("graph"), "Not sim time in CalcPossTimeEdgeAndNode");
            }
            RCLCPP_INFO(rclcpp::get_logger("graph"), "timepair");
            RCLCPP_INFO(rclcpp::get_logger("graph"), "[%f, %f]",
                timepair.first.seconds(), timepair.second.seconds());
            new_poss.AppendTime(timepair);
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("graph"), "new_poss");
    new_poss.PrintTimes();
    RCLCPP_INFO(rclcpp::get_logger("graph"), "old_poss");
    node->getTimes().PrintTimes();
    new_poss -= node->getTimes();
    RCLCPP_INFO(rclcpp::get_logger("graph"), "new new_poss");
    new_poss.PrintTimes();
    TimeTable old_poss_times = node->getPossTimes();

    node->AddPossTimes(new_poss);

    if (old_poss_times != node->getPossTimes()) {
        return true;
    }
    return false;
}

bool graph_node::canGoToThisEdge(TimeTable& edgestime, rclcpp::Time& Tmin, rclcpp::Time Tmax, rclcpp::Time t_finish) {
    bool changes = true;
    for (size_t i = 0; i < edgestime.getSize(); i++) {

        if ( TimeTable::isFirstTimeBigger(t_finish, edgestime.getTime(i).first) &&
             TimeTable::isFirstTimeBigger(edgestime.getTime(i).first, Tmax) ) {

            changes = false;
            
        } else if ( TimeTable::isFirstBiggerOrEq(t_finish, edgestime.getTime(i).second) &&
                    TimeTable::isFirstTimeBigger(edgestime.getTime(i).second, Tmax) ) {

            changes = false; 

        } else if ( TimeTable::isFirstBiggerOrEq(Tmax, edgestime.getTime(i).first) &&
                    TimeTable::isFirstBiggerOrEq(edgestime.getTime(i).second, t_finish)) {

            changes = false; 

        } else if ( TimeTable::isFirstTimeBigger(t_finish, edgestime.getTime(i).first) &&
                    TimeTable::isFirstBiggerOrEq(edgestime.getTime(i).second, Tmin) ) {

            Tmin = edgestime.getTime(i).second;
        }
    }
    return changes;
}

bool graph_node::CanGoToThisNeighbour(std::shared_ptr<graph_node>& node, size_t numberNeighbour,
    double& vel, rclcpp::Time t_finish, rclcpp::Time& t_start, double Vmin, double Vmax) {

    rclcpp::Time Tmin = TimeTable::TimeFloatToRCL(std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmin));
        
    rclcpp::Time Tmax = TimeTable::TimeFloatToRCL(std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmax));

    TimeTable edgestime = ((node->getEdges())[numberNeighbour])->getTimes();

    if (!canGoToThisEdge(edgestime, Tmin, Tmax, t_finish)) return false;
                    
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

        if (isNeighbour && ( ( !allNodes[i]->getDronport() ) || 
                                allNodes[i] == shared_from_this())) {
            if ( CanGoToThisNeighbour(allNodes[i], numberOfNeighbour, vel, t_finish, t_start, Vmin, Vmax) ) {
                return allNodes[i];
            }
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("graph"), "No right neighbour");
    std::exit(EXIT_FAILURE); 
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
        std::pair<rclcpp::Time, rclcpp::Time> time = {curTime, t1 + (Tdelay - zerotime)};
        ( (route.route[i])->getEdges() )[numCurEdge]->AddTime(time);

        if (i != 0) {
            time = {TimeTable::TimeFloatToRCL(
                ((prevTime + (curTime - zerotime)).seconds() ) / 2),
            TimeTable::TimeFloatToRCL(
                ((curTime  + (t1      - zerotime)).seconds() ) / 2) + (Tdelay - zerotime)
            };
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

    ClearTimeValues(allNodes, t_start);

    std::pair<rclcpp::Time, rclcpp::Time> start_time = {t_start, TimeTable::TimeFloatToRCL(-1)};
    poss_times.AppendTime(start_time);
    std::vector<std::shared_ptr<graph_node>> involved_nodes;

    CalcAllPossTimes(Vmin, Vmax, involved_nodes);

    std::vector<double> velocities;
    std::vector<std::shared_ptr<graph_node>> path;
    double curVel;
    std::shared_ptr<graph_node> new_goal = goal_node;
    path.insert(path.begin(), new_goal);
    Route route;
    //RCLCPP_INFO(rclcpp::get_logger("graph"), "t_finish");
    rclcpp::Time t_finish = goal_node->getPossTimes().getTime(0).first;
    rclcpp::Time old_t_finish = goal_node->getPossTimes().getTime(0).first;
    //RCLCPP_INFO(rclcpp::get_logger("graph"), "after t_finish");
    std::shared_ptr<graph_node> old_goal;
    route.time_finish = t_finish;

    std::vector<rclcpp::Time> times;
    times.push_back(t_finish);

    while (new_goal != shared_from_this()) {
        old_t_finish = t_finish; old_goal = new_goal;

        new_goal = GetRightNeighbour(new_goal, allNodes, curVel, t_finish, t_finish, Vmin, Vmax);
        velocities.insert(velocities.begin(), curVel);
        path.insert(path.begin(), new_goal);
    }

    route.route = path;
    route.velocities = velocities;
    route.time_start = t_finish;

    UpdateTimetables(route);
    ClearTimeValues(allNodes, t_start);
    return route;
}

void graph_node::ClearTimeValues(std::vector<std::shared_ptr<graph_node>>& allNodes, rclcpp::Time t_start) {
    std::pair<rclcpp::Time, rclcpp::Time> timepair = {TimeTable::TimeFloatToRCL(0), t_start};

    for (size_t i = 0; i < allNodes.size(); i++) {
        allNodes[i]->ClearPossTimes();
        allNodes[i]->refCurDepRec();
    }
}