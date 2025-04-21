#include <cstddef>
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
                       std::shared_ptr<edge>& edge, float Vmin, float Vmax) {
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
            if ( t0 <= (edge->getTimes()).getTime(i_edge).first &&
                (edge->getTimes()).getTime(i_edge).first <= t2 &&
                (edge->getTimes()).getTime(i_edge).first <= t3) {

                t3 = (edge->getTimes()).getTime(i_edge).first;
            }
        }

        std::pair<rclcpp::Time, rclcpp::Time> timepair;
        timepair.first = t1;

        if (t1 <= t3 && t2 <= t3) {
            timepair.second = t2;
            new_poss.AppendTime(timepair);
        } else if (t1 <= t3 && t2 >= t3) {
            timepair.second = t3;
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
    float& vel, rclcpp::Time t_finish, rclcpp::Time& t_start, float Vmin, float Vmax) {

    rclcpp::Time Tmin = TimeTable::TimeFloatToRCL((float)std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmin));
    rclcpp::Time Tmax = TimeTable::TimeFloatToRCL((float)std::max((double)0,
        t_finish.seconds() - (node->getEdges())[numberNeighbour]->Length() / Vmax));

    TimeTable edgestime = ((node->getEdges())[numberNeighbour])->getTimes();
    for (size_t i = 0; i < edgestime.getSize(); i++) {
        if ( edgestime.getTime(i).first < t_finish && edgestime.getTime(i).second > Tmax) {
            return false;
        } else if (edgestime.getTime(i).first < t_finish && edgestime.getTime(i).second >= Tmin) {
            Tmin = edgestime.getTime(i).second;
        }
    }
                    
    for (size_t j = 0; j < (node->getPossTimes()).getSize(); j++) {
        if (Tmin <= (node->getPossTimes()).getTime(j).second &&
            Tmin >= (node->getPossTimes()).getTime(j).first  ){

            t_start = Tmin;
            vel = (node->getEdges())[numberNeighbour]->Length() / ((t_finish - t_start).seconds());
            return true;

        } else if ((node->getPossTimes()).getTime(j).first <= Tmax &&
                   (node->getPossTimes()).getTime(j).first >= Tmin ){

            t_start = (node->getPossTimes()).getTime(j).first;
            vel = (node->getEdges())[numberNeighbour]->Length() / ((t_finish - t_start).seconds());
            return true;
        }
    }
    return false;
}

std::shared_ptr<graph_node>& graph_node::GetRightNeighbour(std::shared_ptr<graph_node>& goal, 
        std::vector<std::shared_ptr<graph_node>>& allNodes, float& vel,
        rclcpp::Time t_finish, rclcpp::Time& t_start, float Vmin, float Vmax) {

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
            if ( CanGoToThisNeighbour(allNodes[i], numberOfNeighbour, vel, t_finish, t_start, Vmin, Vmax) ) {
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
    } else printf("Edge and node doesn't match!\n");
}

void graph_node::CalcAllPossTimes(float Vmin, float Vmax, std::vector<std::shared_ptr<graph_node>>& involved_nodes) {
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
        ( (route.route[i])->getEdges() )[numCurEdge]->AddTime(time);
        if (i != 0) {
            time = {TimeTable::TimeFloatToRCL(
                static_cast<float>( ((prevTime + (curTime - zerotime)).seconds() ) / 2) ),
            TimeTable::TimeFloatToRCL(
                static_cast<float>( ((curTime  + (t1      - zerotime)).seconds() ) / 2) )
            };
            (route.route[i])->AddTime(time);
        }
        prevTime = curTime; curTime = t1;
    }
}

Route graph_node::GenRouteTo(std::shared_ptr<graph_node>& goal_node,
                 std::vector<std::shared_ptr<graph_node>>& allNodes,
                 rclcpp::Time t_start, float Vmin, float Vmax) {

    std::pair<rclcpp::Time, rclcpp::Time> start_time = {t_start, TimeTable::TimeFloatToRCL(-1)};
    poss_times.AppendTime(start_time);
    std::vector<std::shared_ptr<graph_node>> involved_nodes;
    CalcAllPossTimes(Vmin, Vmax, involved_nodes);

    std::vector<float> velocities;
    std::vector<std::shared_ptr<graph_node>> path;
    float curVel;
    std::shared_ptr<graph_node> new_goal = goal_node;
    path.insert(path.begin(), new_goal);
    Route route;
    rclcpp::Time t_finish = goal_node->getPossTimes().getTime(0).first;
    route.time_finish = t_finish;

    while (new_goal != shared_from_this()) {
        new_goal = GetRightNeighbour(new_goal, allNodes, curVel, t_finish, t_finish, Vmin, Vmax);
        velocities.insert(velocities.begin(), curVel);
        path.insert(path.begin(), new_goal);
    }

    route.route = path;
    route.velocities = velocities;
    route.time_start = t_finish;

    UpdateTimetables(route);
    ClearTimeValues(allNodes);

    return route;
}

void graph_node::ClearTimeValues(std::vector<std::shared_ptr<graph_node>>& allNodes) {
    for (size_t i = 0; i < allNodes.size(); i++) {
        allNodes[i]->ClearPossTimes();
        allNodes[i]->refCurDepRec();
    }
}