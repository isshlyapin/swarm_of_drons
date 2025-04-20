#pragma once

#include <vector>
#include <utility>
#include <memory>
#include <string>
#include <cmath>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "time_table.hpp"

class edge;
struct Route;

class graph_node: public std::enable_shared_from_this<graph_node> {
    private:
        const std::string name;
        rclcpp::Time zerotime = rclcpp::Time(0);
        float x;
        float y;
        int echelon = 1;
        bool dronport = true;
        size_t depth_recursion = 1;
        size_t curRecDep = 0;
        std::vector<std::shared_ptr<graph_node>> neighbors;
        std::vector<std::shared_ptr<edge>> edges;
        TimeTable timetable;
        TimeTable poss_times;

        bool CalcPossTimeEdgeAndNode(std::shared_ptr<graph_node>& node, std::shared_ptr<edge>& edge, float Vmin, float Vmax);

        std::shared_ptr<graph_node>& GetRightNeighbour(std::shared_ptr<graph_node>& goal, 
                std::vector<std::shared_ptr<graph_node>>& allNodes, float& vel,
                rclcpp::Time t_finish, rclcpp::Time& t_start, float Vmin, float Vmax);
    
    public:
        graph_node(const std::string& name, float x, float y, int echelon, bool dronport, size_t rec) 
        : name(name), x(x), y(y), echelon(echelon), dronport(dronport), depth_recursion(rec) {}
        
        float getX()                                             {return x;}
        float getY()                                             {return y;}
        std::string getName()                                    {return name;}
        TimeTable& getTimes()                                    {return timetable;}
        std::vector<std::shared_ptr<graph_node>>& getNeighbors() {return neighbors;}
        std::vector<std::shared_ptr<edge>>& getEdges()           {return edges;}
        TimeTable& getPossTimes()                                {return poss_times;}
        size_t getCurRecDep()                                    {return curRecDep;}
        bool getDronport()                                       {return dronport;}

        void AddNeighbor(std::shared_ptr<graph_node>& neighbor, std::shared_ptr<edge>& edge);

        void AddTime(std::pair<rclcpp::Time, rclcpp::Time>& timing)     {timetable.AppendTime(timing);}
        void AddPossTime(std::pair<rclcpp::Time, rclcpp::Time>& timing) {poss_times.AppendTime(timing);}
        void AddPossTimes(TimeTable& times)                             {poss_times += times;}
        void ClearPossTimes()                                           {poss_times.ClearTimes();}

        void refCurDepRec() {curRecDep = 0;}

        void ClearTimeValues(std::vector<std::shared_ptr<graph_node>>& allNodes);

        void CalcAllPossTimes(float Vmin, float Vmax, std::vector<std::shared_ptr<graph_node>>& involved_nodes);

        void UpdateTimetables(Route& route);

        bool CanGoToThisNeighbour(std::shared_ptr<graph_node>& node, size_t numberNeighbour,
            float& vel, rclcpp::Time t_finish, rclcpp::Time& t_start, float Vmin, float Vmax);

        Route GenRouteTo(std::shared_ptr<graph_node>& goal_node,
                        std::vector<std::shared_ptr<graph_node>>& allNodes,
                        rclcpp::Time t_start, float Vmin, float Vmax);
};

class edge {
    private: 
        std::shared_ptr<graph_node> first_node;
        std::shared_ptr<graph_node> second_node;
        TimeTable timetable;

    public:
        edge(std::shared_ptr<graph_node>& f, std::shared_ptr<graph_node>& s)
        : first_node(f), second_node(s) {}

        std::shared_ptr<graph_node>& first()  {return first_node;}
        std::shared_ptr<graph_node>& second() {return second_node;}

        TimeTable& getTimes() {return timetable;}

        void AddTime(std::pair<rclcpp::Time, rclcpp::Time>& timing) {timetable.AppendTime(timing);}

        float Length() {
            float x1 = first_node->getX();
            float x2 = second_node->getX();
            float y1 = first_node->getY();
            float y2 = second_node->getY();
            return ( std::sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) );
        }
};

struct Route {
    std::vector<std::shared_ptr<graph_node>> route;
    std::vector<float> velocities;
    rclcpp::Time time_start;
    int echelon = 1;
};

struct Mission {
    std::shared_ptr<graph_node> start;
    std::shared_ptr<graph_node> finish;
    rclcpp::Time t_start;
    float Vmin; float Vmax;

    Mission(std::shared_ptr<graph_node>& f, std::shared_ptr<graph_node>& s, float t_s, float V1, float V2)
    : start(f), finish(s), t_start(t_s), Vmin(V1), Vmax(V2) {}
};