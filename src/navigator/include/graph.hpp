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
        rclcpp::Time zerotime = rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);
        double x;
        double y;
        int echelon = 1;
        bool dronport = true;
        size_t depth_recursion = 1;
        size_t curRecDep = 0;
        std::vector<std::shared_ptr<graph_node>> neighbors;
        std::vector<std::shared_ptr<edge>> edges;
        TimeTable timetable;
        TimeTable poss_times;

        bool CalcPossTimeEdgeAndNode(std::shared_ptr<graph_node>& node, std::shared_ptr<edge>& edge, double Vmin, double Vmax);

        std::shared_ptr<graph_node>& GetRightNeighbour(std::shared_ptr<graph_node>& goal, 
                std::vector<std::shared_ptr<graph_node>>& allNodes, double& vel,
                rclcpp::Time t_finish, rclcpp::Time& t_start, double Vmin, double Vmax);
    
    public:
        graph_node(const std::string& name, double x, double y, int echelon, bool dronport, size_t rec) 
        : name(name), x(x), y(y), echelon(echelon), dronport(dronport), depth_recursion(rec) {}
        
        double getX()                                            {return x;}
        double getY()                                            {return y;}
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

        void CalcAllPossTimes(double Vmin, double Vmax, std::vector<std::shared_ptr<graph_node>>& involved_nodes);

        void UpdateTimetables(Route& route);

        bool CanGoToThisNeighbour(std::shared_ptr<graph_node>& node, size_t numberNeighbour,
            double& vel, rclcpp::Time t_finish, rclcpp::Time& t_start, double Vmin, double Vmax);

        Route GenRouteTo(std::shared_ptr<graph_node>& goal_node,
                        std::vector<std::shared_ptr<graph_node>>& allNodes,
                        rclcpp::Time t_start, double Vmin, double Vmax);
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

        double Length() {
            double x1 = first_node->getX();
            double x2 = second_node->getX();
            double y1 = first_node->getY();
            double y2 = second_node->getY();
            return ( std::sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) );
        }
};

struct Route {
    std::vector<std::shared_ptr<graph_node>> route;
    std::vector<double> velocities;
    rclcpp::Time time_start;
    rclcpp::Time time_finish;
    int echelon = 1;
};

struct Mission {
    std::shared_ptr<graph_node> start;
    std::shared_ptr<graph_node> finish;
    rclcpp::Time t_start;
    double Vmin; double Vmax;

    Mission(std::shared_ptr<graph_node>& f, std::shared_ptr<graph_node>& s, rclcpp::Time t_s, double V1, double V2)
    : start(f), finish(s), t_start(t_s), Vmin(V1), Vmax(V2) {}
};