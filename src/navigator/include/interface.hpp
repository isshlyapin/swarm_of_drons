#include <rclcpp/create_timer.hpp>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <queue>
#include "drone_interfaces/msg/global_mission.hpp"

#include "../include/graph.hpp"

using namespace std::chrono_literals;

class GraphInterface: public rclcpp::Node {
    private:
        std::vector<std::shared_ptr<graph_node>> allnodes;
        std::queue<std::shared_ptr<Mission>> allmissions;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string NameOfService;
        float Vmin; float Vmax;
        
        static bool CheckEqualPose(double x1, double y1, double x2, double y2, double z1, double z2);
        void fillMsgMission(Route& path, drone_interfaces::msg::GlobalMission& mission, int32_t drone_id);

    public:
        GraphInterface(const std::string& nodename)
        : Node(nodename) {
            timer_ = this->create_wall_timer(500ms, 
            [this](){
                this->publicRoutes(this->NameOfService, this->Vmin, this->Vmax);
            });
        }
    
        void init(const std::string& pathToGraph, const std::string& pathToEdges);

        void parceMissions(const std::string pathToMissions, float Vmin, float Vmax);

        void publicRoutes(std::string NameOfService, float Vmin, float Vmax);

        void SetVmax(float Vmax) {this->Vmax = Vmax;}
        void SetVmin(float Vmin) {this->Vmin = Vmin;}
        void SetName(const std::string& Name) {NameOfService = Name;}
};