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
#include "navigator_interfaces/srv/free_drone.hpp"


#include "../include/graph.hpp"

using namespace std::chrono_literals;

class GraphInterface: public rclcpp::Node {
    private:
        rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedPtr client;
        rclcpp::Publisher<drone_interfaces::msg::GlobalMission>::SharedPtr mission_publisher;

        std::vector<std::shared_ptr<graph_node>> allnodes;
        std::queue<std::shared_ptr<Mission>> allmissions;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string NameOfService;
        double Vmin; double Vmax;
        
        static bool CheckEqualPose(double x1, double y1, double x2, double y2, double z1, double z2);
        void fillMsgMission(Route& path, drone_interfaces::msg::GlobalMission& mission, int32_t drone_id);

    public:
        GraphInterface(const std::string& nodename)
        : Node(nodename) {
            client = this->create_client<navigator_interfaces::srv::FreeDrone>("free_drone_service");
            mission_publisher = this->create_publisher<drone_interfaces::msg::GlobalMission>("global_mission", 1000);
            timer_ = this->create_wall_timer(500ms, 
            [this](){
                this->publicRoutes("free_drone_service", 10, 40);
            });
        }
    
        void init(const std::string& pathToGraph, const std::string& pathToEdges);

        void parceMissions(const std::string pathToMissions, double Vmin, double Vmax);

        void publicRoutes(std::string NameOfService, double Vmin, double Vmax);

        void SetVmax(double Vmax) {this->Vmax = Vmax;}
        void SetVmin(double Vmin) {this->Vmin = Vmin;}
        void SetName(const std::string& Name) {NameOfService = Name;}
};