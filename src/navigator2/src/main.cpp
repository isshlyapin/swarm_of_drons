#include <future>
#include <list>
#include <iostream>

#include <queue>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>

#include "drone_interfaces/msg/global_mission.hpp"
#include "navigator_interfaces/srv/free_drone.hpp"

#include "navigator2/map.hpp"

#include <fastcsv/csv.hpp>

using namespace std::chrono_literals;

class MinimalPathGenerator : public rclcpp::Node {
public:
    MinimalPathGenerator()
        : Node("minimal_path_generator")
    {
        initMap();

        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_ = this->create_client<navigator_interfaces::srv::FreeDrone>(
            "free_drone_service", 10, client_callback_group_);

        Globalpublisher_ = this->create_publisher<drone_interfaces::msg::GlobalMission>(
            "global_mission", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPathGenerator::timeCallback, this),
            timer_callback_group_);
    }

    void addMission(const std::string& id_from, const std::string& id_to) {
        mission_queue_.push({id_from, id_to});
    } // addMission

    void timeCallback() {
        if (mission_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No missions in queue");
            return;
        }

        static int count = 0;
        RCLCPP_INFO(this->get_logger(), "Timer callback %d", count++);

        auto mission = mission_queue_.front();
        mission_queue_.pop();

        auto node = rclcpp::Node::make_shared("minimal_client");
        auto client = node->create_client<navigator_interfaces::srv::FreeDrone>("free_drone_service");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return ;
            }
            RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
        }

        auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
        auto start_position = map.getVertexPosition(mission.first);

        request->pose_dronport.position.x = start_position.x();
        request->pose_dronport.position.y = start_position.y();
        request->pose_dronport.position.z = start_position.z();

        auto result_future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
        {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        client->remove_pending_request(result_future);
        return;
        }
        auto result = result_future.get();

        auto response = result;

        Vertex::Point drone_pose{
            response->pose_drone.position.x,
            response->pose_drone.position.y,
            response->pose_drone.position.z
        };

        auto drone_vertex_id = map.getVertexId(drone_pose);
        if(drone_vertex_id.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Drone vertex not found");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Drone %s_%d", response->model.c_str(), response->id);

        RCLCPP_INFO(this->get_logger(), "Drone vertex id: %s", drone_vertex_id.c_str());
        Drone drone{
            drone_vertex_id,
            DroneVMax{10.0}, DroneVMin{8.0},
            DroneFreeTime{this->now().seconds() + 15}
        };
        
        if (drone_vertex_id != mission.first) {
            auto mission1 = map.generateMission(drone_vertex_id, mission.first, drone, OptimalPathMode::MIN_DISTANCE);

            drone_interfaces::msg::GlobalMission msg1;
            msg1.drone_id = response->id;
            msg1.start_time.sec = static_cast<int>(mission1.timeStart);
            msg1.start_time.nanosec = static_cast<int>(
                (mission1.timeStart - msg1.start_time.sec) * 1e9
            );
            for (size_t i = 0; i < mission1.vertexes.size() - 1; ++i) {
                Vertex::Point point = map.getVertexPosition(mission1.vertexes[i+1]);
                geometry_msgs::msg::Pose pose;
                pose.position.x = point.x();
                pose.position.y = point.y();
                pose.position.z = point.z();
                msg1.poses.push_back(pose);
                msg1.velocities.push_back(mission1.velocities[i]);
            }
            drone.setFreeTime(mission1.timeFinish + 5);
            Globalpublisher_->publish(msg1);
        }

        auto mission2 = map.generateMission(mission.first, mission.second, drone, OptimalPathMode::MIN_DISTANCE);

        RCLCPP_INFO(this->get_logger(), "MissionMsg2 field start");
        drone_interfaces::msg::GlobalMission msg2;
        msg2.start_time.sec = static_cast<int>(mission2.timeStart);
        msg2.start_time.nanosec = static_cast<int>(
            (mission2.timeStart - msg2.start_time.sec) * 1e9
        );
        RCLCPP_INFO(this->get_logger(), "MissionMsg2 field poses and velocities");
        for (size_t i = 1; i < mission2.vertexes.size(); ++i) {
            Vertex::Point point = map.getVertexPosition(mission2.vertexes[i]);
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            pose.position.z = point.z();
            msg2.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "MissionMsg2 field velocities");

        for (const auto velocity : mission2.velocities) {
            msg2.velocities.push_back(velocity);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing mission from %s to %s", mission.first.c_str(), mission.second.c_str());
        Globalpublisher_->publish(msg2);
    }

    void initMap() {
        map.addVertex("d1", {80,158,0});
        map.addVertex("d2", {251,414,0});
        map.addVertex("d3", {468,40,0});
        map.addVertex("d4", {677,307,0});

        map.addVertex("p1", {508,154,0});
        map.addVertex("p2", {223,110,0});
        map.addVertex("p3", {267,221,0});
        map.addVertex("p4", {80,303,0});
        map.addVertex("p5", {241,317,0});
        map.addVertex("p6", {424,353,0});
        
        map.addEdge("d1", "p3");
        map.addEdge("d1", "p4");
        map.addEdge("p2", "d1");
        map.addEdge("d3", "p2");
        map.addEdge("d3", "p1");
        map.addEdge("p3", "p2");
        map.addEdge("p3", "d3");
        map.addEdge("p3", "p4");
        map.addEdge("p3", "p6");
        map.addEdge("p1", "p3");
        map.addEdge("d4", "p1");
        map.addEdge("d4", "p3");
        map.addEdge("p4", "d2");
        map.addEdge("p5", "p3");
        map.addEdge("d2", "p5");
        map.addEdge("d2", "p6");
        map.addEdge("p6", "d4");
        map.addEdge("p6", "p5");    
    }

private:
    Map map;
    std::queue<std::pair<std::string, std::string>> mission_queue_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedPtr client_;
    rclcpp::Publisher<drone_interfaces::msg::GlobalMission>::SharedPtr Globalpublisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPathGenerator>();

    io::CSVReader<3> in("/workspaces/swarm_of_drons/missions.csv");
    in.read_header(io::ignore_extra_column, "index1", "index2", "time_appearance");

    std::string id_from, id_to;
    double time_appearance;
    while (in.read_row(id_from, id_to, time_appearance)) {
        node->addMission(id_from, id_to);
        RCLCPP_INFO(node->get_logger(), "Adding mission from %s to %s", id_from.c_str(), id_to.c_str());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
