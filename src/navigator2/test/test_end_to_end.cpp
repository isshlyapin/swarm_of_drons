#include <gtest/gtest.h>
#include <rclcpp/context.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <memory>
#include <future>
#include <fstream>
#include <stop_token>
#include <utility>

#include "navigator2/navigator.hpp"
#include "navigator2/mission_manager.hpp"
#include "navigator2/map.hpp"
#include "navigator2/drone.hpp"
#include "drone_interfaces/msg/report.hpp"
#include "drone_interfaces/msg/mission.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

#include "drone_composition/drone.hpp"
#include "drone_interfaces/msg/report.hpp"
#include "drone_interfaces/msg/mission.hpp"
#include "drone_composition/drone_controller.hpp"
#include "drone_interfaces/msg/global_mission.hpp"
#include "navigator_interfaces/srv/free_drone.hpp"

using namespace std::chrono_literals;

class EndToEndTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>(
            "test_node" 
        );
    }
    void TearDown() override {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
};

void f(const std::stop_token& stoken, const std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>& executor) {
    while (!stoken.stop_requested()) {
        executor->spin_some();
        std::this_thread::sleep_for(100ms);
    }
}

class NodeController {
public:
    NodeController(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(node_);
    }

    void spin() {
        thread_ = std::jthread(f, std::ref(executor_));
    }

    void stop() {
        thread_.request_stop();
    }
private:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::shared_ptr<rclcpp::Node> node_;
    std::jthread thread_;    
};

TEST_F(EndToEndTest, TestNavigatorOnlyExecute) {    
    auto drone = std::make_shared<DroneComposition::Drone>(
        rclcpp::NodeOptions().arguments({
            "--ros-args", "-r", "__node:=x500_1",
            "-p", "pose_x:=0.0",
            "-p", "pose_y:=0.0",
            "-p", "pose_z:=0.0"
        })
    );

    NodeController drone_ctrl(drone);
    drone_ctrl.spin();

    auto controller = std::make_shared<DroneComposition::DroneController>(
        rclcpp::NodeOptions().arguments({
            "--ros-args", "-r", "__node:=drone_controller", 
            "-p", "drones_file:=/workspaces/swarm_of_drons/tests/test6/drones.csv"
        })
    );
    NodeController controller_ctrl(controller);
    controller_ctrl.spin();

    std::shared_ptr<rclcpp::Node> client_node = rclcpp::Node::make_shared("minimal_client");
    rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedPtr client =
        node->create_client<navigator_interfaces::srv::FreeDrone>("/free_drone_service");

    auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
    request->pose_dronport.position.x = 0.0;
    request->pose_dronport.position.y = 0.0;
    request->pose_dronport.position.z = 0.0;

    bool srv_free_drone_ok = false;    
    for (int i = 0; i < 30; ++i) {
        if (!client->wait_for_service(1s)) {
            EXPECT_TRUE(rclcpp::ok());
            continue; 
        }
    
        auto future = client->async_send_request(request);
        auto result = rclcpp::spin_until_future_complete(client_node, future);

        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            EXPECT_TRUE(rclcpp::ok()); 
        } else {
            auto response = future.get();
            EXPECT_EQ(response->id, 1);
            EXPECT_EQ(response->model, "x500");
            EXPECT_DOUBLE_EQ(response->pose_drone.position.x, 0.0);
            EXPECT_DOUBLE_EQ(response->pose_drone.position.y, 0.0);
            EXPECT_DOUBLE_EQ(response->pose_drone.position.z, 0.0);
    
            srv_free_drone_ok = true;
            break;
        }        
        rclcpp::sleep_for(100ms);
    }
    ASSERT_TRUE(srv_free_drone_ok) << "Не дождались ответа от сервиса free_drone_service";
   
    auto navigator = std::make_shared<Navigator>(
        rclcpp::NodeOptions().arguments({
            "--ros-args", "-r", "__node:=navigator",
            "-p", "graph_file:=/workspaces/swarm_of_drons/tests/test6/graph.csv",
            "-p", "edges_file:=/workspaces/swarm_of_drons/tests/test6/edges.csv",
            "-p", "missions_file:=/workspaces/swarm_of_drons/tests/test6/missions.csv"
        })
    );
    NodeController navigator_ctrl(navigator);
    navigator_ctrl.spin();

    // executor.add_node(navigator);

    // std::promise<drone_interfaces::msg::Report> fly_report_promise;
    // std::promise<drone_interfaces::msg::Report> ready_report_promise;
    // auto fly_report_future = fly_report_promise.get_future();
    // auto ready_report_future = ready_report_promise.get_future();

    // // Флаги для защиты от повторного set_value
    // std::atomic<bool> fly_set{false};
    // std::atomic<bool> ready_set{false};

    // auto report_sub = node->create_subscription<drone_interfaces::msg::Report>(
    //     "/x500_1/report", 
    //     10,
    //     [&fly_report_promise, &ready_report_promise, &fly_set, &ready_set](drone_interfaces::msg::Report::SharedPtr msg) {
    //         if (msg->state == 1 && !fly_set.exchange(true)) { // FLY
    //             try {
    //                 fly_report_promise.set_value(*msg);
    //             } catch (const std::future_error&) {}
    //         }
    //         if (msg->state == 0 && !ready_set.exchange(true)) { // READY
    //             try {
    //                 ready_report_promise.set_value(*msg);
    //             } catch (const std::future_error&) {}
    //         }
    //     }
    // );

    // // Ждём report с состоянием FLY
    // bool got_fly = false;
    // for (int i = 0; i < 30; ++i) {
    //     executor.spin_some();
    //     if (fly_report_future.wait_for(0ms) == std::future_status::ready) {
    //         got_fly = true;
    //         break;
    //     }
    //     std::this_thread::sleep_for(100ms);
    // }
    // ASSERT_TRUE(got_fly) << "Не дождались report с состоянием FLY";

    // // Ждём report с состоянием READY (миссия завершена)
    // bool got_ready = false;
    // for (int i = 0; i < 50; ++i) {
    //     executor.spin_some();
    //     if (ready_report_future.wait_for(0ms) == std::future_status::ready) {
    //         got_ready = true;
    //         break;
    //     }
    //     std::this_thread::sleep_for(100ms);
    // }
    // ASSERT_TRUE(got_ready) << "Не дождались report с состоянием READY";

    // auto ready_report = ready_report_future.get();
    // EXPECT_NEAR(ready_report.pose.position.x, 2.0, 0.05);
    // EXPECT_NEAR(ready_report.pose.position.y, 2.0, 0.05);
    // EXPECT_NEAR(ready_report.pose.position.z, 2.0, 0.05);

    // srv_free_drone_ok = false;    
    // for (int i = 0; i < 30; ++i) {
    //     executor.spin_some();
    //     if (!client->wait_for_service(1s)) {
    //         EXPECT_TRUE(rclcpp::ok());
    //         continue; 
    //     }
    
    //     auto result = client->async_send_request(request);
        
    //     for (int j = 0; j < 30; ++j) {
    //         executor.spin_some();
    //         if (result.wait_for(1ms) == std::future_status::ready) {
    //             auto response = result.get();
    //             EXPECT_EQ(response->id, 1);
    //             EXPECT_EQ(response->model, "x500");
    //             EXPECT_DOUBLE_EQ(response->pose_drone.position.x, 2.0);
    //             EXPECT_DOUBLE_EQ(response->pose_drone.position.y, 2.0);
    //             EXPECT_DOUBLE_EQ(response->pose_drone.position.z, 2.0);

    //             srv_free_drone_ok = true;
    //             break;
    //         }   
    //     }
    //     rclcpp::sleep_for(100ms);
    // }
    // ASSERT_TRUE(srv_free_drone_ok) << "Не дождались ответа от сервиса free_drone_service";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
