#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <string>
#include <future>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
// #include <rclcpp_components/component_manager.hpp>
// #include <rclcpp/executors/single_threaded_executor.hpp>

#include "drone_composition/drone.hpp"
#include "drone_interfaces/msg/report.hpp"
#include "drone_interfaces/msg/mission.hpp"
#include "drone_composition/drone_controller.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

using namespace std::chrono_literals;

class DroneCompositionTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("test_node");
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
    rclcpp::Node::SharedPtr node;
};

TEST_F(DroneCompositionTest, DroneTopicsAppear) {
    auto drone = std::make_shared<DroneComposition::Drone>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-r", "__node:=x500_1"}
        )
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(drone);

    // Ждем появления топиков с периодической проверкой
    bool found_odom = false;
    bool found_report = false;
    bool found_mission = false;
    bool found_panel_logs = false;
    for (int i = 0; i < 20; ++i) {
        exec.spin_some();
        auto topics = node->get_topic_names_and_types();
        found_mission = topics.count("/x500_1/mission") == 1;
        found_report  = topics.count("/x500_1/report") == 1;
        found_odom    = topics.count("/x500_1/odometry") == 1;
        found_panel_logs = topics.count("/panel_logs") == 1;

        if (found_mission && found_report && found_odom && found_panel_logs) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(found_mission);
    EXPECT_TRUE(found_report);
    EXPECT_TRUE(found_odom);
    EXPECT_TRUE(found_panel_logs);
}

TEST_F(DroneCompositionTest, DroneControllerTopicsAppear) {
    auto controller = std::make_shared<DroneComposition::DroneController>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-p", "drones_file:=/workspaces/swarm_of_drons/tests/test5/drones.csv"}
        )
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(controller);

    // Ждем появления топиков с периодической проверкой
    bool found_odom = false;
    bool found_report = false;
    bool found_mission = false;
    bool found_global_mission = false;

    for (int i = 0; i < 20; ++i) {
        exec.spin_some();
        auto topics = node->get_topic_names_and_types();
        found_odom    = topics.count("/x500_1/odometry") == 1;
        found_report  = topics.count("/x500_1/report") == 1;
        found_mission = topics.count("/x500_1/mission") == 1;
        found_global_mission = topics.count("/global_mission") == 1;

        if (found_mission && found_report && found_odom && found_global_mission) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(found_odom);
    EXPECT_TRUE(found_report);
    EXPECT_TRUE(found_mission);
    EXPECT_TRUE(found_global_mission);
}

TEST_F(DroneCompositionTest, DroneControllerServicesAppear) {
    auto controller = std::make_shared<DroneComposition::DroneController>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-p", "drones_file:=/workspaces/swarm_of_drons/tests/test5/drones.csv"}
        )
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(controller);

    // Ждем появления сервисов с периодической проверкой
    bool found_free_drone = false;
    for (int i = 0; i < 20; ++i) {
        exec.spin_some();
        auto services = node->get_service_names_and_types();
        found_free_drone = services.count("/free_drone_service") == 1;

        if (found_free_drone) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(found_free_drone);
}

TEST_F(DroneCompositionTest, DroneMissionExecution) {
    auto drone = std::make_shared<DroneComposition::Drone>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-r", "__node:=x500_1"}
        )
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(drone);

    std::promise<drone_interfaces::msg::Report> fly_report_promise;
    std::promise<drone_interfaces::msg::Report> ready_report_promise;
    auto fly_report_future = fly_report_promise.get_future();
    auto ready_report_future = ready_report_promise.get_future();

    auto report_sub = node->create_subscription<drone_interfaces::msg::Report>(
        "/x500_1/report", 
        10,
        [&fly_report_promise, &ready_report_promise](drone_interfaces::msg::Report::SharedPtr msg) {
            if (msg->state == 1) { // FLY
                fly_report_promise.set_value(*msg);
            }
            if (msg->state == 0) { // READY
                ready_report_promise.set_value(*msg);
            }
        }
    );

    auto mission_pub = node->create_publisher<drone_interfaces::msg::Mission>("/x500_1/mission", 10);

    drone_interfaces::msg::Mission mission;
    mission.poses.resize(1);
    mission.poses[0].position.x = 1.0;
    mission.poses[0].position.y = 1.0;
    mission.poses[0].position.z = 1.0;
    mission.velocities.push_back(1.0);
    mission.mission_type = "relocate";
    mission.id_from = "d1";
    mission.id_to = "d2";
    mission_pub->publish(mission);

    // Ждём report с состоянием FLY
    bool got_fly = false;
    for (int i = 0; i < 30; ++i) {
        exec.spin_some();
        if (fly_report_future.wait_for(0ms) == std::future_status::ready) {
            got_fly = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    ASSERT_TRUE(got_fly) << "Не дождались report с состоянием FLY";

    // Ждём report с состоянием READY (миссия завершена)
    bool got_ready = false;
    for (int i = 0; i < 50; ++i) {
        exec.spin_some();
        if (ready_report_future.wait_for(0ms) == std::future_status::ready) {
            got_ready = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    ASSERT_TRUE(got_ready) << "Не дождались report с состоянием READY";

    auto ready_report = ready_report_future.get();
    EXPECT_NEAR(ready_report.pose.position.x, 1.0, 0.05);
    EXPECT_NEAR(ready_report.pose.position.y, 1.0, 0.05);
    EXPECT_NEAR(ready_report.pose.position.z, 1.0, 0.05);
}

TEST_F(DroneCompositionTest, DroneControllerMissionExecution) {
    auto drone = std::make_shared<DroneComposition::Drone>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-r", "__node:=x500_1"}
        )
    );

    auto controller = std::make_shared<DroneComposition::DroneController>(
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-p", "drones_file:=/workspaces/swarm_of_drons/tests/test5/drones.csv"}
        )
    );

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(drone);    
    exec.add_node(controller);

    std::promise<drone_interfaces::msg::Report> fly_report_promise;
    std::promise<drone_interfaces::msg::Report> ready_report_promise;
    auto fly_report_future = fly_report_promise.get_future();
    auto ready_report_future = ready_report_promise.get_future();

    auto report_sub = node->create_subscription<drone_interfaces::msg::Report>(
        "/x500_1/report", 
        10,
        [&fly_report_promise, &ready_report_promise](drone_interfaces::msg::Report::SharedPtr msg) {
            if (msg->state == 1) { // FLY
                fly_report_promise.set_value(*msg);
            }
            if (msg->state == 0) { // READY
                ready_report_promise.set_value(*msg);
            }
        }
    );

    auto mission_pub = node->create_publisher<drone_interfaces::msg::GlobalMission>("/global_mission", 10);

    drone_interfaces::msg::GlobalMission mission;
    mission.start_time.sec = static_cast<int>(node->now().seconds());
    mission.drone_id = 1;
    mission.poses.resize(1);
    mission.poses[0].position.x = 1.0;
    mission.poses[0].position.y = 1.0;
    mission.poses[0].position.z = 1.0;
    mission.velocities.push_back(1.0);
    mission.mission_type = "execute";
    mission.id_from = "d1";
    mission.id_to = "d2";

    mission_pub->publish(mission);

    // Ждём report с состоянием FLY
    bool got_fly = false;
    for (int i = 0; i < 30; ++i) {
        exec.spin_some();
        if (fly_report_future.wait_for(0ms) == std::future_status::ready) {
            got_fly = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    ASSERT_TRUE(got_fly) << "Не дождались report с состоянием FLY";

    bool got_ready = false;
    for (int i = 0; i < 50; ++i) {
        exec.spin_some();
        if (ready_report_future.wait_for(0ms) == std::future_status::ready) {
            got_ready = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    ASSERT_TRUE(got_ready) << "Не дождались report с состоянием READY";

    auto ready_report = ready_report_future.get();
    EXPECT_NEAR(ready_report.pose.position.x, 1.0, 0.05);
    EXPECT_NEAR(ready_report.pose.position.y, 1.0, 0.05);
    EXPECT_NEAR(ready_report.pose.position.z, 1.0, 0.05);
}