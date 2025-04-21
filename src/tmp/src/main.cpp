#include <rclcpp/rclcpp.hpp>
#include "navigator_interfaces/srv/free_drone.hpp"

class SimpleClient : public rclcpp::Node {
public:
    SimpleClient() : Node("simple_client") {
        client_ = this->create_client<navigator_interfaces::srv::FreeDrone>("free_drone_service");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { send_request(); }
        );
    }

    void send_request() {
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }

        auto request_ = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
        request_->pose_dronport.position.x = 45.0;
        request_->pose_dronport.position.y = 12.0;
        request_->pose_dronport.position.z = 5.0;

        auto result_future = client_->async_send_request(
            request_,
            [this](rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedFuture result) {
                if (result.valid()) {
                    RCLCPP_INFO(this->get_logger(), "Request sent");
                    RCLCPP_INFO(this->get_logger(), "Result drone %s_%d", result.get()->model.c_str(), result.get()->id);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get a response");
                }
            }
        );

        if (result_future.valid()) {
            auto result = result_future.get();
            if (result) {
                RCLCPP_INFO(this->get_logger(), "Result: %s_%d", result->model.c_str(), result->id);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get a response");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Future is not valid");
        }
    }

private:
    rclcpp::Client<navigator_interfaces::srv::FreeDrone>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<navigator_interfaces::srv::FreeDrone>("free_drone_service");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return 1;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<navigator_interfaces::srv::FreeDrone::Request>();
    request->pose_dronport.position.x = 41;
    request->pose_dronport.position.y = 12;
    request->pose_dronport.position.z = 5;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
      client->remove_pending_request(result_future);
      return 1;
    }
    auto result = result_future.get();
    RCLCPP_INFO(
      node->get_logger(), "Result: %s_%d",
      result->model.c_str(), result->id);
    rclcpp::shutdown();
    return 0;
}