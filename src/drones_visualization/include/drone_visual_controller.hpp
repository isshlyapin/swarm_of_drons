#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "drone_visual.hpp"

class DroneVisualController : public rclcpp::Node {
public:
    DroneVisualController(const std::string &node_name);

    void init();

    const std::vector<DroneVisual::SharedPtr> &getDrones() const {
        return drones;
    }

private:
    std::vector<DroneVisual::SharedPtr> drones;
};