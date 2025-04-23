#include <set>
#include <string>

#include <fastcsv/csv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_visual_controller.hpp"

DroneVisualController::DroneVisualController(const std::string &node_name)
: Node(node_name) 
{
    declare_parameter("drones_file", "");

    init();  
}

void DroneVisualController::init() {
    std::string pathToDronesCSV = get_parameter("drones_file").as_string();

    if (pathToDronesCSV.empty()) {
        RCLCPP_FATAL(this->get_logger(), "DroneVisualController: No drones file provided");
        throw rclcpp::exceptions::InvalidParameterValueException(
            "No drones file provided"
        );
    }

    io::CSVReader<6> in(pathToDronesCSV);

    in.read_header(io::ignore_extra_column, "model", "id", "state", "x", "y", "z");

    std::string model;
    int id;
    int state;
    double x, y, z;

    std::set<std::string> droneModels;

    while (in.read_row(model, id, state, x, y, z)) {
        RCLCPP_INFO(this->get_logger(), "DroneVisualController: Adding drone %s with id %d", model.c_str(), id);
        std::string droneID = model + "_" + std::to_string(id);
        if (droneModels.find(droneID) != droneModels.end()) {
            RCLCPP_ERROR(this->get_logger(), "DroneVisualController: Duplicate drone model %s", model.c_str());
            continue;       
        } else {
            droneModels.insert(droneID);
        }
        
        auto droneV = std::make_shared<DroneVisual>(droneID);
        
        drones.push_back(droneV);
    }
}