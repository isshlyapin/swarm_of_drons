#include <rclcpp/rclcpp.hpp>

#include "drone_visual.hpp"
#include "csv.hpp"

#include <magic_enum.hpp>
#include <unordered_map>

class DroneVisualController : public rclcpp::Node {
public:
    DroneVisualController(const std::string &node_name)
      : Node(node_name) {
    }

    void init(std::string pathToDronesCSV) {
        io::CSVReader<6> in(pathToDronesCSV);
        in.read_header(io::ignore_extra_column, "model", "id", "state", "x", "y", "z");

        std::string model;
        int id;
        int state;
        double x, y, z;


        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

        while (in.read_row(model, id, state, x, y, z)) {
            RCLCPP_INFO(this->get_logger(), "DroneVisualController: Adding drone %s with id %d", model.c_str(), id);
            std::string droneID = model + "_" + std::to_string(id);
            if (drones.find(droneID) != drones.end()) {
                RCLCPP_ERROR(this->get_logger(), "DroneVisualController: Non-unique drone id");
                continue;
            }
            
            auto droneV = std::make_shared<DroneVisual>(droneID);
            
            drones[droneID] = droneV;

            executor->add_node(droneV);
        }

        RCLCPP_INFO(this->get_logger(), "DroneVisualController: Executor thread started");
        executorThread = std::thread(
            [this]() {
                executor->spin();
            }
        );
    }

private:
    std::unordered_map<std::string, DroneVisual::SharedPtr> drones;
    rclcpp::Executor::SharedPtr executor;
    std::thread executorThread;
};