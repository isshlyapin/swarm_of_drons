#include <fastcsv/csv.hpp>
#include <magic_enum.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>

#include "drone_controller.hpp"

DroneController::DroneController(const std::string& nodeName) 
  : Node(nodeName) 
{
    RCLCPP_INFO(this->get_logger(), "DroneController: Creating...");

    declare_parameter("glob_miss_qos", 100);
    declare_parameter("drone_odm_qos", 100);
    declare_parameter("drone_mis_qos", 25);
    declare_parameter("drone_rep_qos", 25);

    declare_parameter<std::string>("drones_file", "");

    if (get_parameter("drones_file").as_string().empty()) {
        RCLCPP_FATAL(this->get_logger(), "DroneController: Drones file not set");
        throw rclcpp::exceptions::InvalidParameterValueException(
            "DroneController: Drones file not set"
        );
    }

    int glob_miss_qos = get_parameter("glob_miss_qos").as_int();

    RCLCPP_INFO(this->get_logger(), "DroneController: Global mission subscription init");
    globalMissionSubscription = create_subscription<MsgGlobalMissionT>(
        "global_mission",
        glob_miss_qos,
        [this](const MsgGlobalMissionPtrT msg) {
            this->globalMissionHandler(msg);
        }
    );

    init(get_parameter("drones_file").as_string());
    RCLCPP_INFO(this->get_logger(), "DroneController: Drones init");
}

void DroneController::init(std::string pathToDronesCSV) {
    io::CSVReader<6> in(pathToDronesCSV);

    in.read_header(io::ignore_extra_column, "model", "id", "state", "x", "y", "z");

    std::string model;
    int id;
    int state;
    double x, y, z;

    while (in.read_row(model, id, state, x, y, z)) {
        if (drones.find(id) != drones.end()) {
            RCLCPP_ERROR(this->get_logger(), "DroneController: Non-unique drone id");
            continue;
        }
        if (!magic_enum::enum_contains<DroneState>(state)) {
            RCLCPP_ERROR(this->get_logger(), "DroneController: Bad value for state drone");
            continue;
        }

        auto drone = std::make_shared<Drone>(model, id, Point{x, y, z});
        
        DroneInfo droneInfo;
        droneInfo.drone = drone;
        droneInfo.count_missions = 0;
        droneInfo.state = magic_enum::enum_cast<DroneState>(state).value();
        droneInfo.missionPublisher = this->create_publisher<MsgMissionT>(
            drone->getMissionTopic(),
            this->get_parameter("drone_mis_qos").as_int()
        );
        droneInfo.reportSubscription = this->create_subscription<MsgReportT>(
            drone->getReportTopic(),
            this->get_parameter("drone_rep_qos").as_int(),
            [this](const MsgReportPtrT msg) {
                this->reportHandler(msg);
            }
        );
        droneInfo.odometrySubscription = this->create_subscription<MsgOdometryT>(
            drone->getOdometryTopic(),
            this->get_parameter("drone_odm_qos").as_int(),
            [this](const MsgOdometryPtrT msg) {
                this->odometryHandler(msg);
            }
        );

        drones[id] = droneInfo;
    }
}

std::vector<Drone::SharedPtr> DroneController::getDrones() const {
    std::vector<Drone::SharedPtr> dronesList;
    for (const auto& [id, droneInfo] : drones) {
        dronesList.push_back(droneInfo.drone);
    }
    return dronesList;
}

void DroneController::globalMissionHandler(const MsgGlobalMissionPtrT msg) {
    if (drones.find(msg->drone_id) == drones.end()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "DroneController: Global mission received");

    drones[msg->drone_id].count_missions++;

    MsgMissionT mission;
    mission.poses      = msg->poses;
    mission.start_time = msg->start_time;
    mission.velocities = msg->velocities;

    std::thread{
        [this, msg, mission]() {
            if (this->now() < msg->start_time) {
                RCLCPP_INFO(this->get_logger(), "DroneController: Waiting for start time");
                rclcpp::Rate{
                    rclcpp::Time(msg->start_time) - this->now(),
                    this->get_clock()
                }.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "DroneController: Mission for drone [%d] published", msg->drone_id);
            drones[msg->drone_id].missionPublisher->publish(mission);
        }
    }.detach();
}

void DroneController::reportHandler(const MsgReportPtrT msg) {
    if (drones.find(msg->id) == drones.end()) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "DroneController: Report received from drone [%s_%d]", msg->model.c_str(), msg->id);
    if (!magic_enum::enum_contains<DroneState>(msg->state)) {
        RCLCPP_ERROR(this->get_logger(), "DroneController: Drone [%s_%d] state invalid: %d", msg->model.c_str(), msg->id, msg->state);
    } else {
        drones[msg->id].state = magic_enum::enum_cast<DroneState>(msg->state).value();
        if (drones[msg->id].state == DroneState::READY) {
            drones[msg->id].count_missions--;
        }
    }
}

void DroneController::odometryHandler(const MsgOdometryPtrT msg) {}
