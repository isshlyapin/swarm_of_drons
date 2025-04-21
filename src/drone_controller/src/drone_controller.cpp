#include <vector>

#include <magic_enum.hpp>
#include <fastcsv/csv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_controller.hpp"

DroneController::DroneController(const std::string& nodeName) 
  : Node(nodeName) 
{
    RCLCPP_INFO(this->get_logger(), "DroneController: Creating...");

    declare_parameter("glob_miss_qos", 100);
    declare_parameter("drone_odm_qos", 100);
    declare_parameter("drone_mis_qos", 25);
    declare_parameter("drone_rep_qos", 25);

    declare_parameter("find_free_drone_rate", 0.2);

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

    freeDroneService = this->create_service<SrvFreeDroneT>(
        "free_drone_service",
        [this](const std::shared_ptr<SrvFreeDroneT::Request> request,
                     std::shared_ptr<SrvFreeDroneT::Response> response) {
            this->freeDroneHandler(request, response);
        }
    );

    init(get_parameter("drones_file").as_string());
    RCLCPP_INFO(this->get_logger(), "DroneController: Drones init");
}

std::vector<int> DroneController::findFreeDrone() {
    std::vector<int> freeDrones;
    while (freeDrones.empty()) {
        rclcpp::Rate rate{
            get_parameter("find_free_drone_rate").as_double(), 
            this->get_clock()
        };

        for (const auto& [droneId, droneInfo] : drones) {
            if (droneInfo.getState() == DroneState::READY && droneInfo.getMissionCount() == 0) {
                freeDrones.push_back(droneId);
            }
        }

        rate.sleep();
    }

    return freeDrones;
}

int DroneController::getNearDroneID(std::vector<int>& v, Point target) {
    if (v.empty()) {
        throw std::runtime_error("Invalid input vector");
    }

    int nearDroneID = v[0];
    double nearSqDistance = (drones.at(nearDroneID).drone->getCurrentPosition() - target).squaredNorm();
    for (const auto& droneId : v) {
        double sqDistance = (drones.at(droneId).drone->getCurrentPosition() - target).squaredNorm();
        if (sqDistance < nearSqDistance) {
            nearDroneID = droneId;
            nearSqDistance = sqDistance;
        }
    }

    return nearDroneID;
}

void DroneController::freeDroneHandler (
    const std::shared_ptr<SrvFreeDroneT::Request> request,
          std::shared_ptr<SrvFreeDroneT::Response> response )
{
    Point targetPoint{
        request->pose_dronport.position.x, 
        request->pose_dronport.position.y,
        request->pose_dronport.position.z
    };
    auto freeDrones = findFreeDrone();

    auto nearDroneID = getNearDroneID(freeDrones, targetPoint);
    Point nearDronePosition = drones[nearDroneID].drone->getCurrentPosition();

    response->model = drones[nearDroneID].drone->getModel();
    response->id = nearDroneID;
    response->pose_drone.position.x = nearDronePosition.x();
    response->pose_drone.position.y = nearDronePosition.y();
    response->pose_drone.position.z = nearDronePosition.z();
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
        
        drones.emplace(id, DroneInfo{});
        drones[id].drone = drone;
        drones[id].setState(magic_enum::enum_cast<DroneState>(state).value());
        drones[id].missionPublisher = this->create_publisher<MsgMissionT>(
            drone->getMissionTopic(),
            this->get_parameter("drone_mis_qos").as_int()
        );
        drones[id].reportSubscription = this->create_subscription<MsgReportT>(
            drone->getReportTopic(),
            this->get_parameter("drone_rep_qos").as_int(),
            [this](const MsgReportPtrT msg) {
                this->reportHandler(msg);
            }
        );
        drones[id].odometrySubscription = this->create_subscription<MsgOdometryT>(
            drone->getOdometryTopic(),
            this->get_parameter("drone_odm_qos").as_int(),
            [this](const MsgOdometryPtrT msg) {
                this->odometryHandler(msg);
            }
        );
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

    drones[msg->drone_id].incrementMissionCount();

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
        drones[msg->id].setState(magic_enum::enum_cast<DroneState>(msg->state).value());
        if (drones[msg->id].getState() == DroneState::READY) {
            drones[msg->id].decrementMissionCount();
        }
    }
}

void DroneController::odometryHandler(const MsgOdometryPtrT msg) {}
