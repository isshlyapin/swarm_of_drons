#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

#include <magic_enum.hpp>
#include <fastcsv/csv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_composition/drone_controller.hpp"
#include "drone_composition/drone.hpp"

namespace DroneComposition {

DroneController::DroneController(const rclcpp::NodeOptions& options)
  : Node("DroneController", options)
{
    RCLCPP_INFO(this->get_logger(), "DroneController: Creating...");

    declare_parameter("glob_miss_qos", 100);
    declare_parameter("drone_odm_qos", 100);
    declare_parameter("drone_mis_qos", 25);
    declare_parameter("drone_rep_qos", 25);

    declare_parameter("drones_file", "");
    declare_parameter("find_free_drone_rate", 0.2);
    declare_parameter("free_drone_service_qos_depth", 10);

    communicationInit();    
    
    dronesContextInit();
}

void DroneController::communicationInit() {
    RCLCPP_INFO(this->get_logger(), "DroneController: Global mission subscription init");
    globalMissionSubscription = create_subscription<MsgGlobalMissionT>(
        "global_mission",
        this->get_parameter_or("glob_miss_qos", 100),
        [this](const MsgGlobalMissionPtrT msg) {
            this->globalMissionHandler(msg);
        }
    );

    RCLCPP_INFO(this->get_logger(), "DroneController: Free drone service init");
    freeDroneSrvCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
    freeDroneService = this->create_service<SrvFreeDroneT>(
        "free_drone_service",
        [this](const std::shared_ptr<SrvFreeDroneRequestT> request,
                     std::shared_ptr<SrvFreeDroneResponseT> response) {
            this->freeDroneSrvHandler(request, response);
        },
        get_parameter("free_drone_service_qos_depth").as_int(),
        freeDroneSrvCallbackGroup
    );
}

void DroneController::dronesContextInit() {
    RCLCPP_INFO(this->get_logger(), "DroneController: Drones context init started");
    std::string pathToDronesCSV = get_parameter("drones_file").as_string();
    if (pathToDronesCSV.empty()) {
        RCLCPP_FATAL(this->get_logger(), "DroneController: Drones file path is empty");
        throw rclcpp::exceptions::InvalidParameterValueException(
            "Drones file path is empty"
        );
    }

    io::CSVReader<6> in(pathToDronesCSV);

    in.read_header(io::ignore_extra_column, "model", "id", "state", "x", "y", "z");

    int id;
    int state;
    double x, y, z;
    std::string model;

    while (in.read_row(model, id, state, x, y, z)) {
        if (drones.find(id) != drones.end()) {
            RCLCPP_ERROR(this->get_logger(), "DroneController: Non-unique drone id");
            continue;
        }
        if (!magic_enum::enum_contains<DroneState>(state)) {
            RCLCPP_ERROR(this->get_logger(), "DroneController: Bad value for state drone");
            continue;
        }

        addDroneContext(id, model, magic_enum::enum_cast<DroneState>(state).value(), 
                        Point{x, y, z});        
    }
}

void DroneController::addDroneContext(int id, const std::string& model, 
                                      DroneState state, const Point& position) {
    auto contextPtr = std::make_shared<DroneContext>();
    contextPtr->id = id;
    contextPtr->model = model;
    contextPtr->setState(state);
    contextPtr->setPosition(position);

    contextPtr->missionPublisher = this->create_publisher<Drone::MsgMissionT>(
        Drone::getMissionTopic(contextPtr->getFullName()),
        this->get_parameter("drone_mis_qos").as_int()
    );

    contextPtr->reportSubscription = this->create_subscription<Drone::MsgReportT>(
        Drone::getReportTopic(contextPtr->getFullName()),
        this->get_parameter("drone_rep_qos").as_int(),
        [this](const Drone::MsgReportPtrT msg) {
            this->reportHandler(msg);
        }
    );

    contextPtr->odometrySubscription = this->create_subscription<Drone::MsgOdometryT>(
        Drone::getOdometryTopic(contextPtr->getFullName()),
        this->get_parameter("drone_odm_qos").as_int(),
        [this, id](const Drone::MsgOdometryPtrT msg) {
            this->odometryHandler(msg, id);
        }
    );

    drones[id] = contextPtr;
}

void DroneController::freeDroneSrvHandler (
    const std::shared_ptr<SrvFreeDroneRequestT> request,
          std::shared_ptr<SrvFreeDroneResponseT> response )
{
    RCLCPP_INFO(this->get_logger(), "DroneController: Start find free drones");
    std::vector<std::shared_ptr<DroneContext>> freeDrones;
    findFreeDrones(freeDrones);
    
    Point targetPoint{
        request->pose_dronport.position.x, 
        request->pose_dronport.position.y,
        request->pose_dronport.position.z
    };

    auto nearDroneContext = getNearDroneContext(freeDrones, targetPoint);

    fieldFreeDroneSrvResponse(nearDroneContext, response);

    RCLCPP_INFO(this->get_logger(), "DroneController: Free drone [%s_%d] found",
        nearDroneContext->model.c_str(), nearDroneContext->id);
}

void DroneController::fieldFreeDroneSrvResponse(
    const std::shared_ptr<DroneContext> contextPtr,
          std::shared_ptr<SrvFreeDroneResponseT> response) {
    response->id = contextPtr->id;
    response->model = contextPtr->model;

    auto position = contextPtr->getPosition();
    response->pose_drone.position.x = position.x();
    response->pose_drone.position.y = position.y();
    response->pose_drone.position.z = position.z();
}

void DroneController::findFreeDrones(std::vector<std::shared_ptr<DroneContext>>& v) {
    while (v.empty()) {
        rclcpp::Rate rate{
            get_parameter("find_free_drone_rate").as_double(), 
            this->get_clock()
        };

        for (const auto& [id, contextPtr] : drones) {
            if (contextPtr->isFree()) {
                v.push_back(contextPtr);
            }
        }

        rate.sleep();
    }
}

std::shared_ptr<DroneContext> DroneController::getNearDroneContext(
    const std::vector<std::shared_ptr<DroneContext>>& v, 
    const Point& target) {
    if (v.empty()) {
        throw std::runtime_error("Invalid input vector");
    }

    auto res = v[0];
    double resSqDistance = (v[0]->getPosition() - target).squaredNorm();
    for (const auto& context : v) {
        double sqDistance = (context->getPosition() - target).squaredNorm();
        if (sqDistance < resSqDistance) {
            res = context;
            resSqDistance = sqDistance;
        }
    }

    return res;
}

void DroneController::globalMissionHandler(const MsgGlobalMissionPtrT msg) {
    if (drones.find(msg->drone_id) == drones.end()) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "DroneController: Global mission received");

    drones[msg->drone_id]->incrementMissionCount();

    Drone::MsgMissionT mission;
    mission.poses      = msg->poses;
    mission.start_time = msg->start_time;
    mission.velocities = msg->velocities;
    mission.mission_type = msg->mission_type;
    mission.id_from   = msg->id_from;
    mission.id_to     = msg->id_to;

    std::thread{
        [this, msg, mission]() {
            if (this->now() < msg->start_time) {
                RCLCPP_INFO(this->get_logger(), "DroneController: Waiting for start time");
                RCLCPP_INFO(this->get_logger(), "DroneController: Start time: %d; now: %lf", 
                    msg->start_time.sec, this->now().seconds());
                rclcpp::Rate{
                    rclcpp::Time(msg->start_time) - this->now(),
                    this->get_clock()
                }.sleep();
            }
            RCLCPP_INFO(this->get_logger(), "DroneController: Mission for drone [%d] published", msg->drone_id);
            if (drones[msg->drone_id]->getState() != DroneState::READY) {
                RCLCPP_ERROR(this->get_logger(), "DroneController: Drone [%d] is not ready", msg->drone_id);
                return;
            }
            drones[msg->drone_id]->missionPublisher->publish(mission);
            RCLCPP_DEBUG(this->get_logger(), "DroneController: Start time: %d; now: %lf", 
                msg->start_time.sec, this->now().seconds());
        }
    }.detach();
}

void DroneController::reportHandler(const Drone::MsgReportPtrT msg) {
    if (drones.find(msg->id) == drones.end()) {
        RCLCPP_INFO(this->get_logger(), "DroneController: Report message for unknown drone");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "DroneController: Report received from drone [%s_%d]", msg->model.c_str(), msg->id);
    if (!magic_enum::enum_contains<DroneState>(msg->state)) {
        RCLCPP_ERROR(this->get_logger(), "DroneController: Drone [%s_%d] state invalid: %d", msg->model.c_str(), msg->id, msg->state);
    } else {
        DroneState state = magic_enum::enum_cast<DroneState>(msg->state).value();
        drones[msg->id]->setState(state);
        if (state == DroneState::READY) {
            drones[msg->id]->decrementMissionCount();
            Drone::Point lastPose{
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z
            };
            drones[msg->id]->setPosition(lastPose);
        }
    }
}

void DroneController::odometryHandler(const Drone::MsgOdometryPtrT msg, int droneId) {}

} // namespace DroneComposition

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DroneComposition::DroneController)
