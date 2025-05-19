#include <sys/types.h>

#include <magic_enum.hpp>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "drone_composition/drone.hpp"

namespace DroneComposition {

Drone::Drone(const rclcpp::NodeOptions & options)
: Node("Drone", options), realName(this->get_name())
{
    model = realName.substr(0, realName.rfind('_'));
    id = std::stoi(realName.substr(realName.rfind('_') + 1));

    RCLCPP_INFO(this->get_logger(), "Drone %s: Created", realName.c_str());

    declare_parameter("flight_rate", 50.0);
    declare_parameter("time_scale", 1.0);
    declare_parameter("qos_report_publisher", 25);
    declare_parameter("qos_odometry_publisher", 100);
    declare_parameter("qos_mission_subscription", 10);

    declare_parameter("pose_x", 0.0);
    declare_parameter("pose_y", 0.0);
    declare_parameter("pose_z", 0.0);

    currentPosition.x() = get_parameter("pose_x").as_double();
    currentPosition.y() = get_parameter("pose_y").as_double();
    currentPosition.z() = get_parameter("pose_z").as_double();

    int qos_rep = get_parameter("qos_report_publisher").as_int();
    int qos_odm = get_parameter("qos_odometry_publisher").as_int();
    int qos_mis = get_parameter("qos_mission_subscription").as_int();

    RCLCPP_INFO(this->get_logger(), "Drone %s: Mission subscription init", realName.c_str());
    missionSubscription = create_subscription<MsgMissionT>(
        getMissionTopic(), 
        qos_mis,
        [this](const MsgMissionPtrT msg) {
            this->missionHandler(msg);
        }
    );

    RCLCPP_INFO(this->get_logger(), "Drone %s: Report Publisher Init", realName.c_str());
    reportPublisher = create_publisher<MsgReportT>(
        getReportTopic(), 
        qos_rep
    );

    RCLCPP_INFO(this->get_logger(), "Drone %s: Odometry Publisher Init", realName.c_str());
    odometryPublisher = create_publisher<MsgOdometryT>(
        getOdometryTopic(),
        qos_odm
    );

    logsPublisher = create_publisher<std_msgs::msg::String>(
        "/panel_logs",
        25
    );

    RCLCPP_INFO(this->get_logger(), "Drone %s: flight_rate = %g", realName.c_str(), 
        get_parameter("flight_rate").as_double());
}

void Drone::flight(Point targetPoint, Vector3 velocity) {
    Point newPos = currentPosition;
    double newDistance = (targetPoint - currentPosition).squaredNorm();

    double curDistance = (targetPoint - currentPosition).squaredNorm();
    
    rclcpp::Time curTime = this->now();

    rclcpp::Rate rate{
        get_parameter("flight_rate").as_double() *
        get_parameter("time_scale").as_double(),
        this->get_clock()
    };

    rclcpp::Duration dTime{0, 0};

    do {
        rate.sleep();

        MsgOdometryT odometry;
        odometry.pose.pose.position.x = currentPosition.x();
        odometry.pose.pose.position.y = currentPosition.y();
        odometry.pose.pose.position.z = currentPosition.z();

        odometry.header.stamp = curTime;
        odometry.header.frame_id = "map";

        odometry.child_frame_id  = "base_link";

        odometry.twist.twist.linear.x = velocity.x();
        odometry.twist.twist.linear.y = velocity.y();
        odometry.twist.twist.linear.z = velocity.z();

        odometryPublisher->publish(odometry);

        const rclcpp::Time tmpTime = this->now();
        dTime = tmpTime - curTime;
        if (dTime == rclcpp::Duration(0, 0)) {
            continue;
        }
        curTime = tmpTime;
   
        currentPosition = newPos;
        curDistance     = newDistance;

        newPos.x() += velocity.x() * (dTime.seconds());
        newPos.y() += velocity.y() * (dTime.seconds());
        newPos.z() += velocity.z() * (dTime.seconds());
        
        newDistance = (targetPoint - newPos).squaredNorm();

        // RCLCPP_INFO(get_logger(), "Drone %s: Current distance: %.2f", realName.c_str(), curDistance);
    } while(newDistance < curDistance || dTime == rclcpp::Duration(0, 0));
}

void Drone::missionHandler(const MsgMissionPtrT msg) {
    RCLCPP_INFO(get_logger(), "Drone %s: Mission started", realName.c_str());

    sendReport(DroneState::FLY);
    sendLog(msg);

    ssize_t start_index = 0;
    if (msg->poses.size() == msg->velocities.size() + 1) {
        start_index = 1;
        Point startPoint{
            msg->poses[0].position.x, 
            msg->poses[0].position.y,
            msg->poses[0].position.z
        };
        if ((startPoint - currentPosition).squaredNorm() < 1) {
            RCLCPP_ERROR(get_logger(), "Drone %s: Start point is not current position", realName.c_str());
            sendReport(DroneState::ERROR);
            return;
        }
    } else if (msg->poses.size() == msg->velocities.size()) {
        start_index = 0;
    } else {
        RCLCPP_ERROR(get_logger(), "Drone %s: Invalid mission data", realName.c_str());
        sendReport(DroneState::ERROR);
        return;
    }
    
    for (size_t i = start_index; i < msg->poses.size(); ++i) {
        Point targetPoint{
            msg->poses[i].position.x, 
            msg->poses[i].position.y,
            msg->poses[i].position.z
        };
        RCLCPP_INFO(get_logger(), "Drone %s: Target point %ld: [%.2f, %.2f, %.2f]", 
            realName.c_str(), i, targetPoint.x(), targetPoint.y(), targetPoint.z());

        if ((targetPoint - currentPosition).squaredNorm() < 1e-6) {
            RCLCPP_INFO(get_logger(), "Drone %s: Target point %ld is too close", realName.c_str(), i);
            continue;
        }

        Vector3 velocity = (targetPoint - currentPosition).normalized() * msg->velocities[i];
        RCLCPP_INFO(get_logger(), "Drone %s: Velocity: [%.2f, %.2f, %.2f]", 
            realName.c_str(), velocity.x(), velocity.y(), velocity.z());

        flight(targetPoint, velocity);
    }

    RCLCPP_INFO(get_logger(), "Drone %s: Mission finished", realName.c_str());

    sendReport(DroneState::READY);
}

void Drone::sendLog(const MsgMissionPtrT msg) {
    std_msgs::msg::String logMsg;
    logMsg.data = realName + " ";
    if (msg->mission_type == "relocate") {
        logMsg.data += "relocates";
    } else if (msg->mission_type == "execute") {
        logMsg.data += "executes mission";
    } else {
        logMsg.data += "unknown mission type";
    }
    logMsg.data += " from " + msg->id_from + " to " + msg->id_to;
    logsPublisher->publish(
        logMsg
    );
}

void Drone::sendReport(DroneState state) {
    MsgReportT report;
    report.model = realName.substr(0, realName.rfind('_'));
    report.id = id;
    report.state = magic_enum::enum_integer(state);

    report.pose.position.x = currentPosition.x();
    report.pose.position.y = currentPosition.y();
    report.pose.position.z = currentPosition.z();

    reportPublisher->publish(report);
    RCLCPP_INFO(get_logger(), "Drone %s: Report sent", realName.c_str());
}

} // namespace DroneComposition

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DroneComposition::Drone)