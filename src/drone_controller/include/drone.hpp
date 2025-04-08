#pragma once

#include <cstddef>
#include <string>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/duration.hpp>

#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "drone_interfaces/msg/mission.hpp"

#include "config.hpp"

struct Point {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct Vector3 {
    double x;
    double y;
    double z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3 (double x, double y, double z) : x(x), y(y), z(z) {}
    Vector3 (Point p1, Point p2) {
        x = p2.x - p1.x;
        y = p2.y - p1.y;
        z = p2.z - p1.z;
    }

    double length() {
        return sqrt(x * x + y * y + z * z);
    }

    Vector3 normalize() {
        double len = length();
        return {x / len, y / len, z / len};
    }

    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
};

class Drone : public rclcpp::Node {
private:
    using MsgMissionT     = drone_interfaces::msg::Mission;
    using MsgMissionPtrT  = MsgMissionT::SharedPtr;
    using MsgReportT      = std_msgs::msg::UInt16;
    using MsgReportPtrT   = MsgReportT::SharedPtr;
    using MsgLocationT    = geometry_msgs::msg::Pose;
    using MsgLocationPtrT = MsgLocationT::SharedPtr;

public:
    Drone(const std::string &nodeName, int id, Point pos, rclcpp::QoS& qos)
    : Node(nodeName), id(id), nodeName(nodeName), currentPosition(pos) {
        RCLCPP_INFO(get_logger(), "Drone %s_%d: Created", nodeName.c_str(), id);

        RCLCPP_INFO(get_logger(), "Drone %s_%d: Mission Subscription Init", nodeName.c_str(), id);
        missionSubscription = create_subscription<MsgMissionT>(
            getMissionTopic(), 
            qos, 
            [this](const MsgMissionPtrT msg) {
                this->missionHandler(msg);
            }
        );
        RCLCPP_INFO(get_logger(), "Drone %s_%d: Report Publisher Init", nodeName.c_str(), id);
        reportPublisher = create_publisher<MsgReportT>(
            getReportTopic(), 
            qos
        );
        RCLCPP_INFO(get_logger(), "Drone %s_%d: Location Publisher Init", nodeName.c_str(), id);
        locationPublisher = create_publisher<geometry_msgs::msg::Pose>(
            getLocationTopic(),
            qos
        );
    }

private:
    std::string getMissionTopic() {
        return nodeName + "_" + std::to_string(id) + "/mission";
    }

    std::string getReportTopic() {
        return nodeName + "_" + std::to_string(id) + "/report";
    }

    std::string getLocationTopic() {
        return nodeName + "_" + std::to_string(id) + "/location";
    }

    void flight(Point targetPoint, Vector3 velocity) {
        Point newPos = currentPosition;
        double newDistance = Vector3(currentPosition, targetPoint).length();
        
        rclcpp::Time time = this->now();
        double curDistance = Vector3(currentPosition, targetPoint).length();        

        rclcpp::Rate rate(DRONE_FLIGHT_RATE, this->get_clock());
        do {
            geometry_msgs::msg::Pose pose;
            pose.position.x = currentPosition.x;
            pose.position.y = currentPosition.y;
            pose.position.z = currentPosition.z;

            locationPublisher->publish(pose);

            rclcpp::Time tmpTime = this->now();
            rclcpp::Duration dTime = tmpTime - time;
            time = tmpTime;
       
            currentPosition = newPos;
            curDistance = newDistance; 
            newPos.x += velocity.x * (dTime.seconds() + 10e-9 * dTime.nanoseconds());
            newPos.y += velocity.y * (dTime.seconds() + 10e-9 * dTime.nanoseconds());
            newPos.z += velocity.z * (dTime.seconds() + 10e-9 * dTime.nanoseconds());

            newDistance = Vector3(newPos, targetPoint).length();

            rate.sleep();
        } while(newDistance < curDistance);
    }

    void missionHandler(const MsgMissionPtrT msg) {
        RCLCPP_INFO(get_logger(), "Drone %s_%d: Mission started", nodeName.c_str(), id);
        
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            Point targetPoint{
                msg->poses[i].position.x, 
                msg->poses[i].position.y,
                msg->poses[i].position.z
            };
            Vector3 velocity = Vector3{currentPosition, targetPoint}.normalize() * msg->velocities[i];

            flight(targetPoint, velocity);
        }
    }

private:
    int id;
    std::string nodeName;
    Point currentPosition;
    rclcpp::Publisher<MsgReportT>::SharedPtr     reportPublisher;
    rclcpp::Publisher<MsgLocationT>::SharedPtr   locationPublisher;
    rclcpp::Subscription<MsgMissionT>::SharedPtr missionSubscription;
};
