#pragma once

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "navigator2/map.hpp"
#include "navigator2/mission_manager.hpp"

#include "navigator_interfaces/srv/free_drone.hpp"
#include "drone_interfaces/msg/global_mission.hpp"

class Navigator : public rclcpp::Node {
public:
    using SrvFreeDroneT = navigator_interfaces::srv::FreeDrone;
    using SrvFreeDroneRequestT = navigator_interfaces::srv::FreeDrone::Request;
    using SrvFreeDroneResponseT = navigator_interfaces::srv::FreeDrone::Response;

    using MsgGlobalMissionT = drone_interfaces::msg::GlobalMission;
    using MsgGlobalMissionPtrT = MsgGlobalMissionT::SharedPtr;

    Navigator();

private:
    void initCommunication();

    void timerCallback();

    void publishGlobalMission(const Mission& map_mission, 
                                    uint32_t drone_id,
                          const std::string& type);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<SrvFreeDroneT>::SharedPtr free_drone_client_;
    rclcpp::Publisher<MsgGlobalMissionT>::SharedPtr global_mission_publisher_;

    std::shared_ptr<rclcpp::Node> client_node_;

    Map map_;
    MissionManager mission_manager_;
};