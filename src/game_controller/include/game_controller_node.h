#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "RoboCupGameControlData.h"

#include "game_controller_interface/msg/game_control_data.hpp"

using namespace std;

class GameControllerNode : public rclcpp::Node
{
public:
    GameControllerNode(string name);
    ~GameControllerNode();

    void init();

    void spin();

private:
    bool check_ip_white_list(string ip);

    void handle_packet(HlRoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg);

    int _port;
    bool _enable_ip_white_list;
    vector<string> _ip_white_list;

    // UDP Socket
    int _socket;
    // thread
    
    thread _thread;

    // Ros2 publisher
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr _publisher;
};