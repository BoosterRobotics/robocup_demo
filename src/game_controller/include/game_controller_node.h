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

    // 初始化 UDP Socket
    void init();

    // 进入循环，接收 UDP 广播消息，处理后发布到 Ros2 Topic 中
    void spin();

private:
    // 检查收到的包是否来自白名单机器
    bool check_ip_white_list(string ip);

    // 处理数据包（逐字段复制）
    void handle_packet(RoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg);

    // 监听端口，从配置文件读
    int _port;
    // 是否启用 IP 白名单
    bool _enable_ip_white_list;
    // IP 白名单列表
    vector<string> _ip_white_list;

    // UDP Socket
    int _socket;
    // thread
    
    thread _thread;

    // Ros2 publisher
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr _publisher;
};