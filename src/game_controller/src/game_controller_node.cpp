#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "game_controller_node.h"

GameControllerNode::GameControllerNode(string name) : rclcpp::Node(name)
{
    _socket = -1;

    // 声明 Ros2 参数，注意在配置文件中新加的参数需要在这里显示声明
    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

    // 从配置中读取参数，注意把读取到的参数打印到日志中方便查问题
    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%ld)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%ld]: %s", i, _ip_white_list[i].c_str());
    }

    // 创建 publisher，发布到 /game_state
    _publisher = create_publisher<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    if (_socket >= 0)
    {
        // 关闭打开的文件描述符是个好习惯
        close(_socket);
    }

    if (_thread.joinable())
    {
        _thread.join();
    }
}

/**
 * 创建 Soket 并绑定到指定的端口
 */
void GameControllerNode::init()
{
    // 创建 socket，失败了直接抛异常
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    // 初始化地址
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    // INADDR_ANY 将监听本机所有网络接口，默认情况这样就可以
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    // 绑定地址，失败了就抛异常
    if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    // bind 成功后就可以开始从 socket 里接收数据了
    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);

    // 启用一个新的线程来接收数据，主线程进入 Node 自身的 spin，处理一些 Node 自己的服务
    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    // 用来获取远程地址
    sockaddr_in remote_addr;
    socklen_t remote_addr_len = sizeof(remote_addr);

    // data 和 msg 在循环内是复用的，后续更新代码需要注意一下这个点
    RoboCupGameControlData data;
    game_controller_interface::msg::GameControlData msg;

    // 进入循环
    while (rclcpp::ok())
    {
        // 从 socket 中接收数据包，期望的是接收完整的数据包
        ssize_t ret = recvfrom(_socket, &data, sizeof(data), 0, (sockaddr *)&remote_addr, &remote_addr_len);
        if (ret < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        // 获取远端 IP
        string remote_ip = inet_ntoa(remote_addr.sin_addr);

        // 接收到不完整的包或其它非法的包，忽略掉
        if (ret != sizeof(data))
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid length=%ld", remote_ip.c_str(), ret);
            continue;
        }

        if (data.version != GAMECONTROLLER_STRUCT_VERSION)
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid version: %d", remote_ip.c_str(), data.version);
            continue;
        }

        // 过滤 IP 白名单
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it", remote_ip.c_str());
            continue;
        }

        // 处理消息，把 data 数据 copy 到 msg
        handle_packet(data, msg);

        // 将消息发布到 Topic 中
        _publisher->publish(msg);

        RCLCPP_INFO(get_logger(), "handle packet successfully ip=%s, packet_number=%d", remote_ip.c_str(), data.packetNumber);
    }
}

/**
 * 检查 IP 是否在白名单里，如果未开启白名单或者在白名单里，返回 true，其它情况返回 false
 */
bool GameControllerNode::check_ip_white_list(string ip)
{
    // 没有开启或在白名单内，返回 true
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        if (ip == _ip_white_list[i])
        {
            return true;
        }
    }
    return false;
}

/**
 * 将 UDP 数据格式转成自定交 Ros2 message 格式（逐字段复制）
 * 如需更改，一定要仔细各字段
 */
void GameControllerNode::handle_packet(RoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg)
{

    // header 是固定长度 4
    for (int i = 0; i < 4; i++)
    {
        msg.header[i] = data.header[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.competition_type = data.competitionType;
    msg.stopped = data.stopped;
    msg.game_phase = data.gamePhase;
    msg.state = data.state;
    msg.set_play = data.setPlay;
    msg.first_half = data.firstHalf;
    msg.kicking_team = data.kickingTeam;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    // RCLCPP_INFO(get_logger(), "-------------------- GameController Data -------------------------");
    
    // RCLCPP_INFO(get_logger(), "header: %02x %02x %02x %02x, "
    //             "version=%d, packet_number=%d, players_per_team=%d",
    //             msg.header[0], msg.header[1], msg.header[2], msg.header[3],
    //             msg.version, msg.packet_number, msg.players_per_team);
    // RCLCPP_INFO(get_logger(), "competition_type=%d, stopped=%d, game_phase=%d, "
    //             "state=%d, set_play=%d, first_half=%d, kicking_team=%d, "
    //             "secs_remaining=%d, secondary_time=%d",
    //             msg.competition_type, msg.stopped, msg.game_phase,
    //             msg.state, msg.set_play, msg.first_half, msg.kicking_team,
    //             msg.secs_remaining, msg.secondary_time);

    // teams 是固定长度 2
    for (int i = 0; i < 2; i++)
    {
        msg.teams[i].team_number = data.teams[i].teamNumber;
        msg.teams[i].field_player_colour = data.teams[i].fieldPlayerColour;
        msg.teams[i].goalkeeper_colour = data.teams[i].goalkeeperColour;
        msg.teams[i].goalkeeper = data.teams[i].goalkeeper;
        msg.teams[i].score = data.teams[i].score;
        msg.teams[i].penalty_shot = data.teams[i].penaltyShot;
        msg.teams[i].single_shots = data.teams[i].singleShots;
        msg.teams[i].message_budget = data.teams[i].messageBudget;

        // players is fixed-size in new interface message.
        int players_len = sizeof(data.teams[i].players) / sizeof(data.teams[i].players[0]);
        for (int j = 0; j < players_len; j++)
        {
            msg.teams[i].players[j].penalty = data.teams[i].players[j].penalty;
            msg.teams[i].players[j].secs_till_unpenalised = data.teams[i].players[j].secsTillUnpenalised;
            msg.teams[i].players[j].warnings = data.teams[i].players[j].warnings;
            msg.teams[i].players[j].cautions = data.teams[i].players[j].cautions;
        }
    }
}