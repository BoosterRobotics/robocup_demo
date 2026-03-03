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

    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

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

    _publisher = create_publisher<game_controller_interface::msg::GameControlData>("/booster_soccer/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    if (_socket >= 0)
    {
        close(_socket);
    }

    if (_thread.joinable())
    {
        _thread.join();
    }
}


void GameControllerNode::init()
{
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);
    RCLCPP_INFO(get_logger(), "Expected HlRoboCupGameControlData size: %ld", sizeof(HlRoboCupGameControlData));

    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    sockaddr_in remote_addr;
    socklen_t remote_addr_len = sizeof(remote_addr);

    HlRoboCupGameControlData data;
    game_controller_interface::msg::GameControlData msg;

    while (rclcpp::ok())
    {
        ssize_t ret = recvfrom(_socket, &data, sizeof(data), 0, (sockaddr *)&remote_addr, &remote_addr_len);
        if (ret < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        string remote_ip = inet_ntoa(remote_addr.sin_addr);

        if (ret != sizeof(data))
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid length=%ld, expected=%ld", remote_ip.c_str(), ret, sizeof(data));
            continue;
        }

        if (data.version != HL_GAMECONTROLLER_STRUCT_VERSION)
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid version: %d", remote_ip.c_str(), data.version);
            continue;
        }

        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it", remote_ip.c_str());
            continue;
        }

        handle_packet(data, msg);

        _publisher->publish(msg);

        RCLCPP_INFO(get_logger(), "handle packet successfully ip=%s, packet_number=%d", remote_ip.c_str(), data.packetNumber);
    }
}


bool GameControllerNode::check_ip_white_list(string ip)
{
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

void GameControllerNode::handle_packet(HlRoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg)
{

    // header is fixed length 4
    for (int i = 0; i < 4; i++)
    {
        msg.header[i] = data.header[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.game_type = data.gameType;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickOffTeam;
    msg.secondary_state = data.secondaryState;
    // secondary_state_info is fixed length 4
    for (int i = 0; i < 4; i++)
    {
        msg.secondary_state_info[i] = data.secondaryStateInfo[i];
    }
    msg.drop_in_team = data.dropInTeam;
    msg.drop_in_time = data.dropInTime;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    // teams is fixed length 2
    for (int i = 0; i < 2; i++)
    {
        msg.teams[i].team_number = data.teams[i].teamNumber;
        msg.teams[i].field_player_colour = data.teams[i].fieldPlayerColour;
        msg.teams[i].score = data.teams[i].score;
        msg.teams[i].penalty_shot = data.teams[i].penaltyShot;
        msg.teams[i].single_shots = data.teams[i].singleShots;
        msg.teams[i].coach_sequence = data.teams[i].coachSequence;

        int coach_message_len = sizeof(data.teams[i].coachMessage) / sizeof(data.teams[i].coachMessage[0]);
        msg.teams[i].coach_message.clear(); // because msg is reused, remember to clear it
        for (int j = 0; j < coach_message_len; j++)
        {
            msg.teams[i].coach_message.push_back(data.teams[i].coachMessage[j]);
        }

        // msg.teams[i].cocah
        msg.teams[i].coach.penalty = data.teams[i].coach.penalty;
        msg.teams[i].coach.secs_till_unpenalised = data.teams[i].coach.secsTillUnpenalised;
        msg.teams[i].coach.number_of_warnings = data.teams[i].coach.numberOfWarnings;
        msg.teams[i].coach.yellow_card_count = data.teams[i].coach.yellowCardCount;
        msg.teams[i].coach.red_card_count = data.teams[i].coach.redCardCount;
        msg.teams[i].coach.goal_keeper = data.teams[i].coach.goalKeeper;

        // msg.teams[i].coach_message is defined as a variable-length array, note the difference from fixed-length arrays
        int players_len = sizeof(data.teams[i].players) / sizeof(data.teams[i].players[0]);
        msg.teams[i].players.clear(); // because msg is reused, remember to clear it
        for (int j = 0; j < players_len; j++)
        {
            game_controller_interface::msg::RobotInfo rf;
            rf.penalty = data.teams[i].players[j].penalty;
            rf.secs_till_unpenalised = data.teams[i].players[j].secsTillUnpenalised;
            rf.number_of_warnings = data.teams[i].players[j].numberOfWarnings;
            rf.yellow_card_count = data.teams[i].players[j].yellowCardCount;
            rf.red_card_count = data.teams[i].players[j].redCardCount;
            rf.goal_keeper = data.teams[i].players[j].goalKeeper;
            msg.teams[i].players.push_back(rf);
        }
    }
}