#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
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
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%zu)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); ++i)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%zu]: %s", i, _ip_white_list[i].c_str());
    }

    _publisher = create_publisher<game_controller_interface::msg::GameControlData>(
        "/robocup/game_controller", 10);
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

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);
    if (bind(_socket, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);
    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    constexpr size_t packet_buffer_size =
        sizeof(HlRoboCupGameControlData) > sizeof(game_controller_protocol_v20::GameControlData)
            ? sizeof(HlRoboCupGameControlData)
            : sizeof(game_controller_protocol_v20::GameControlData);

    sockaddr_in remote_addr{};
    std::array<unsigned char, packet_buffer_size> packet{};
    game_controller_interface::msg::GameControlData msg;

    while (rclcpp::ok())
    {
        socklen_t remote_addr_len = sizeof(remote_addr);
        const ssize_t received = recvfrom(
            _socket, packet.data(), packet.size(), 0,
            reinterpret_cast<sockaddr *>(&remote_addr), &remote_addr_len);
        if (received < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        const string remote_ip = inet_ntoa(remote_addr.sin_addr);
        if (received < 4 || std::memcmp(packet.data(), GAMECONTROLLER_STRUCT_HEADER, 4) != 0)
        {
            RCLCPP_WARN(get_logger(), "packet from %s has invalid GameController header", remote_ip.c_str());
            continue;
        }
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it",
                        remote_ip.c_str());
            continue;
        }

        msg = game_controller_interface::msg::GameControlData();
        int protocol_version = -1;
        int packet_number = -1;

        if (received == static_cast<ssize_t>(sizeof(HlRoboCupGameControlData)))
        {
            HlRoboCupGameControlData data{};
            std::memcpy(&data, packet.data(), sizeof(data));
            if (data.version != HL_GAMECONTROLLER_STRUCT_VERSION)
            {
                RCLCPP_WARN(get_logger(), "legacy packet from %s has unsupported version: %d",
                            remote_ip.c_str(), data.version);
                continue;
            }
            handle_packet(data, msg);
            protocol_version = data.version;
            packet_number = data.packetNumber;
        }
        else if (received == static_cast<ssize_t>(sizeof(game_controller_protocol_v20::GameControlData)))
        {
            game_controller_protocol_v20::GameControlData data{};
            std::memcpy(&data, packet.data(), sizeof(data));
            if (data.version != game_controller_protocol_v20::kStructVersion)
            {
                RCLCPP_WARN(get_logger(), "v20-sized packet from %s has unsupported version: %d",
                            remote_ip.c_str(), data.version);
                continue;
            }
            handle_packet(data, msg);
            protocol_version = data.version;
            packet_number = data.packetNumber;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "packet from %s has unsupported length=%ld (expected legacy=%zu or v20=%zu)",
                remote_ip.c_str(), received, sizeof(HlRoboCupGameControlData),
                sizeof(game_controller_protocol_v20::GameControlData));
            continue;
        }

        _publisher->publish(msg);
        RCLCPP_INFO(get_logger(),
                    "handled GameController packet ip=%s, version=%d, packet_number=%d",
                    remote_ip.c_str(), protocol_version, packet_number);
    }
}

bool GameControllerNode::check_ip_white_list(string ip)
{
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (const auto &allowed_ip : _ip_white_list)
    {
        if (ip == allowed_ip)
        {
            return true;
        }
    }
    return false;
}

void GameControllerNode::handle_packet(
    HlRoboCupGameControlData &data,
    game_controller_interface::msg::GameControlData &msg)
{
    for (int i = 0; i < 4; ++i)
    {
        msg.header[i] = data.header[i];
        msg.secondary_state_info[i] = data.secondaryStateInfo[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.game_type = data.gameType;
    msg.stopped = false;
    msg.game_phase = 0;
    msg.set_play = 0;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickOffTeam;
    msg.secondary_state = data.secondaryState;
    msg.drop_in_team = data.dropInTeam;
    msg.drop_in_time = data.dropInTime;
    msg.secs_remaining = static_cast<int16_t>(data.secsRemaining);
    msg.secondary_time = static_cast<int16_t>(data.secondaryTime);

    for (int i = 0; i < 2; ++i)
    {
        const auto &source_team = data.teams[i];
        auto &target_team = msg.teams[i];
        target_team.team_number = source_team.teamNumber;
        target_team.field_player_colour = source_team.fieldPlayerColour;
        target_team.goalkeeper_colour = source_team.fieldPlayerColour;
        target_team.goalkeeper = 0;
        target_team.score = source_team.score;
        target_team.penalty_shot = source_team.penaltyShot;
        target_team.single_shots = source_team.singleShots;
        target_team.message_budget = 0;
        target_team.coach_sequence = source_team.coachSequence;
        target_team.coach_message.assign(
            source_team.coachMessage,
            source_team.coachMessage + sizeof(source_team.coachMessage));

        target_team.coach.penalty = source_team.coach.penalty;
        target_team.coach.secs_till_unpenalised = source_team.coach.secsTillUnpenalised;
        target_team.coach.number_of_warnings = source_team.coach.numberOfWarnings;
        target_team.coach.yellow_card_count = source_team.coach.yellowCardCount;
        target_team.coach.red_card_count = source_team.coach.redCardCount;
        target_team.coach.cautions = source_team.coach.yellowCardCount;
        target_team.coach.goal_keeper = source_team.coach.goalKeeper;

        target_team.players.clear();
        target_team.players.reserve(HL_MAX_NUM_PLAYERS);
        for (int j = 0; j < HL_MAX_NUM_PLAYERS; ++j)
        {
            const auto &source_player = source_team.players[j];
            game_controller_interface::msg::RobotInfo target_player;
            target_player.penalty = source_player.penalty;
            target_player.secs_till_unpenalised = source_player.secsTillUnpenalised;
            target_player.number_of_warnings = source_player.numberOfWarnings;
            target_player.yellow_card_count = source_player.yellowCardCount;
            target_player.red_card_count = source_player.redCardCount;
            target_player.cautions = source_player.yellowCardCount;
            target_player.goal_keeper = source_player.goalKeeper;
            if (target_player.goal_keeper)
            {
                target_team.goalkeeper = static_cast<uint8_t>(j + 1);
            }
            target_team.players.push_back(target_player);
        }
    }
}

void GameControllerNode::handle_packet(
    const game_controller_protocol_v20::GameControlData &data,
    game_controller_interface::msg::GameControlData &msg)
{
    using namespace game_controller_protocol_v20;

    for (int i = 0; i < 4; ++i)
    {
        msg.header[i] = data.header[i];
        msg.secondary_state_info[i] = 0;
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.game_type = data.competitionType;
    msg.stopped = data.stopped != 0;
    msg.game_phase = data.gamePhase;
    msg.set_play = data.setPlay;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickingTeam;
    msg.secondary_state = toLegacySecondaryState(data.gamePhase, data.setPlay);
    msg.secondary_state_info[0] = data.kickingTeam;
    msg.secondary_state_info[1] = data.stopped ? 0 : 1;
    msg.drop_in_team = kKickingTeamNone;
    msg.drop_in_time = 0xffff;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    for (int i = 0; i < 2; ++i)
    {
        const auto &source_team = data.teams[i];
        auto &target_team = msg.teams[i];
        target_team.team_number = source_team.teamNumber;
        target_team.field_player_colour = source_team.fieldPlayerColour;
        target_team.goalkeeper_colour = source_team.goalkeeperColour;
        target_team.goalkeeper = source_team.goalkeeper;
        target_team.score = source_team.score;
        target_team.penalty_shot = source_team.penaltyShot;
        target_team.single_shots = source_team.singleShots;
        target_team.message_budget = source_team.messageBudget;
        target_team.coach_sequence = 0;
        target_team.coach_message.clear();
        target_team.players.clear();
        target_team.players.reserve(kMaxNumPlayers);

        for (size_t j = 0; j < kMaxNumPlayers; ++j)
        {
            const auto &source_player = source_team.players[j];
            game_controller_interface::msg::RobotInfo target_player;
            target_player.penalty = source_player.penalty;
            target_player.secs_till_unpenalised = source_player.secsTillUnpenalised;
            target_player.number_of_warnings = 0;
            target_player.yellow_card_count = source_player.cautions;
            target_player.red_card_count = source_player.penalty == kPenaltySentOff ? 1 : 0;
            target_player.cautions = source_player.cautions;
            target_player.goal_keeper = source_team.goalkeeper == j + 1;
            target_team.players.push_back(target_player);
        }
    }
}
