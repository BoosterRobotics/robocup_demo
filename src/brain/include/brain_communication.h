#pragma once

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>
#include <atomic>
#include <thread>


#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include "utils/print.h"

static_assert(sizeof(RoboCupGameControlReturnData) == 32,
              "GameController v20 return packet layout mismatch");


class Brain; // 向前声明

using namespace std;


class BrainCommunication
{
public:
    BrainCommunication(Brain *argBrain);
    ~BrainCommunication();
    
    void initCommunication();
    void setGameControllerProtocolVersion(uint16_t version) { _game_controller_protocol_version.store(version); }

private:
    Brain *brain;

    void initGameControllerUnicast();
    std::thread _gamecontrol_unicast_thread;
    void unicastToGameController();
    void clearupGameControllerUnicast();
    std::atomic<bool> _unicast_gamecontrol_flag{false};
    int _gc_send_socket = -1;
    sockaddr_in _gcsaddr{};
    HlRoboCupGameControlReturnData gc_return_data;
    RoboCupGameControlReturnData gc_return_data_v20;
    // Do not send a response until an actual GameController packet selects the protocol.
    std::atomic<uint16_t> _game_controller_protocol_version{0};
    static constexpr int BROADCAST_GAME_CONTROL_INTERVAL_MS = 1000;
    static constexpr int GAME_CONTROLLER_PROTOCOL_WAIT_INTERVAL_MS = 50;

    void initTeamCommunication();
    void clearupTeamCommunication();
    void broadcastTeamCommunication();
    void spinTeamCommunicationReceiver();
    int _team_communication_msg_id = 0;
    std::atomic<bool> _team_communication_flag{false};
    std::thread _team_broadcast_thread;
    std::thread _team_receive_thread;
    int _team_socket = -1;
    int _team_udp_port = 0;
    sockaddr_in _team_broadcast_addr{};
    static constexpr int TEAM_COMMUNICATION_INTERVAL_MS = 100;
};
