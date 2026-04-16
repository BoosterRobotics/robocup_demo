#pragma once

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>


#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include "utils/print.h"


class Brain; // forward declaration

using namespace std;


class BrainCommunication
{
public:
    BrainCommunication(Brain *argBrain);
    ~BrainCommunication();
    
    void initCommunication();

private:
    Brain *brain;

    void initGameControllerUnicast();
    std::thread _gamecontrol_unicast_thread;
    void unicastToGameController();
    void clearupGameControllerUnicast();
    bool _unicast_gamecontrol_flag = false;
    int _gc_send_socket = -1;
    sockaddr_in _gcsaddr;
    RoboCupGameControlReturnData gc_return_data;
    static constexpr int BROADCAST_GAME_CONTROL_INTERVAL_MS = 500; // 2 packets per second

    void initTeamBroadcast();
    void clearupTeamBroadcast();
    void broadcastTeamCommunication();
    std::thread _team_broadcast_thread;
    bool _broadcast_team_flag = false;
    int _team_send_socket = -1;
    int _team_udp_port = 0;
    sockaddr_in _team_saddr;
    static constexpr int BROADCAST_TEAM_INTERVAL_MS = 500; // 2 packets per second (complies with rules)
    int _team_communication_msg_id = 0;
    int _counted_messages = 0; // 仅在 READY/SET/PLAY 状态计数的消息数量

    void initTeamReceiver();
    void clearupTeamReceiver();
    void spinTeamReceiver();
    bool _receive_team_flag = false;
    std::thread _team_recv_thread;
    int _team_recv_socket = -1;
};