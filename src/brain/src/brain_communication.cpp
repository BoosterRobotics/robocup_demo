#include "brain.h"
#include "brain_communication.h"

BrainCommunication::BrainCommunication(Brain *argBrain) : brain(argBrain)
{
}

BrainCommunication::~BrainCommunication()
{
    clearupGameControllerUnicast();

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    local_addr.sin_port = htons(_discovery_udp_port);
    _broadcast_discovery_flag.store(false);
    _receive_discovery_flag.store(false);
    // Send an empty packet to unblock send and receive threads
    int ret = sendto(_discovery_send_socket, nullptr, 0, 0, (sockaddr *)&local_addr, sizeof(local_addr));
    if (ret < 0){
        RCLCPP_ERROR(brain->get_logger(), "Failed to send empty packet to self to unblock discovery send socket: %s", strerror(errno));
    }
    clearupDiscoveryBroadcast();
    clearupDiscoveryReceiver();

    _unicast_communication_flag.store(false);
    _receive_communication_flag.store(false);
    // Send an empty packet to unblock send and receive threads
    local_addr.sin_port = htons(_unicast_udp_port);
    ret = sendto(_unicast_socket, nullptr, 0, 0, (sockaddr *)&local_addr, sizeof(local_addr));
    if (ret < 0) {
       RCLCPP_ERROR(brain->get_logger(), "Failed to send empty packet to self to unblock communication send socket: %s", strerror(errno));
    }
    clearupCommunicationUnicast();
    clearupCommunicationReceiver();
}

void BrainCommunication::initCommunication()
{
    initGameControllerUnicast();
    
    if (brain->config->get_enable_com())
    {
        int teamId = brain->config->get_team_id();
        cout << RED_CODE << "Communication enabled." << RESET_CODE << endl;
        _discovery_udp_port = 20000 + teamId;
        _unicast_udp_port = 30000 + teamId;

        initDiscoveryBroadcast();
        initDiscoveryReceiver();
        RCLCPP_INFO(brain->get_logger(), "InitCommunication Discovery enabled. UDP port: %d", _discovery_udp_port);

        initCommunicationUnicast();
        initCommunicationReceiver();
        RCLCPP_INFO(brain->get_logger(), "InitCommunication Communication enabled. UDP port: %d", _unicast_udp_port);
    }
    else
    {
        cout << RED_CODE << "Communication disabled." << RESET_CODE << endl;
    }
}

// soccer agent will switch team_id, determine whether the team has changed based on the port and the current teamId
bool BrainCommunication::isTeamChanged(){
    int teamId = brain->config->get_team_id();
    if (teamId != (_discovery_udp_port - 20000) || teamId != (_unicast_udp_port - 30000)) {
        RCLCPP_INFO(brain->get_logger(), "Team changed current discovery_port=%d unicast_port=%d, team_id to %d", 
        _discovery_udp_port, _unicast_udp_port, teamId);
        return true;
    }
    return false;
}
    

void BrainCommunication::initGameControllerUnicast()
{
    try
    {
        _gc_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_gc_send_socket < 0)
        {
        cout << RED_CODE << format("gc socket failed: %s", strerror(errno))
            << RESET_CODE << endl;
        throw std::runtime_error(strerror(errno));
        }
        // Configure target address
        string gamecontrol_ip = brain->config->get_game_control_ip();
        cout << GREEN_CODE << format("GameControl IP: %s", gamecontrol_ip.c_str())
            << RESET_CODE << endl;
        _gcsaddr.sin_family = AF_INET;
        _gcsaddr.sin_addr.s_addr = inet_addr(gamecontrol_ip.c_str());
        _gcsaddr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);

        _unicast_gamecontrol_flag.store(true);
        _gamecontrol_unicast_thread = std::thread([this](){ this->unicastToGameController(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::clearupGameControllerUnicast()
{
    _unicast_gamecontrol_flag.store(false);
    if (_gc_send_socket >= 0)
    {
        close(_gc_send_socket);
        _gc_send_socket = -1;
        cout << RED_CODE << format("GameControl send socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_gamecontrol_unicast_thread.joinable())
    {
        _gamecontrol_unicast_thread.join();
    }
}

void BrainCommunication::initDiscoveryBroadcast()
{
    try
    {
        _discovery_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_send_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Set broadcast options
        int broadcast = 1;
        if (setsockopt(_discovery_send_socket, SOL_SOCKET, SO_BROADCAST, 
                    &broadcast, sizeof(broadcast)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_BROADCAST: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Configure broadcast address
        _saddr.sin_family = AF_INET;
        _saddr.sin_addr.s_addr = INADDR_BROADCAST;  // 255.255.255.255
        _saddr.sin_port = htons(_discovery_udp_port);

        _broadcast_discovery_flag.store(true);
        _discovery_broadcast_thread = std::thread([this](){ this->broadcastDiscovery(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", format("Failed to initialize discovery broadcast: %s", e.what()));
    }
    
}

void BrainCommunication::clearupDiscoveryBroadcast()
{
    _broadcast_discovery_flag.store(false);
    if (_discovery_send_socket >= 0)
    {
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
        cout << RED_CODE << format("Discovery send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_discovery_broadcast_thread.joinable())
    {
        _discovery_broadcast_thread.join();
    }
}

void BrainCommunication::initDiscoveryReceiver()
{
    try
    {
        _discovery_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_recv_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Allow address reuse
        int reuse = 1;
        if (setsockopt(_discovery_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive data from all network interfaces
        addr.sin_port = htons(_discovery_udp_port);
        
        if (bind(_discovery_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _discovery_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        cout << GREEN_CODE << format("Listening for UDP broadcast on port %d", _discovery_udp_port)
            << RESET_CODE << endl;

        _receive_discovery_flag.store(true);
        _discovery_recv_thread = std::thread([this](){ this->spinDiscoveryReceiver(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", format("Failed to initialize discovery receiver: %s", e.what()));
    }
}

void BrainCommunication::clearupDiscoveryReceiver()
{
    // _receive_discovery_flag.store(false);
    if (_discovery_recv_socket >= 0)
    {
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        cout << RED_CODE << format("Discovery receiver socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_discovery_recv_thread.joinable())
    {
        _discovery_recv_thread.join();
    }
}

void BrainCommunication::unicastToGameController() {
    while (_unicast_gamecontrol_flag.load(std::memory_order_relaxed))
    {
        gc_return_data.team = brain->config->get_team_id();
        gc_return_data.player = brain->config->get_player_id(); // return data id is 1,2,3,4
        gc_return_data.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

        int ret = sendto(_gc_send_socket, &gc_return_data, sizeof(gc_return_data), 0, (sockaddr *)&_gcsaddr, sizeof(_gcsaddr));
        if (ret < 0)
        {
            cout << RED_CODE << format("gc sendto failed: %s", strerror(errno))
                << RESET_CODE << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_GAME_CONTROL_INTERVAL_MS));
    }
}

void BrainCommunication::broadcastDiscovery() {
    while (_broadcast_discovery_flag.load(std::memory_order_relaxed))
    {
        TeamDiscoveryMsg msg;

        msg.communicationId = _discovery_msg_id++;
        msg.teamId = brain->config->get_team_id();
        msg.playerId = brain->config->get_player_id();

        int ret = sendto(_discovery_send_socket, &msg, sizeof(msg), 0, (sockaddr *)&_saddr, sizeof(_saddr));
        if (ret < 0)
        {
            cout << RED_CODE << format("sendto failed: %s", strerror(errno))
                << RESET_CODE << endl;
        }

        brain->data->discoveryMsgId = msg.communicationId;
        brain->data->discoveryMsgTime = brain->get_clock()->now();
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_DISCOVERY_INTERVAL_MS));
    } 
}

void BrainCommunication::spinDiscoveryReceiver() {    
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamDiscoveryMsg msg;

    while (_receive_discovery_flag.load(std::memory_order_relaxed)) {

        ssize_t len = recvfrom(_discovery_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0)
        {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (len != sizeof(TeamDiscoveryMsg)) {
            cout << YELLOW_CODE << format("received TeamDiscoveryMsg packet with wrong size: %ld, expected: %ld", len, sizeof(TeamDiscoveryMsg))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_DISCOVERY) { // fail to pass validation
            cout << RED_CODE << format("received TeamDiscoveryMsg packet with invalid validation: %d", msg.validation)
                << RESET_CODE << endl;
            continue;
        } 

        if (msg.teamId != brain->config->get_team_id()) { // Ignore messages from other teams
            cout << YELLOW_CODE << format("Received message from team %d, expected team %d", msg.teamId, brain->config->get_team_id())
                << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId == brain->config->get_player_id()) {  // Ignore own messages
            continue;
        } else {

            
            auto time_now = brain->get_clock()->now();
            std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
            _teammate_addresses[addr.sin_addr.s_addr] = {
                addr.sin_addr.s_addr,
                msg.playerId,
                time_now
            };
        }
    }
}

void BrainCommunication::cleanupExpiredTeammates() {
    std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);    
    for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end();) {
        auto timeDiff = this->brain->get_clock()->now().nanoseconds() - it->second.lastUpdate.nanoseconds();
        if (timeDiff > TEAMMATE_TIMEOUT_MS * 1e6) {
            cout << YELLOW_CODE << format("Teammate id %d timed out", it->second.playerId) 
                 << RESET_CODE << endl;
            it = _teammate_addresses.erase(it);
        } else {
            ++it;
        }
    }
}

void BrainCommunication::initCommunicationUnicast() {
    try
    {
        _unicast_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_unicast_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error("Failed to create unicast socket");
        }

        _unicast_saddr.sin_family = AF_INET;
        _unicast_saddr.sin_port = htons(_unicast_udp_port);

        _unicast_communication_flag.store(true);
        _unicast_thread = std::thread([this](){ this->unicastCommunication(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", format("Failed to initialize unicast communication: %s", e.what()));
    }
    
}

void BrainCommunication::unicastCommunication() {
    auto log = [=](string msg) {
        brain->log->debug("sendMsg", msg);
    };
    while (_unicast_communication_flag) {
        cleanupExpiredTeammates();
        TeamCommunicationMsg msg;
        msg.validation = VALIDATION_COMMUNICATION;
        msg.communicationId = _team_communication_msg_id++;
        msg.teamId = brain->config->get_team_id();
        msg.playerId = brain->config->get_player_id();
        
        string role = brain->tree->getEntry<string>("player_role");
        if (role == "striker") {
            msg.playerRole = 1;
        } else if (role == "goal_keeper") {
            msg.playerRole = 2;
        } else {
            msg.playerRole = 3; // Unknown role
        }

        msg.isAlive = brain->data->tmImAlive;
        msg.isLead = brain->data->tmImLead;
        msg.ballDetected = brain->data->ballDetected;
        msg.ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
        msg.ballConfidence = brain->data->ball.confidence;
        msg.ballRange = brain->data->ball.range;
        msg.cost = brain->data->tmMyCost;
        msg.ballPosToField = brain->data->ball.posToField;
        msg.robotPoseToField = brain->data->robotPoseToField;
        msg.kickDir = brain->data->kickDir;
        msg.thetaRb = brain->data->robotBallAngleToField;
        msg.cmdId = brain->data->tmMyCmdId;
        msg.cmd = brain->data->tmMyCmd;
        log(format("ImAlive: %d, ImLead: %d, myCost: %.1f, myCmdId: %d, myCmd: %d", msg.isAlive, msg.isLead, msg.cost, msg.cmdId, msg.cmd));

        std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
        for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end(); ++it) {
            auto ip = it->second.ip;

            brain->data->tmIP = inet_ntoa(*(in_addr *)&ip);
            brain->data->sendId = msg.communicationId;
            brain->data->sendTime = brain->get_clock()->now();
            
            _unicast_saddr.sin_addr.s_addr = ip;
            int ret = sendto(_unicast_socket, &msg, sizeof(msg), 0, (sockaddr *)&_unicast_saddr, sizeof(_unicast_saddr));
            if (ret < 0) {
                cout << RED_CODE << format("sendto failed: %s", strerror(errno))
                    << RESET_CODE << endl;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(UNICAST_INTERVAL_MS));
    }
}

void BrainCommunication::clearupCommunicationUnicast() {
    // _unicast_communication_flag.store(false);
    if (_unicast_socket >= 0) {
        close(_unicast_socket);
        _unicast_socket = -1;
        cout << RED_CODE << format("Communication send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_unicast_thread.joinable()) {
        _unicast_thread.join();
    }
}

void BrainCommunication::initCommunicationReceiver() {
    try
    {
        _communication_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_communication_recv_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Allow address reuse
        int reuse = 1;
        if (setsockopt(_communication_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(_unicast_udp_port);
        
        if (bind(_communication_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0) {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _unicast_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        _receive_communication_flag.store(true);
        _communication_recv_thread = std::thread([this](){ this->spinCommunicationReceiver(); });
    
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", format("Failed to initialize communication receiver: %s", e.what()));
    }
}

void BrainCommunication::spinCommunicationReceiver() {
    auto log = [=](string msg) {
        brain->log->debug("receiveMsg", msg);
    };
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamCommunicationMsg msg;

    while (_receive_communication_flag.load(std::memory_order_relaxed)) {

        ssize_t len = recvfrom(_communication_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0) {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (len != sizeof(TeamCommunicationMsg)) {
            cout << YELLOW_CODE << format("received TeamCommunicationMsg packet with wrong size: %ld, expected: %ld", len, sizeof(TeamCommunicationMsg))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_COMMUNICATION) { // fail to pass validation
            cout << RED_CODE << format("received TeamCommunicationMsg packet with invalid validation: %d", msg.validation)
                << RESET_CODE << endl;
            continue;
        }

        if (msg.teamId != brain->config->get_team_id()) { // Ignore messages from other teams
            cout << YELLOW_CODE << format("Received message from team %d, expected team %d", msg.teamId, brain->config->get_team_id())
                << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId == brain->config->get_player_id()) {  // Ignore own messages
            // Handle own messages
            cout << CYAN_CODE <<  format(
                "communicationId: %d, alive: %d, ballDetected: %d ballRange: %.2f playerId: %d",
                msg.communicationId, msg.isAlive, msg.ballDetected, msg.ballRange, msg.playerId)
                << RESET_CODE << endl;
            brain->data->sendId = msg.communicationId;
            brain->data->sendTime = brain->get_clock()->now();
            continue;
        } 

        auto tmIdx = msg.playerId - 1;

        if (tmIdx < 0 || tmIdx >= HL_MAX_NUM_PLAYERS) { // HL_MAX_NUM_PLAYERS is the maximum number of players
            cout << YELLOW_CODE << format("Received message with invalid playerId: %d", msg.playerId) << RESET_CODE << endl;
            continue;
        }

        if (brain->data->penalty[tmIdx] == SUBSTITUTE) { // Ignore substitute players' information
            cout << YELLOW_CODE << format("Communication playerId %d is substitute", msg.playerId) << RESET_CODE << endl;
            continue;
        }

        log(format("TMID: %.d, alive: %d, lead: %d, cost: %.1f, CmdId: %d, Cmd: %d", msg.playerId, msg.isAlive, msg.isLead, msg.cost, msg.cmdId, msg.cmd));

        TMStatus &tmStatus = brain->data->tmStatus[tmIdx];
        
        switch(msg.playerRole) {
            case 1: tmStatus.role = "striker"; break;
            case 2: tmStatus.role = "goal_keeper"; break;
            default: tmStatus.role = "unknown"; break;
        }
        tmStatus.isAlive = msg.isAlive;
        tmStatus.ballDetected = msg.ballDetected;
        tmStatus.ballLocationKnown = msg.ballLocationKnown;
        tmStatus.ballConfidence = msg.ballConfidence;
        tmStatus.ballRange = msg.ballRange;
        tmStatus.cost = msg.cost;
        tmStatus.isLead = msg.isLead;
        tmStatus.ballPosToField = msg.ballPosToField;
        tmStatus.robotPoseToField = msg.robotPoseToField;
        tmStatus.kickDir = msg.kickDir;
        tmStatus.thetaRb = msg.thetaRb;
        tmStatus.timeLastCom = brain->get_clock()->now();
        tmStatus.cmd = msg.cmd;
        tmStatus.cmdId = msg.cmdId;

        // Check if a new command has been received
        if (msg.cmdId > brain->data->tmCmdId) {
            brain->data->tmCmdId = msg.cmdId;
            brain->data->tmReceivedCmd = msg.cmd;
            brain->data->tmLastCmdChangeTime = brain->get_clock()->now();
            log(format("Received new command from teammate %d: %d", msg.playerId, msg.cmd));
        }

    }
}

void BrainCommunication::clearupCommunicationReceiver() {
    // _receive_communication_flag.store(false);
    if (_communication_recv_socket >= 0) {
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_communication_recv_thread.joinable()) {
        _communication_recv_thread.join();
    }
}
