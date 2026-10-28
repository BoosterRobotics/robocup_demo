#pragma once

#include <tuple>
#include <vector>
#include <string>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include <rclcpp/rclcpp.hpp> 

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();
    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value{};
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;
    void initEntry();
};

// ------------------------------- 比赛用 -------------------------------

// 分析赛场状况
class Analyze : public SyncActionNode
{
public:
    Analyze(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override { return NodeStatus::SUCCESS; } 
private:
    Brain *brain;
};

// 计算踢球的角度
class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("cross_threshold", 0.2, "可进门的角度范围") }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense"),
            OutputPort<string>("decision_out")};
    }
    NodeStatus tick() override;
private:
    Brain *brain;
    double lastDeltaDir = 0.0;
    bool hasLastDeltaDir = false;
    rclcpp::Time timeLastTick;
};

class NewDecide : public SyncActionNode
{
public:
    NewDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("chase_threshold", 1.0, "") }; }
    NodeStatus tick() override { return NodeStatus::SUCCESS; }
private:
    Brain *brain;
};

// 守门员四态决策：guard_idle / guard_defend / block_kick / intercept_kick。
class Analyze2 : public SyncActionNode
{
public:
    Analyze2(const string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
    bool _interceptKickDisabled = false;
    bool _interceptKickCloseStartValid = false;
    rclcpp::Time _interceptKickCloseStartTime;
};

// 未见球时回到门线前固定锚点。
class GoalieIdleHome : public SyncActionNode
{
public:
    GoalieIdleHome(const string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("dist_to_goalline", 1.0, ""),
            InputPort<double>("vx_limit", 0.7, ""),
            InputPort<double>("vy_limit", 0.5, ""),
            InputPort<double>("vtheta_limit", 1.2, ""),
            InputPort<double>("pos_tolerance", 0.12, ""),
            InputPort<double>("theta_tolerance", 0.25, ""),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
    bool _positionLocked = false;
};

class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);
    NodeStatus tick() override;
private:
    double _cmdSequence[6][2];
    rclcpp::Time _timeLastCmd;
    int _cmdIndex;
    long _cmdIntervalMSec;
    long _cmdRestartIntervalMSec;
    Brain *brain;
};

class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("low_pitch", 0.35, "向下看时的最大 pitch"),
            InputPort<double>("high_pitch", 0.15, "向上看时的最小 pitch"),
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("msecs_interval", 300, "") }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    double _cmdSequence[7][2] = {
        {0.2, 1.1}, {0.2, 0.0}, {0.2, -1.1},
        {0.9, -1.1}, {0.9, 0.0}, {0.9, 1.1}, {0.2, 0.0},
    };
    rclcpp::Time _timeLastCmd;
    int _cmdIndex = 0;
    Brain *brain;
};

class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("vyaw_limit", 1.0, "") }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    double _turnDir = 1.0;
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("rad", 0, ""),
            InputPort<bool>("towards_ball", false, "")
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    double _lastAngle = 0.0;
    double _angle = 0.0;
    double _cumAngle = 0.0;
    double _msecLimit = 5000;
    rclcpp::Time _timeStart;
    Brain *brain;
};

class DecideCheckBehind : public SyncActionNode
{
public:
    DecideCheckBehind(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("vx_limit", 0.4, ""),
            InputPort<double>("vy_limit", 0.4, ""),
            InputPort<double>("vtheta_limit", 0.1, ""),
            InputPort<double>("dist", 1.0, ""),
            InputPort<double>("safe_dist", 1.0, ""),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
    string _state;
    double _dir = 1.0;
};

// 补充 Chase 变体定义
class Chase3 : public SyncActionNode { public: Chase3(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Chase4 : public SyncActionNode { public: Chase4(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Chase2 : public StatefulActionNode { public: Chase2(const string &n, const NodeConfig &c, Brain *b) : StatefulActionNode(n,c), brain(b) {} NodeStatus onStart() override { return NodeStatus::SUCCESS; } NodeStatus onRunning() override { return NodeStatus::SUCCESS; } void onHalted() override {} static PortsList providedPorts() { return {}; } private: Brain *brain; };

class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("turn_threshold", 0.2, ""),
            InputPort<double>("vx_limit", 0.1, ""),
            InputPort<double>("vy_limit", 0.1, ""),
            InputPort<double>("vtheta_limit", 0.4, ""),
            InputPort<double>("range", 1.5, ""),
            InputPort<double>("vtheta_factor", 1.5, ""),
            InputPort<double>("tangential_speed_far", 0.7, ""),
            InputPort<double>("tangential_speed_near", 0.15, ""),
            InputPort<double>("near_threshold", 0.8, ""),
            InputPort<double>("no_turn_threshold", 0.1, ""),
            InputPort<double>("turn_first_threshold", 0.5, ""),
            InputPort<double>("min_range", 1.0, ""),
            InputPort<string>("position", "offense", "")};
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

// 补充 Adjust 变体定义
class Adjust2 : public SyncActionNode { public: Adjust2(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust4 : public SyncActionNode { public: Adjust4(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust5 : public SyncActionNode { public: Adjust5(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class Adjust3 : public SyncActionNode { public: Adjust3(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };
class AdjustForPowerShoot : public SyncActionNode { public: AdjustForPowerShoot(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {} NodeStatus tick() override { return NodeStatus::SUCCESS; } static PortsList providedPorts() { return {}; } private: Brain *brain; };

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("min_msec_kick", 500, ""),
            InputPort<double>("msecs_stablize", 1000, ""),
            InputPort<double>("speed_limit", 1.2, ""),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    rclcpp::Time _startTime;
    string _state = "kick";
    int _msecKick = 1000;
    double _speed = 0.0;
    double _minRange = 0.0;
    tuple<double, double, double> _calcSpeed();
};

class Shoot : public StatefulActionNode
{
public:
    Shoot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("msecs", 2000, ""), InputPort<double>("speed", 0.0, "") }; }
    NodeStatus onStart() override { return NodeStatus::SUCCESS; }
    NodeStatus onRunning() override { return NodeStatus::SUCCESS; }
    void onHalted() override {};
private:
    Brain *brain;
};

class RLVisionKick : public StatefulActionNode
{
public:
    RLVisionKick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("max_msec_kick", 10000, "Maximum duration for the kick action (milliseconds)"),
            InputPort<double>("min_msec_kick", 3000, "Minimum duration for the kick action (milliseconds)"),
            InputPort<double>("range", 1.5, "Range within which the strategy is considered effective"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

    static rclcpp::Time getLastExitTime() { return _lastExitTime; }

    static bool isMinIntervalSatisfied(double minIntervalMsec);

private:
    Brain *brain;
    rclcpp::Time _startTime;
    rclcpp::Time _sessionStartTime;
    rclcpp::Time _lastProgressTime;
    rclcpp::Time _headScanStartTime;
    // VisualKick 接管头部后持续保持低头，避免 CamFindAndTrackBall 在后续帧抬头。
    rclcpp::Time _lastHeadCommandTime;
    bool _headCommandSent = false;

    bool _isDecelerating = false;
    bool _visionKickStarted = false;
    bool _pendingRobocupWalk = false;
    rclcpp::Time _decelStartTime;
    double _decelDurationMs = 500.0;
    int _extraRoundsUsed = 0;
    int _badPoseConsecutiveCount = 0;
    int _ballRangeOverLimitConsecutiveCount = 0;
    int _lostOwnershipConsecutiveCount = 0;
    bool _progressBaselineValid = false;
    double _progressBallRange = 0.0;
    Point _progressBallField{0.0, 0.0, 0.0};
    double _progressKickDir = 0.0;
    string _pendingExitReason;

    static rclcpp::Time _lastExitTime;

    void recordExitTime();
    void startDecelerate(double durationMs = 500.0);
    bool stepDecelerate();
};

// 站立横移封堵：威胁球跟随预测交点，其余情况跟随球的 y。
class Intercept2 : public SyncActionNode
{
public:
    Intercept2(const string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return { InputPort<double>("dist_to_goalline", 1.0, "") };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<int>("msecs", 1000, ""), }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    rclcpp::Time _startTime;
};

class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<string>("mode", "enter_field", ""),
            InputPort<double>("msecs_interval", 10000, ""),
        };
    };
private:
    Brain *brain;
};

// SelfLocate 变体 
class SelfLocateLine : public SyncActionNode {
public:
    SelfLocateLine(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
    double lineToLineAvgDist(const FieldLine& a, const FieldLineRef& b, int samples = 10);
};

class SelfLocateEnterField : public SyncActionNode {
public:
    SelfLocateEnterField(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return { InputPort<double>("msecs_interval", 1000, "") }; }
private:
    Brain *brain;
};

class SelfLocateLocal : public SyncActionNode {
public:
    SelfLocateLocal(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return { InputPort<double>("msecs_interval", 1000, "") };
    };
private:
    Brain *brain;
    bool _singlePenalty();
    bool _doubleX();
};

class SelfLocate1P : public SyncActionNode {
public:
    SelfLocate1P(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate1M : public SyncActionNode {
public:
    SelfLocate1M(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate2X : public SyncActionNode {
public:
    SelfLocate2X(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocate2T : public SyncActionNode {
public:
    SelfLocate2T(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocateLT : public SyncActionNode {
public:
    SelfLocateLT(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocatePT : public SyncActionNode {
public:
    SelfLocatePT(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class SelfLocateBorder : public SyncActionNode {
public:
    SelfLocateBorder(const string &n, const NodeConfig &c, Brain *b) : SyncActionNode(n,c), brain(b) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return {
            InputPort<double>("msecs_interval", 1000, ""),
            InputPort<double>("max_dist", 2.0, ""),
            InputPort<double>("max_drift", 1.0, ""),
            InputPort<bool>("validate", true, ""),
        };
    };
private:
    Brain *brain;
};

class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""),
            InputPort<double>("long_range_threshold", 1.5, ""), InputPort<double>("turn_threshold", 0.4, ""),
            InputPort<double>("vx_limit", 1.0, ""), InputPort<double>("vy_limit", 0.5, ""), InputPort<double>("vtheta_limit", 0.4, ""),
            InputPort<double>("x_tolerance", 0.2, ""), InputPort<double>("y_tolerance", 0.2, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<bool>("avoid_obstacle", false, "")
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.6, ""), InputPort<double>("theta_tolerance", 0.2, ""),
            InputPort<double>("vx_limit", 1.2, ""), InputPort<double>("vy_limit", 0.5, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
    // READY 阶段到达站位后锁存，避免定位噪声让机器人在目标点附近反复修正。
    bool _positionLocked = false;
    bool _lockedTargetValid = false;
    Pose2D _lockedTargetPose = {0.0, 0.0, 0.0};
    std::string _lastGameState;
};

class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<double>("vx_limit", 0.5, ""), InputPort<double>("vy_limit", 0.5, ""),
            InputPort<double>("dist_to_goalline", 1.0, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""), InputPort<double>("theta_tolerance", 0.1, ""),
            InputPort<double>("vx_limit", 1.0, ""), InputPort<double>("vy_limit", 0.6, ""),
            InputPort<double>("dist_to_goalline", 1.0, ""),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

// 开球阶段非当前半区进攻组的待命站位。
class KickoffStand : public SyncActionNode
{
public:
    KickoffStand(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config), brain(_brain) {}
    static BT::PortsList providedPorts() {
        return {
            InputPort<double>("dist_tolerance", 0.5, ""),
            InputPort<double>("theta_tolerance", 0.2, ""),
            InputPort<double>("vx_limit", 0.6, ""),
            InputPort<double>("vy_limit", 0.5, ""),
            InputPort<double>("hold_vtheta_limit", 0.6, "原地待命时的身体旋转速度"),
        };
    }
    BT::NodeStatus tick() override;
private:
    Brain *brain;
};

class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() {
        return { InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""), };
    }
private:
    Brain *brain;
};

/**
 * @brief Switch robot to robocup walking mode (kSoccer) and exit VisualKick(false)
 *
 * This node is intended to be executed once when the whole behavior tree starts.
 */
class RobocupWalk : public SyncActionNode
{
public:
    RobocupWalk(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
 
    NodeStatus tick() override;
 
private:
    Brain *brain;
};

class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return {}; }
private:
    Brain *brain;
};

class RoleSwitchIfNeeded : public SyncActionNode
{
public:
    RoleSwitchIfNeeded(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { }; }
private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("action", "start", ""), }; }
private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<double>("pitch", 0, ""), InputPort<double>("yaw", 0, ""), }; }
private:
    Brain *brain;
};

class CheckAndStandUp : public SyncActionNode
{
public:
    CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class MoveToFarSetPlaySearchArea : public StatefulActionNode
{
public:
    MoveToFarSetPlaySearchArea(const string &name, const NodeConfig &config, Brain *_brain)
        : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return {}; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    Pose2D _targetPose = {0.0, 0.0, 0.0};
    rclcpp::Time _startTime;
    double _maxMoveSecs = 30.0;
    double _vxLimit = 0.7;
    double _vyLimit = 0.4;
    bool _arrived = false;
    bool _moveTimedOut = false;
    int _direction = 0;
};

class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<string>("side", "attack", ""), InputPort<double>("attack_dist", 0.7, ""),
            InputPort<double>("defense_dist", 1.9, ""), InputPort<double>("vx_limit", 1.2, ""), InputPort<double>("vy_limit", 0.5, ""),
        };
    }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    Brain *brain;
    bool _isInFinalAdjust = false;
};

class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("valve", 0.5, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

// ------------------------------- FOR DEMO -------------------------------

class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() {
        return {
            InputPort<double>("stop_dist", 1.0, ""), InputPort<double>("stop_angle", 0.1, ""),
            InputPort<double>("vy_limit", 0.2, ""), InputPort<double>("vx_limit", 0.6, ""),
        };
    }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class CrabWalk : public SyncActionNode
{
public:
    CrabWalk(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("angle", 0, ""), InputPort<double>("speed", 0.5, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class AutoCalibrateVision : public StatefulActionNode
{
public:
    AutoCalibrateVision(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { }; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};
private:
    Brain *brain;
    rclcpp::Time _paramChangeTime;
    vector<vector<double>> _res; 
    double _calcResidual(); 
    int _index = -1;
    double _subTotal = 0.0;
    double _count = 0.0;
    double _bestResidual = 100.0;
};

class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    static PortsList providedPorts() { return { InputPort<double>("x", 0, ""), InputPort<double>("y", 0, ""), InputPort<double>("theta", 0, ""), }; }
    NodeStatus tick() override;
private:
    Brain *brain;
};

class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static PortsList providedPorts() { return {InputPort<std::string>("msg")}; }
private:
    Brain *brain;
};

class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("sound", "cheerful", ""), InputPort<bool>("allow_repeat", false, ""), }; }
private:
    Brain *brain;
};

class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
    NodeStatus tick() override;
    static BT::PortsList providedPorts() { return { InputPort<string>("text", "", ""), }; }
private:
    Brain *brain;
};
