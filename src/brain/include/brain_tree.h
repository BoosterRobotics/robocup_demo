#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

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

    /**
     * Initialize entries in the blackboard. When adding new fields, set a default value here.
     */
    void initEntry();
};


class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("cross_threshold", 0.2, "If the angle range for scoring is less than this value, then pass")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "If the distance exceeds this value, execute the chase action"),
            InputPort<string>("decision_in", "", "Used to read the previous decision"),
            InputPort<string>("position", "offense", "offense | defense, determines the direction to kick the ball"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; 
    rclcpp::Time timeLastTick; 
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "If the distance exceeds this value, execute the chase action"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "If the angle is less than this value, consider adjust successful"), //
            InputPort<double>("adjust_y_tolerance", 0.1, "If the y-direction offset is less than this value, consider y-direction adjust successful"),  //
            InputPort<string>("decision_in", "", "Used to read the previous decision"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "Minimum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "Maximum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "Angle range for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "Obstacle distance threshold for auto visual kick, if an obstacle is within this distance, auto visual kick is not executed"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "Obstacle angle threshold for auto visual kick, if an obstacle is within this angle, auto visual kick is not executed"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
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


class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "Maximum turning speed when searching for the ball"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; 
    Brain *brain;
};


class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 300, "How many milliseconds to stay at the same position"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1},
        {0.45, 0.0},
        {0.45, -1.1},
        {1.0, -1.1},
        {1.0, 0.0},
        {1.0, 1.1},
        {0.45, 0.0},
    };    
    rclcpp::Time _timeLastCmd;    
    int _cmdIndex = 0;               
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "How many radians to turn, positive for left"),
            InputPort<bool>("towards_ball", false, "If true, ignore the sign of rad and turn towards the last seen ball direction")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _lastAngle; 
    double _angle;
    double _cumAngle; 
    double _msecLimit = 5000;  
    rclcpp::Time _timeStart;
    Brain *brain;
};


class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "Maximum x speed when chasing the ball"),
            InputPort<double>("vy_limit", 0.4, "Maximum y speed when chasing the ball"),
            InputPort<double>("vtheta_limit", 1.0, "Maximum angular speed when chasing the ball"),
            InputPort<double>("dist", 0.1, "Target distance behind the ball when chasing"),
            InputPort<double>("safe_dist", 4.0, "Safe distance to maintain when circling back"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};


class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 3.25, "If the ball angle is greater than this value, the robot will turn to face the ball and pause linear movement"),
            InputPort<double>("vx_limit", 0.05, "Limit for vx during adjustment [-limit, limit]"),
            InputPort<double>("vy_limit", 0.05, "Limit for vy during adjustment [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.1, "Limit for vtheta during adjustment [-limit, limit]"),
            InputPort<double>("range", 2.25, "Maintain this ball range"),
            InputPort<double>("vtheta_factor", 3.0, "Multiplier for vtheta when adjusting angle, higher values result in faster turning"),
            InputPort<double>("tangential_speed_far", 0.2, "Tangential speed when far from the target"),
            InputPort<double>("tangential_speed_near", 0.15, "Tangential speed when near the target"),
            InputPort<double>("near_threshold", 0.8, "Use near speed when distance to target is less than this value"),
            InputPort<double>("no_turn_threshold", 0.1, "Do not turn if angle difference is less than this value"),
            InputPort<double>("turn_first_threshold", 0.5, "Turn first without moving if angle difference is greater than this value"),

        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Execute kick action
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "Minimum duration for the kick action (milliseconds)"),
            InputPort<double>("msecs_stablize", 1000, "Duration to stabilize before kicking (milliseconds)"),
            InputPort<double>("speed_limit", 0.8, "Maximum speed for the kick action"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
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
            InputPort<double>("auto_visual_kick_enable_dist_min", 0.0, "Minimum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 4.0, "Maximum distance for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "Angle range for enabling auto visual kick"),
            InputPort<double>("auto_visual_kick_enable_goal_angle", 0.35, "Angle range for enabling auto visual kick towards the goal"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 1.0, "Distance threshold for obstacles during auto visual kick, if an obstacle is within this distance, auto visual kick will not be executed"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "Angle threshold for obstacles in front during auto visual kick, if an obstacle is within this angle, auto visual kick will not be executed"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;
    
    static rclcpp::Time getLastExitTime() { return _lastExitTime; }
    
    static bool isMinIntervalSatisfied(double minIntervalMsec);

private:
    Brain *brain;
    rclcpp::Time _startTime;              // Start time of the kick action
    rclcpp::Time _headScanStartTime;      // Start time for head scanning
    
    // Non-blocking deceleration state
    bool _isDecelerating = false;         // Whether deceleration is in progress
    bool _visionKickStarted = false;     // Whether visual kick mode has started
    bool _pendingRobocupWalk = false;    // Whether waiting to switch to robocupWalk after deceleration
    rclcpp::Time _decelStartTime;        // Deceleration start time
    double _decelDurationMs = 500.0;     // Deceleration duration (milliseconds)
    
    // Static variable: records the last exit time of RLVisionKick to limit frequent switching
    static rclcpp::Time _lastExitTime;
    
    // Helper methods
    void recordExitTime();
    void startDecelerate(double durationMs = 500.0);
    bool stepDecelerate();
};

class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("msecs", 1000, "Duration to stand still (milliseconds)"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
};


class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.5, "Maximum pitch when looking down"),
            InputPort<double>("high_pitch", 0.2, "Minimum pitch when looking up"),
            InputPort<double>("left_yaw", 0.8, "Maximum yaw when looking left"),
            InputPort<double>("right_yaw", -0.8, "Minimum yaw when looking right"),
            InputPort<int>("msec_cycle", 4000, "Duration for a full scan cycle (milliseconds)"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Target x coordinate in the Field frame"),
            InputPort<double>("y", 0, "Target y coordinate in the Field frame"),
            InputPort<double>("theta", 0, "Target final orientation in the Field frame"),
            InputPort<double>("long_range_threshold", 1.5, "If the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "When the distance is long, if the direction to the target point exceeds this value, turn towards the target point first"),
            InputPort<double>("vx_limit", 0.8, "x speed limit"),
            InputPort<double>("vy_limit", 0.5, "y speed limit"),
            InputPort<double>("vtheta_limit", 0.2, "theta speed limit"),
            InputPort<double>("x_tolerance", 0.5, "x tolerance"),
            InputPort<double>("y_tolerance", 0.5, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),
            InputPort<bool>("avoid_obstacle", false, "Whether to avoid obstacles")
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

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "x tolerance"),
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),
            InputPort<double>("vx_limit", 0.8, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 2.5, "Distance from the robot to the goal line"),
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

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 2.5, "Distance from the robot to the goal line"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


/**
 * @brief Set the robot's velocity
 *
 * @param x,y,theta double, Robot's velocity in the x, y directions (m/s) and counterclockwise angular velocity (rad/s), default is 0. When all are 0, it is equivalent to giving a standstill command.
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};


class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),

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

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "Distance from the boundary to stop when returning to the field"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "Distance from the ball to stop moving towards it"),
            InputPort<double>("stop_angle", 0.1, "Angle of the ball to stop turning towards it"),
            InputPort<double>("vy_limit", 0.2, "Limit Y direction speed to prevent walking instability. Must be less than the robot's maximum speed 0.4 to take effect"),
            InputPort<double>("vx_limit", 0.6, "Limit X direction speed to prevent walking instability. Must be less than the robot's maximum speed 1.2 to take effect"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "x"),
            InputPort<double>("y", 0, "y"),
            InputPort<double>("theta", 0, "theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};
