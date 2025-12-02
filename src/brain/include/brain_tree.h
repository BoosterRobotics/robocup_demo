#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

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
        T value = T();
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
     * Initialize the entries in the blackboard.
     * Note: For newly added fields, set a default value here.
     */

    void initEntry();
};

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "Perform the chasing action if the distance exceeds this threshold"),
            InputPort<string>("decision_in", "", "Used to read the last decision"),
            InputPort<string>("position", "offense", "offense | defense, determines the direction to kick the ball"),
            OutputPort<string>("decision_out"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "Perform the chasing action if the distance exceeds this threshold"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "Consider the adjustment successful if the angle is smaller than this value"),
            InputPort<double>("adjust_y_tolerance", 0.1, "Consider the y-direction adjustment successful if the offset is smaller than this value"),
            InputPort<string>("decision_in", "", "Used to read the last decision"),
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
    double _cmdSequence[6][2];    // The sequence of actions for finding the ball, in which the robot looks towards these positions in order.
    rclcpp::Time _timeLastCmd;    // The time of the last command execution, used to ensure there is a time interval between commands.
    int _cmdIndex;                // The current step in the cmdSequence that is being executed.
    long _cmdIntervalMSec;        // The time interval (in milliseconds) between executing actions in the sequence.
    long _cmdRestartIntervalMSec; // If the time since the last execution exceeds this value, the sequence will restart from step 0.

    Brain *brain;
};

// The robot performs the action of finding the ball, which needs to be used in conjunction with CamFindBall.
class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "yaw limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; // 1.0 left -1.0 right
    Brain *brain;
};

// Chasing the ball: If the ball is behind the robot, it will move around to the back of the ball.
class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.4, "Maximum x velocity for chasing the ball"),
            InputPort<double>("vy_limit", 0.4, "Maximum y velocity for chasing the ball"),
            InputPort<double>("vtheta_limit", 0.1, "Maximum angular velocity for real-time direction adjustment while chasing the ball"),
            InputPort<double>("dist", 1.0, "The target distance behind the ball for chasing it"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     // circl_back, chase;
    double _dir = 1.0; // 1.0 circle back from left, -1.0  circle back from right
};

// After approaching the ball, adjust to the appropriate kicking angle for offense or defense.
class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.2, "If the angle to the ball exceeds this value, the robot will first turn to face the ball"),
            InputPort<double>("vx_limit", 0.1, "Limit for vx during adjustment, [-limit, limit]"),
            InputPort<double>("vy_limit", 0.1, "Limit for vy during adjustment, [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.4, "Limit for vtheta during adjustment, [-limit, limit]"),
            InputPort<double>("max_range", 1.5, "When the ball range exceeds this value, move slightly forward"),
            InputPort<double>("min_range", 1.0, "When the ball range is smaller than this value, move slightly backward"),
            InputPort<string>("position", "offense", "offense | defense, determines which direction to kick the ball"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("min_msec_kick", 500, "The minimum duration (in milliseconds) for executing a kick action"),
            InputPort<int>("msec_stand", 500, "The number of milliseconds after issuing a stop command"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.4, "vy limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime;
    int _msecKick = 1000;
};

// A full sweep of the field of view involves first tilting the head upwards in one direction,
// and then lowering it to sweep in another direction, completing one full circle.
class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.4, "The minimum pitch when looking upwards"),
            InputPort<double>("high_pitch", 0.2, "The minimum pitch when looking upwards"),
            InputPort<double>("left_yaw", 0.8, "The maximum yaw when looking to the left"),
            InputPort<double>("right_yaw", -0.8, " The minimum yaw when looking to the right"),
            InputPort<int>("msec_cycle", 4000, "How many milliseconds it takes to complete one full rotation"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// SelfLocate: Uses particle filtering to correct the current position, compensating for odometry drift.
class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            // enter_field: Used when entering the field, at this point, the robot is definitely in its own half, and the direction can be further narrowed based on the position of its own goal.
            // trust_direction: Used in normal conditions, where the odom information is generally accurate (the robot has not fallen over).
            // trust_position: Used after the robot has fallen. At this point, x and y are reliable, but the direction is not. (Note: if near the midfield line, the position should also be considered unreliable due to the symmetry of the field).
            // trust_nothing: An extreme case where neither x nor y is reliable, and the robot needs to identify its direction using landmarks.
            // face_forward: The direction facing the opponent's goal, primarily used for testing.
            InputPort<string>("mode", "enter_field", "must be one of [enter_field, trust_direction, trust_position, trust_nothing, face_forward]"),
        };
    };

private:
    Brain *brain;
};

// Move to a Pose in the Field coordinate system, including the final target orientation.
// It is recommended to use this together with CamScanField and SelfLocate for a more accurate final position.
class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Target x-coordinate in the Field coordinate system"),
            InputPort<double>("y", 0, "Target y-coordinate in the Field coordinate system"),
            InputPort<double>("theta", 0, "Final orientation of the target in the Field coordinate system"),
            InputPort<double>("long_range_threshold", 1.5, "When the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if the angle to the target point exceeds this threshold, turn towards the target point first"),
            InputPort<double>("vx_limit", 1.0, "x limit"),
            InputPort<double>("vy_limit", 0.5, "y limit"),
            InputPort<double>("vtheta_limit", 0.4, "theta limit"),
            InputPort<double>("x_tolerance", 0.2, "X tolerance"),
            InputPort<double>("y_tolerance", 0.2, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// ToWalkMode: 
class ToWalkMode : public SyncActionNode
{
public:
    ToWalkMode(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
          
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

/**
 * @brief Set the robot's velocity.
 *
 * @param x, y, theta double, the robot's velocity in the x and y directions (m/s) and angular velocity (rad/s) for counterclockwise rotation.
 * Default values are 0. If all values are 0, it is equivalent to issuing a command to make the robot stand still.
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

// ------------------------------- FOR DEMO -------------------------------

// This node is for demonstrating chasing the ball and is not used during actual gameplay.
// The difference from Chase is that Simple Chase just moves towards the ball without circling around to the ball's back,
// and therefore does not require field, localization, or video to run.
class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "The distance at which the robot will stop moving towards the ball"),
            InputPort<double>("stop_angle", 0.1, "The angle at which the robot will stop turning towards the ball"),
            InputPort<double>("vy_limit", 0.2, "Limit the velocity in the Y direction to prevent instability while walking"),
            InputPort<double>("vx_limit", 0.6, "Limit the velocity in the X direction to prevent instability while walking"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// ------------------------------- FOR DEBUG -------------------------------
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
