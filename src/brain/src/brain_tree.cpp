#include <cmath>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"

/**
 * Here, a macro definition is used to reduce the amount of code in RegisterBuilder.
 * The effect after expanding REGISTER_BUILDER(Test) is as follows:
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *      [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(ToWalkMode)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)

    // Action Nodes for debug
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<bool>("gamecontroller_isKickOffExecuted", true);

    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("treat_person_as_robot", false);
    setEntry<int>("control_state", 3);
    setEntry<bool>("B_pressed", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);
}

void BrainTree::tick()
{
    static int cnt = 0;
    cnt++;
    // print states
    if (cnt % 30 == 0)
        prtDebug(format(
            "GameState: %s\tIsKickOffSide: %d\nScore: %d\t JustScored: %d",
            getEntry<string>("gc_game_state").c_str(),
            getEntry<bool>("gc_is_kickoff_side"),
            brain->data->lastScore,
            getEntry<bool>("we_just_scored")));

    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else
    {
        const double pixTolerance = 10;

        double deltaX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin) - brain->config->camPixX / 2;
        double deltaY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin) - brain->config->camPixY * 2 / 3;

        if (std::fabs(deltaX) < pixTolerance && std::fabs(deltaY) < pixTolerance)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 0.8;
    double highPitch = 0.3;
    double leftYaw = 0.55;
    double rightYaw = -0.55;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    }
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {
        _cmdIndex = 0;
    }
    else
    {
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    double vxLimit, vyLimit, vthetaLimit, dist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    Pose2D target_f, target_r;
    if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
    {
        _state = "circle_back";

        target_f.x = brain->data->ball.posToField.x - dist;

        if (brain->data->robotPoseToField.y > brain->data->ball.posToField.y - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = brain->data->ball.posToField.y + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = brain->data->ball.posToField.x - dist;
        target_f.y = brain->data->ball.posToField.y;
    }

    target_r = brain->data->field2robot(target_f);

    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);
    vy = cap(vy, vyLimit, -vyLimit);

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("max_range", maxRange);
    getInput("min_range", minRange);
    string position;
    getInput("position", position);

    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = (position == "defense") ? atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    double s = 0.4;
    double r = 0.8;
    vx = -s * dir * sin(ballYaw);
    if (ballRange > maxRange)
        vx += 0.1;
    if (ballRange < maxRange)
        vx -= 0.1;
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = (position == "defense") ? atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus Kick::onStart()
{
    _startTime = brain->get_clock()->now();

    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;
    double yawOffset = brain->config->yawOffset;

    double adjustedYaw = brain->data->ball.yawToRobot - yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range;
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    double vx, vy;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    {
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);

    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;

    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    if (brain->msecsSince(_startTime) < _msecKick)
        return NodeStatus::RUNNING;

    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus RobotFindBall::onStart()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    _turnDir = 1.0;
}

NodeStatus SelfLocate::tick()
{
    string mode = getInput<string>("mode").value();
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    auto markers = brain->data->getMarkers();

    if (mode == "enter_field")
    {

        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;

        if (brain->config->playerStartPos == "left")
        {
            yMin = brain->config->fieldDimensions.width / 2;
            yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        }
        else if (brain->config->playerStartPos == "right")
        {
            yMin = -brain->config->fieldDimensions.width / 2 - 1.0;
            yMax = -brain->config->fieldDimensions.width / 2;
        }

        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = max(-brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 18;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 18;
    }

    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    auto res = brain->locator->locateRobot(markers, constraints);

    // if (brain->config->rerunLogEnable) {
    if (false)
    {
        brain->log->setTimeNow();
        brain->log->log("locator/time",
                        rerun::Scalar(res.msecs));
        brain->log->log("locator/residual",
                        rerun::Scalar(res.residual));
        brain->log->log("locator/result",
                        rerun::Scalar(res.code));
        brain->log->log("locator/constraints",
                        rerun::TextLog(
                            "xMin: " + to_string(xMin) + " " +
                            "xMax: " + to_string(xMax) + " " +
                            "yMin: " + to_string(yMin) + " " +
                            "yMax: " + to_string(yMax) + " " +
                            "thetaMin: " + to_string(thetaMin) + " " +
                            "thetaMax: " + to_string(thetaMax)));
    }
    prtDebug("locate result: res: " + to_string(res.code) + " time: " + to_string(res.msecs));

    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("locate success: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus ToWalkMode::tick()
{
    brain->client->walkMode();
    return NodeStatus::SUCCESS;
}

NodeStatus MoveToPoseOnField::tick()
{
    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);

    brain->client->moveToPoseOnField(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}
