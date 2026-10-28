#include <cmath>
#include <cstdlib>
#include <memory> 
#include "brain_tree.h"
#include "brain.h"
#include "utils/kick_geometry.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>
#include <limits>

using namespace std;
using namespace BT;

namespace
{
constexpr double kVisionKickBadPoseBackX = -0.10;
constexpr double kVisionKickBadPoseYawLimit = 1.75;
constexpr int kVisionKickBadPoseMaxTicks = 3;
constexpr double kVisionKickHeadScanPitch = 0.4;
constexpr double kVisionKickHeadDownPitch = 0.7;
constexpr double kVisionKickHeadCommandIntervalMs = 100.0;

double visualKickRealBallHoldMs(Brain *brain)
{
    double value = 1000.0;
    brain->get_parameter("strategy.auto_visual_kick.real_ball_hold_ms", value);
    return std::clamp(value, 150.0, 3000.0);
}

double visualKickCurrentBallMaxAgeMs(Brain *brain)
{
    double value = 650.0;
    brain->get_parameter("strategy.auto_visual_kick.current_real_ball_max_age_ms", value);
    return std::clamp(value, 50.0, 1000.0);
}

double visualKickNearMemoryRejectRange(Brain *brain)
{
    double value = 0.45;
    brain->get_parameter("strategy.auto_visual_kick.near_ball_memory_reject_range", value);
    return std::clamp(value, 0.3, 3.0);
}

double visualKickDecelerateMs(Brain *brain, bool startup)
{
    double value = startup ? 80.0 : 140.0;
    brain->get_parameter(startup
        ? "strategy.auto_visual_kick.startup_decelerate_ms"
        : "strategy.auto_visual_kick.exit_decelerate_ms", value);
    return std::clamp(value, 20.0, startup ? 500.0 : 700.0);
}

bool visualKickCurrentRealBallFresh(Brain *brain, double *ageMs = nullptr)
{
    const double age = brain->data->ball.timePoint.nanoseconds() > 0
        ? brain->msecsSince(brain->data->ball.timePoint)
        : std::numeric_limits<double>::infinity();
    if (ageMs) *ageMs = age;
    return age >= 0.0 && age <= visualKickCurrentBallMaxAgeMs(brain) &&
        std::isfinite(brain->data->ball.posToRobot.x) &&
        std::isfinite(brain->data->ball.posToRobot.y) &&
        std::isfinite(brain->data->ball.range) && brain->data->ball.range > 0.05;
}

bool visualKickNearRealBall(Brain *brain, double maxRange = 1.0)
{
    return visualKickCurrentRealBallFresh(brain) &&
        brain->data->ball.range <= maxRange;
}

bool visualKickGoalieTargetUsable(Brain *brain, bool goalieKick)
{
    return goalieKick && brain->data->goalieVisualKickSubState == "intercept_kick" &&
        brain->data->goalieVisualKickTargetValid &&
        std::isfinite(brain->data->goalieVisualKickTargetField.x) &&
        std::isfinite(brain->data->goalieVisualKickTargetField.y);
}

bool visualKickBallUsable(Brain *brain, bool goalieKick)
{
    if (brain->data->lose_ball) return false;
    if (visualKickCurrentRealBallFresh(brain)) return true;
    if (visualKickGoalieTargetUsable(brain, goalieKick)) return true;

    const double age = brain->data->ball.timePoint.nanoseconds() > 0
        ? brain->msecsSince(brain->data->ball.timePoint)
        : std::numeric_limits<double>::infinity();
    if (age < 0.0 || age > visualKickRealBallHoldMs(brain)) return false;

    const double predictedRange = std::hypot(
        brain->data->ball.reliablePredictPosToRobot.x,
        brain->data->ball.reliablePredictPosToRobot.y);
    const bool nearWithoutDetection = !brain->data->ballDetected &&
        ((std::isfinite(brain->data->ball.range) &&
          brain->data->ball.range <= visualKickNearMemoryRejectRange(brain)) ||
         (brain->data->reliableBallPredict && std::isfinite(predictedRange) &&
          predictedRange <= visualKickNearMemoryRejectRange(brain)));
    if (nearWithoutDetection) return false;

    bool allowPredicted = true;
    bool allowMemory = true;
    brain->get_parameter("strategy.auto_visual_kick.allow_predicted_kick_msg", allowPredicted);
    brain->get_parameter("strategy.auto_visual_kick.allow_memory_kick_msg", allowMemory);
    return (allowPredicted && brain->data->reliableBallPredict) ||
        (allowMemory && brain->tree->getEntry<bool>("ball_location_known"));
}

bool kickoffTacticalPhase(Brain *brain)
{
    return (brain->data->isKickingOff || brain->data->isOpponentKickingOff) &&
        brain->tree->getEntry<string>("gc_game_state") == "PLAY" &&
        brain->tree->getEntry<string>("player_role") == "striker";
}

int kickoffGroupRank(Brain *brain)
{
    if (brain->data->kickoffStrikerGroupLatched) {
        return std::clamp(brain->data->kickoffStrikerGroupRank, 0, 3);
    }
    return std::clamp(brain->data->tmMyStrikerCostRank, 0, 3);
}

bool kickoffBallInOpponentHalf(Brain *brain, bool ballKnown)
{
    if (!ballKnown) {
        // 短暂丢球时保持上一次半区判断，避免两组机器人因视觉丢球互换任务。
        if (brain->data->kickoffBallHalf != 0) {
            return brain->data->kickoffBallHalf > 0;
        }
        // 开球刚结束、球仍在中圈附近时，由开球方决定默认的进攻半区。
        return brain->tree->getEntry<bool>("gc_is_kickoff_side");
    }
    constexpr double HALF_HYSTERESIS = 0.5;
    if (brain->data->ball.posToField.x > HALF_HYSTERESIS) {
        brain->data->kickoffBallHalf = 1;
        return true;
    }
    if (brain->data->ball.posToField.x < -HALF_HYSTERESIS) {
        brain->data->kickoffBallHalf = -1;
        return false;
    }
    if (brain->data->kickoffBallHalf != 0) {
        return brain->data->kickoffBallHalf > 0;
    }
    return brain->tree->getEntry<bool>("gc_is_kickoff_side");
}
} // namespace

/**
 * 这里使用宏定义来缩减 RegisterBuilder 的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
 * factory.registerBuilder<Test>(  \
 * "Test",                    \
 * [this](const string& name, const NodeConfig& config) { return std::make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return std::make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes

    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    
    // 如果cpp里没实现这些类，注释掉，否则链接会报错。
    // 但在 header 里我给了它们空的 tick 实现，所以这里注册是安全的。
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SelfLocateLine)
    REGISTER_BUILDER(SelfLocateEnterField)
    REGISTER_BUILDER(SelfLocateLocal)
    REGISTER_BUILDER(SelfLocate1P)
    REGISTER_BUILDER(SelfLocate1M)
    REGISTER_BUILDER(SelfLocate2X)
    REGISTER_BUILDER(SelfLocate2T)
    REGISTER_BUILDER(SelfLocateLT)
    REGISTER_BUILDER(SelfLocatePT)
    REGISTER_BUILDER(SelfLocateBorder)

    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(RobocupWalk)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(MoveToFarSetPlaySearchArea)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(Analyze2)
    REGISTER_BUILDER(GoalieIdleHome)
    REGISTER_BUILDER(DecideCheckBehind)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RLVisionKick)
    REGISTER_BUILDER(Intercept2)

    REGISTER_BUILDER(RoleSwitchIfNeeded)

    REGISTER_BUILDER(Assist)
    REGISTER_BUILDER(KickoffStand)

    // Action Nodes for debug
    REGISTER_BUILDER(CrabWalk)
    REGISTER_BUILDER(AutoCalibrateVision)
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 构造完成后，初始化 blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    // 开球，对方开球时球动了，或到了时间限制时，置为 true，表示我们可以动了。
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<bool>("gc_play_stopped", false);
    setEntry<int>("gc_set_play", 0);
    setEntry<int>("gc_protocol_version", 0);
    setEntry<int>("gc_secondary_time", 0);
    setEntry<int>("gc_opponent_set_play_search_direction", 0);

    setEntry<bool>("need_check_behind", false);

    // 双机通讯相关
    setEntry<bool>("is_lead", true); // true 时代表自己为控球, false 时代表自己不控球, 而是给其它队员打配合.
    setEntry<string>("goalie_mode", "guard_idle");
    setEntry<bool>("goalie_block_kick_active", false);
    setEntry<bool>("penalty_kick_active", false);
    setEntry<bool>("penalty_kick_defending", false);
    setEntry<double>("penalty_kick_goalie_dist_to_goalline", 0.05);
    setEntry<double>("penalty_kick_goalie_dist_tolerance", 0.12);
    setEntry<double>("penalty_kick_goalie_theta_tolerance", 0.25);
    setEntry<double>("penalty_kick_goalie_vx_limit", 0.35);
    setEntry<double>("penalty_kick_goalie_vy_limit", 0.45);

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // 自动视觉校准相关
    setEntry<string>("calibrate_state", "pitch");
    setEntry<double>("calibrate_pitch_center", 0.0);
    setEntry<double>("calibrate_pitch_step", 1.0);
    setEntry<double>("calibrate_yaw_center", 0.0);
    setEntry<double>("calibrate_yaw_step", 1.0);
    setEntry<double>("calibrate_z_center", 0.0);
    setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick()
{
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

NodeStatus RobocupWalk::tick()
{
    // Intended to be wrapped by a RunOnce decorator at behavior-tree startup.
    brain->client->changeRobocupMode();
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw, ballX, ballY, deltaX, deltaY;
    const double pixToleranceX = brain->config->camPixX * 3 / 10.; // 球距离视野中心的像素差小于这个容差, 则认为在视野中心了.
    const double pixToleranceY = brain->config->camPixY * 3 / 10.;
    const double xCenter = brain->config->camPixX / 2;
    const double yCenter = brain->config->camPixY / 2; // 用下视野 2/3（不符）位置来追踪球, 以获得更多的场上信息.

    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/CamTrackBall", rerun::TextLog(msg));
    };
    auto logTrackingBox = [=](int color, string label) {
        brain->log->setTimeNow();
        vector<rerun::Vec2D> mins;
        vector<rerun::Vec2D> sizes;
        mins.push_back(rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
        sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
        brain->log->log(
            "image/track_ball",
            rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                .with_labels({label})
                .with_colors(color)
        );   

    };

    bool iSeeBall = brain->data->ballDetected;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
        return NodeStatus::SUCCESS;

    if (!iSeeBall)
    { // 没看见, 看向记忆中球的大致位置
        if (iKnowBallPos) {
            pitch = brain->data->ball.pitchToRobot;
            yaw = brain->data->ball.yawToRobot;
        } else if (tmBallPosReliable) {
            pitch = brain->data->tmBall.pitchToRobot;
            yaw = brain->data->tmBall.yawToRobot;
        } else {
            log("reached impossible condition");
        }
        logTrackingBox(0x000000FF, "ball not detected"); 
    }
    else { // 看见了, 视线跟踪球                
        ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
        ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter; 
        
        if (std::fabs(deltaX) < pixToleranceX && std::fabs(deltaY) < pixToleranceY)
        { // 认为已经在中心了
            auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f", ballX, ballY, deltaX, deltaY);
            logTrackingBox(0x00FF00FF, label);
            return NodeStatus::SUCCESS;
        }

        double smoother = 3.5; // 越大头部运动越平滑, 越小则越快, 小于 1.0 会超调震荡
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
        auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, pitch: %.1f, yaw: %.1f", ballX, ballY, deltaX, deltaY, pitch, yaw);
        logTrackingBox(0xFF0000FF, label);
    }

    brain->client->moveHead(pitch, yaw);

    // 新添加逻辑：检查计算后的 yaw 是否超过阈值，调整身体转动
    // const double maxHeadYaw = 0.8;  // 头部 yaw 最大值
    // const double minHeadYaw = -0.8; // 头部 yaw 最小值
    // const double bodyTurnSpeed = 1.0;  // 身体转动速度（弧度/秒）
    // const double vthetaLimit = 1.5;  // 旋转速度上限（与 Chase 等节点一致）
    // const double bodyTurnGain = 1.1;  // 比例增益，根据超出量放大 vtheta
    // static double lastVtheta = 0.0;  // 跟踪上一次的 vtheta，减少不必要调用

    // double vtheta = 0.0;  // 初始化 vtheta，避免未定义行为
    // if (yaw > maxHeadYaw) {
    //     // yaw 太大，身体向右转（vtheta 正值，根据超出量比例计算）
    //     vtheta = bodyTurnGain * (yaw - maxHeadYaw);
    //     vtheta = cap(vtheta, bodyTurnSpeed, 0.0);  // 限制正向速度
    //     log(format("Turning body right: yaw=%.2f > max=%.2f, vtheta=%.2f", yaw, maxHeadYaw, vtheta));
    // } else if (yaw < minHeadYaw) {
    //     // yaw 太小，身体向左转（vtheta 负值，根据超出量比例计算）
    //     vtheta = bodyTurnGain * (yaw - minHeadYaw);
    //     vtheta = cap(vtheta, 0.0, -bodyTurnSpeed);  // 限制负向速度
    //     log(format("Turning body left: yaw=%.2f < min=%.2f, vtheta=%.2f", yaw, minHeadYaw, vtheta));
    // } else {
    //     // yaw 在范围内，停止身体转动
    //     vtheta = 0.0;
    //     log(format("No body turn needed: yaw=%.2f within [%.2f, %.2f]", yaw, minHeadYaw, maxHeadYaw));
    // }

    // // 仅在 vtheta 变化时调用 setVelocity，减少对其他节点的干扰
    // if (vtheta != lastVtheta) {
    //     vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);  // 限制 vtheta 在全局范围内
    //     brain->client->setVelocity(0, 0, vtheta, false, false, false);  // 设置身体转动速度
    //     lastVtheta = vtheta;  // 更新 lastVtheta
    //     log(format("Set velocity: vtheta=%.2f", vtheta));
    // }
     
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 1.0;
    double highPitch = 0.2;
    double leftYaw = 1.1;
    double rightYaw = -1.1;

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
    } // 目前全部节点都是返回 Success 的, 返回 failure 会影响后面节点的执行.

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    } // 没到下条指令的执行时间
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {                   // 超过一定时间, 认为这是重新从头执行
        _cmdIndex = 0; // 注意这里不 return
    }
    else
    { // 达到时间, 执行下一个指令, 同样不 return
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

NodeStatus DecideCheckBehind::tick()
{
    double maxAngle = 0.0;
    double minAngle = 0.0;
    auto fd = brain->config->fieldDimensions;
    double corners[4][2] = {
        {fd.length / 2.0, fd.width / 2.0}, 
        {-fd.length / 2.0, fd.width / 2.0}, 
        {-fd.length / 2.0, -fd.width / 2.0}, 
        {fd.length / 2.0, -fd.width / 2.0}
    };
    for (int i = 0; i < 4; i++) {
        double angle_f = atan2(corners[i][1] - brain->data->robotPoseToField.y, corners[i][0] - brain->data->robotPoseToField.x);
        double angle = toPInPI(angle_f - brain->data->robotPoseToField.theta);
        if (angle > maxAngle) maxAngle = angle;
        if (angle < minAngle) minAngle = angle;
    }
    if (maxAngle < 1.8 && minAngle > -1.8) brain->tree->setEntry<bool>("need_check_behind", false);
    else brain->tree->setEntry<bool>("need_check_behind", true);
    return NodeStatus::SUCCESS;
}


NodeStatus Chase::tick()
{
    if (
        !brain->tree->getEntry<bool>("ball_location_known")
        || brain->isBallOut(3.0, 1.5)
    )
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle = false;
    double oaSafeDist = 2.0;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;

    double vx, vy, vtheta;
    Pose2D target_f, target_r;
    static string targetType = "direct";
    static double circleBackDir = 1.0;
    double dirThreshold = M_PI / 2;
    if (targetType == "direct") dirThreshold *= 1.2;

    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = -0.2 * circleBackDir;
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist / max(ballRange, 1e-5)));
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }

    target_r = brain->data->field2robot(target_f);
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
    } else {
        vx = min(vxLimit, ballRange);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid(fabs(vtheta), 1, 3);
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
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
    double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // 距离远时, 优先转向
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);     // 进一步限速
    vy = cap(vy, vyLimit, -vyLimit); // vy 进一步限速

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
        // if (fabs(brain->data->ball.yawToRobot) < stopAngle) vtheta = 0; // uncomment 这一行, 会站住. 现在站不太稳, 就让它一直动着吧.
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus MoveToFarSetPlaySearchArea::onStart()
{
    const int searchDirection =
        brain->tree->getEntry<int>("gc_opponent_set_play_search_direction");
    if (searchDirection == 0)
    {
        return NodeStatus::SUCCESS;
    }

    _direction = searchDirection > 0 ? 1 : -1;
    const double triggerX =
        brain->get_parameter("strategy.far_set_play_search.trigger_x").as_double();
    const double robotX = brain->data->robotPoseToField.x;
    const bool isAlreadyOnSearchSide =
        (_direction > 0 && robotX >= triggerX) ||
        (_direction < 0 && robotX <= -triggerX);
    if (isAlreadyOnSearchSide)
    {
        // 已经在球应出现的半场，直接使用后续的原地扫描找球流程。
        _direction = 0;
        return NodeStatus::SUCCESS;
    }

    const auto fd = brain->config->fieldDimensions;
    const double fieldTargetXRatio =
        brain->get_parameter("strategy.far_set_play_search.field_target_x_ratio").as_double();
    const double laneYRatio =
        brain->get_parameter("strategy.far_set_play_search.lane_y_ratio").as_double();

    const double targetXAbs = cap(
        fd.length * fieldTargetXRatio,
        fd.length / 2.0 - 1.0,
        0.3);
    _targetPose.x = _direction * targetXAbs;
    if (_direction > 0
        && brain->data->realGameSubState == "GOAL_KICK")
    {
        // 对方门球时，远距离搜索点本身也必须位于对方罚球区外。
        const double opponentPenaltyFrontX =
            fd.length / 2.0 - fd.penaltyAreaLength;
        _targetPose.x = min(
            _targetPose.x,
            opponentPenaltyFrontX - 0.5);
    }

    // 使用固定 player_id 分配搜索通道，避免动态排名变化导致机器人互换目标。
    static const double laneSlots[5] = {0.0, 1.0, -1.0, 0.5, -0.5};
    const int playerIndex = std::max(0, brain->config->playerId - 1) % 5;
    _targetPose.y = cap(
        laneSlots[playerIndex] * laneYRatio * fd.width,
        fd.width / 2.0 - 1.0,
        -fd.width / 2.0 + 1.0);

    if (fabs(_targetPose.y) > 0.1) {
        const double cornerY = _targetPose.y > 0.0 ? fd.width / 2.0 : -fd.width / 2.0;
        _targetPose.theta = atan2(
            cornerY - _targetPose.y,
            _direction * fd.length / 2.0 - _targetPose.x);
    } else {
        _targetPose.theta = _direction > 0 ? 0.0 : M_PI;
    }

    _maxMoveSecs =
        brain->get_parameter("strategy.far_set_play_search.max_move_secs").as_double();
    _vxLimit =
        brain->get_parameter("strategy.far_set_play_search.vx_limit").as_double();
    _vyLimit =
        brain->get_parameter("strategy.far_set_play_search.vy_limit").as_double();
    _startTime = brain->get_clock()->now();
    _arrived = false;
    _moveTimedOut = false;

    RCLCPP_INFO(
        brain->get_logger(),
        "far set-play search started: half=%s player=%d target=(%.2f, %.2f, %.2f)",
        _direction > 0 ? "OPPONENT" : "OWN",
        brain->config->playerId,
        _targetPose.x,
        _targetPose.y,
        _targetPose.theta);
    return NodeStatus::RUNNING;
}

NodeStatus MoveToFarSetPlaySearchArea::onRunning()
{
    const int searchDirection =
        brain->tree->getEntry<int>("gc_opponent_set_play_search_direction");
    const bool ballKnown =
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    const bool isExpectedSetPlay =
        _direction != 0 && searchDirection == _direction;
    if (!isExpectedSetPlay)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    const double triggerX =
        brain->get_parameter("strategy.far_set_play_search.trigger_x").as_double();
    const double robotX = brain->data->robotPoseToField.x;
    const bool isOnExpectedHalf =
        (_direction > 0 && robotX >= triggerX) ||
        (_direction < 0 && robotX <= -triggerX);
    if (isOnExpectedHalf && ballKnown)
    {
        // 队友给出的球坐标只能在机器人进入正确半场后结束远距离移动。
        // 这样既不会在错误半场提前停下，到达后也能立即交回正常找球/站位逻辑。
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (!brain->tree->getEntry<bool>("odom_calibrated"))
    {
        // 场地坐标不可靠时不做长距离盲走，保持扫描并等待定位恢复。
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    const double elapsedSecs =
        (brain->get_clock()->now() - _startTime).seconds();
    if (_maxMoveSecs > 0.0 && elapsedSecs >= _maxMoveSecs)
    {
        if (!_moveTimedOut)
        {
            RCLCPP_WARN(
                brain->get_logger(),
                "far set-play search timed out after %.1f seconds",
                elapsedSecs);
            _moveTimedOut = true;
        }
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    const auto robotPose = brain->data->robotPoseToField;
    const double dist =
        norm(_targetPose.x - robotPose.x, _targetPose.y - robotPose.y);
    if (dist < 0.5)
    {
        if (!_arrived)
        {
            RCLCPP_INFO(
                brain->get_logger(),
                "far set-play search target reached: half=%s player=%d",
                _direction > 0 ? "OPPONENT" : "OWN",
                brain->config->playerId);
            _arrived = true;
        }
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    brain->client->moveToPoseOnField3(
        _targetPose.x,
        _targetPose.y,
        _targetPose.theta,
        1.4,
        0.4,
        _vxLimit,
        _vyLimit,
        1.2,
        0.4,
        0.4,
        0.2,
        true);
    return NodeStatus::RUNNING;
}

void MoveToFarSetPlaySearchArea::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _arrived = false;
    _moveTimedOut = false;
    _direction = 0;
}

// 我方角球/门球开球时，非开球者（rank >= 1）的站位：
// 默认原地不动、面向球；只在挡住 "球 -> kickDir" 的开球通道时才做局部横向让位。
// 由 Assist::tick（v20 协议下我方开球时的实际生效路径）和 GoToFreekickPosition（兼容旧协议）共用。
static Pose2D calcSetPlayStandPose(Brain *brain, const Pose2D &robotPose, const Point &ballPos)
{
    double corridorHalfWidth, corridorForwardRange, sideMargin;
    brain->get_parameter("strategy.set_play_stand.corridor_half_width", corridorHalfWidth);
    brain->get_parameter("strategy.set_play_stand.corridor_forward_range", corridorForwardRange);
    brain->get_parameter("strategy.set_play_stand.side_margin", sideMargin);

    const double kickDir = brain->data->kickDir;
    Pose2D targetPose;
    targetPose.x = robotPose.x;
    targetPose.y = robotPose.y;

    const double dx = robotPose.x - ballPos.x;
    const double dy = robotPose.y - ballPos.y;
    const double forwardProj = dx * cos(kickDir) + dy * sin(kickDir); // 沿 kickDir 方向的投影
    const double perpDist = -dx * sin(kickDir) + dy * cos(kickDir);   // 到 kickDir 射线的带符号垂距

    const bool blocksCorridor =
        forwardProj >= 0.0
        && forwardProj <= corridorForwardRange
        && fabs(perpDist) < corridorHalfWidth;
    if (blocksCorridor) {
        const double pushDist = corridorHalfWidth + sideMargin;
        const double sign = perpDist >= 0.0 ? 1.0 : -1.0;
        targetPose.x = ballPos.x + forwardProj * cos(kickDir) - sign * pushDist * sin(kickDir);
        targetPose.y = ballPos.y + forwardProj * sin(kickDir) + sign * pushDist * cos(kickDir);
    }
    targetPose.theta = atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);
    return targetPose;
}

NodeStatus GoToFreekickPosition::onStart() {
    // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");

    string side;
    getInput("side", side);
    if (side != "attack" && side != "defense") return NodeStatus::SUCCESS;

    Pose2D targetPose = {0.0, 0.0, 0.0};
    const auto fd = brain->config->fieldDimensions;
    const bool ballLocationKnown =
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!ballLocationKnown) {
        brain->client->setVelocity(0.0, 0.0, 0.0);
        return NodeStatus::RUNNING;
    }
    const auto ballPos = brain->data->ball.posToField;
    const auto robotPose = brain->data->robotPoseToField;
    const double ownGoalX = -fd.length / 2.0;
    const double oppGoalX = fd.length / 2.0;

    const double kickDir = brain->data->kickDir;
    const double defenseDir = atan2(ballPos.y, ballPos.x + fd.length / 2.0);
    // 和 assist 对齐：freekick 按 tmMyCostRank 分工，不再按前锋序号。
    const int rank = brain->data->tmMyCostRank;
    const bool isCornerOrGoalKick =
        brain->data->realGameSubState == "CORNER_KICK"
        || brain->data->realGameSubState == "GOAL_KICK";

    if (side == "attack") {
        double attackDist = 0.7;
        getInput("attack_dist", attackDist);

        if (rank == 0) {
            targetPose.x = ballPos.x - attackDist * cos(kickDir);
            targetPose.y = ballPos.y - attackDist * sin(kickDir);
            targetPose.theta = kickDir;
        } else if (isCornerOrGoalKick) {
            // 角球/门球：只让 rank 0 上前开球，其余机器人原地待命，
            // 只在挡住 "球 -> kickDir" 开球通道时才做局部横向让位。
            targetPose = calcSetPlayStandPose(brain, robotPose, ballPos);
            log(format("rank %d stand at set play (corner/goal kick)", rank));
        } else if (rank == 1) {
            targetPose.x = ballPos.x - 2.0 * cos(defenseDir);
            targetPose.y = ballPos.y - 2.0 * sin(defenseDir);
            targetPose.theta = defenseDir;
        } else if (rank == 2) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = fd.goalAreaWidth / 2.0;
        } else { // rank >= 3
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = - fd.goalAreaWidth / 2.0;
            log(format("freekick attack fallback, rank=%d", rank));
        }
    } else if (side == "defense") {
        if (rank == 0) {
            targetPose.x = ballPos.x - 3.0 * cos(defenseDir);
            targetPose.y = ballPos.y - 2.5 * sin(defenseDir);
            targetPose.theta = defenseDir;
           } else if (rank == 1) {
            targetPose.x = ballPos.x - 3.5 * cos(defenseDir);
            targetPose.y = ballPos.y - 4.0 * sin(defenseDir);
            targetPose.theta = defenseDir;
            } else if (rank == 2) {
                targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
                targetPose.y = fd.goalAreaWidth / 2.0;
            } else { // rank >= 3
                targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
                targetPose.y = - fd.goalAreaWidth / 2.0;
                log(format("freekick defense fallback, rank=%d", rank));
        }
    }

    // 场地约束（对齐 assist 思路）：保证目标点始终在可行区域。
    targetPose.x = cap(targetPose.x, oppGoalX - 0.3, ownGoalX + 0.3);
    targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.6, -fd.width / 2.0 + 0.6);

    if (side == "defense"
        && !brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side")) {
        const double legalMargin = 0.5;
        if (brain->data->realGameSubState == "GOAL_KICK") {
            // 对方门球：我方所有机器人必须在对方罚球区外。
            const double opponentPenaltyFrontX =
                oppGoalX - fd.penaltyAreaLength;
            const bool overlapsPenaltyAreaWidth =
                fabs(targetPose.y) < fd.penaltyAreaWidth / 2.0 + legalMargin;
            if (overlapsPenaltyAreaWidth) {
                targetPose.x = min(
                    targetPose.x,
                    opponentPenaltyFrontX - legalMargin);
            }
        } else if (brain->data->realGameSubState == "CORNER_KICK") {
            // 对方角球：在通用任意球避让半径之外再预留 0.5 m。
            const double minimumDistance = fd.circleRadius + legalMargin;
            double dx = targetPose.x - ballPos.x;
            double dy = targetPose.y - ballPos.y;
            double targetBallDistance = norm(dx, dy);
            if (targetBallDistance < minimumDistance) {
                if (targetBallDistance < 1e-6) {
                    dx = 1.0;
                    dy = 0.0;
                    targetBallDistance = 1.0;
                }
                targetPose.x =
                    ballPos.x + dx / targetBallDistance * minimumDistance;
                targetPose.y =
                    ballPos.y + dy / targetBallDistance * minimumDistance;
            }
        }

        targetPose.x = cap(targetPose.x, oppGoalX - 0.3, ownGoalX + 0.3);
        targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.6, -fd.width / 2.0 + 0.6);
    }

    // 对 rank>=2 的固定后场位，朝向球更稳定。
    if (rank >= 2) {
        targetPose.theta = atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);
    }

    const double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    const double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

    if ( // 认为到达了目标位置
        dist < 0.3
        && fabs(deltaDir) < 0.15
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    static bool adjustFreekickCached = false;
    static bool enableFreekickAvoid = false;
    if (!adjustFreekickCached) {
        enableFreekickAvoid = brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool();
        adjustFreekickCached = true;
    }

    if (!enableFreekickAvoid || dist < 1.5 || _isInFinalAdjust) {
        _isInFinalAdjust = true; // 进入最后的微调阶段
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // 距离远时, 优先转向
        vx *= linearFactor;
        vy *= linearFactor;

        // 防止撞到球
        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (
            pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.7
            && brain->data->ball.range < 1.2
        ) {
            vx = min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1: vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -0.4);         // 进一步限速
        vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.4;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

    return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
    // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}

NodeStatus GoToGoalBlockingPosition::tick() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
    };
    log("GoToGoalBlockingPosition ticked");

    // if (!brain->tree->getEntry<bool>("ball_location_known")) {
    //      brain->client->setVelocity(0, 0, 0);
    //      return NodeStatus::SUCCESS;
    // }
    // brain->log->setTimeNow();
    // brain->log->log("tree/GoToGoalBlockingPosition", rerun::TextLog("GoToGoalBlockingPosition tick"));
     
    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    string curRole = brain->tree->getEntry<string>("player_role");

    Pose2D targetPose;
    targetPose.x = curRole == "striker" ? (std::max(- fd.length / 2.0 + distToGoalline, ballPos.x - 1.5))
            : (- fd.length / 2.0 + distToGoalline);
    if (ballPos.x + fd.length / 2.0 < distToGoalline) {
        targetPose.y = curRole == "striker" ? (ballPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
            : (std::abs(ballPos.x + fd.length / 2.0) < 1e-4
                ? (ballPos.y >= 0.0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
                : std::clamp(
                    ballPos.y / (ballPos.x + fd.length / 2.0) * distToGoalline,
                    -fd.goalWidth / 2.0,
                    fd.goalWidth / 2.0));
    } else {
        targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
        targetPose.y = curRole == "striker" ? (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth/ 2.0, -fd.penaltyAreaWidth / 2.0));
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // 认为到达了目标位置
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);     // 进一步限速
    vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速
     

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Assist", rerun::TextLog(msg));
    };
    log("ticked");

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;
    Pose2D targetPose = {0.0, 0.0, 0.0};

    const double ownGoalX = -fd.length / 2.0;
    const double oppGoalX = fd.length / 2.0;
    const int rank = brain->data->tmMyCostRank;

    // 重新计算“仅前锋”的到球成本排名，避免守门员的 cost 占掉一个接应名额。
    // rank 0 是当前主攻手，rank 1/2 是前场接应，rank 3 负责中场保护。
    int strikerCostRank = 0;
    const int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; ++i) {
        if (i == selfIdx) {
            continue;
        }
        const auto &status = brain->data->tmStatus[i];
        if (status.isAlive
            && status.role == "striker"
            && status.cost < brain->data->tmMyCost) {
            ++strikerCostRank;
        }
    }
    const bool aggressiveAssist =
        brain->data->liveCount >= 3 && ballPos.x > 0.0;

    // 我方角球/门球开球期间：只让 rank 0 上前开球（走 StrikerDecide 的 chase/kick 分支，
    // 不会进入这里），其余机器人原地待命、面向球，仅在挡住开球通道时才让位。
    const bool isCornerOrGoalKickAttack =
        brain->data->isFreekickKickingOff
        && (brain->data->realGameSubState == "CORNER_KICK"
            || brain->data->realGameSubState == "GOAL_KICK");

    auto calcBlockLineY = [&](double targetX) {
        const double denom = ballPos.x - ownGoalX;
        if (fabs(denom) < 1e-6) {
            return 0.0;
        }
        return ballPos.y * (targetX - ownGoalX) / denom;
    };

    if (isCornerOrGoalKickAttack) {
        targetPose = calcSetPlayStandPose(brain, robotPose, ballPos);
        log(format("corner/goal kick stand, rank=%d", rank));
    } else if (aggressiveAssist) {
        if (strikerCostRank <= 2) {
            // 主攻手以外的前两名进入对方半场，形成前后错开的两条接应线。
            const bool firstSupport = strikerCostRank <= 1;
            targetPose.x = max(
                firstSupport ? 0.8 : 0.5,
                ballPos.x - (firstSupport ? 2.0 : 3.5));
            targetPose.y = calcBlockLineY(targetPose.x)
                + (firstSupport ? 1.5 : -1.5);
            log(format(
                "aggressive support: strikerCostRank=%d target=(%.2f, %.2f)",
                strikerCostRank,
                targetPose.x,
                targetPose.y));
        } else {
            // 最后一名前锋留在中线附近，防止所有场上机器人同时压到前场。
            targetPose.x = min(-0.5, ballPos.x - 4.0);
            targetPose.y = calcBlockLineY(targetPose.x);
            log(format(
                "midfield cover: strikerCostRank=%d target=(%.2f, %.2f)",
                strikerCostRank,
                targetPose.x,
                targetPose.y));
        }
    // rank 0 在 Assist 状态下是异常情况（通常是只剩我自己或切换瞬间），这里退化为 rank 1 逻辑.
    } else if (brain->data->tmMyCostRank == 0) {
        // Bug 修复：当没有队友在线（或我是 cost 最低的）时，也需要计算防守位置
        // 这种情况发生在：队友被罚下，但我仍然处于 Assist 状态
        targetPose.x = ballPos.x - 2.0;
        targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); // 不要太接近底线
        targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
        log("tmMyCostRank == 0, using default assist position");
    } else if (brain->data->tmMyCostRank == 1) {
        targetPose.x = ballPos.x - 2.0;
        targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); // 不要太接近底线
        targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
    } else if (brain->data->tmMyCostRank == 2) {  //todo 之后的assist时候的tmMyCostRank不考虑守门员的cost
        // targetPose.x = ballPos.x - 2.0;
        // targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); // 不要太接近底线
        // targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
        targetPose.x = -fd.length / 2.0 + fd.penaltyAreaLength + 1;
        if (targetPose.x > ballPos.x) targetPose.x = ballPos.x - 1.0;
        targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
    } else if (brain->data->tmMyCostRank == 3) {
        targetPose.x = -fd.length / 2.0 + fd.penaltyAreaLength;
        if (targetPose.x > ballPos.x) targetPose.x = ballPos.x - 0.5;
        targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); // 可以挡住球的位置
    } else {
        // tmMyCostRank >= 4 的情况，使用默认防守位置
        targetPose.x = -fd.length / 2.0 + fd.penaltyDist;
        if (targetPose.x > ballPos.x) targetPose.x = ballPos.x - 0.5;
        targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0);
        log(format("tmMyCostRank = %d, using fallback position", brain->data->tmMyCostRank));
    }

    if (isCornerOrGoalKickAttack) {
        // 角球/门球时 rank>=1 允许贴近对方底线原地待命，不套用下方防守拦截线的禁区裁剪。
        targetPose.x = cap(targetPose.x, oppGoalX - 0.3, ownGoalX + 0.3);
        targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.3, -fd.width / 2.0 + 0.3);
    } else {
        // 场地约束：防止辅助位冲出边界或压到对方禁区深处.
        targetPose.x = cap(targetPose.x, oppGoalX - fd.penaltyAreaLength - 0.2, ownGoalX + distToGoalline);
        targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.7, -fd.width / 2.0 + 0.7);
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // 认为到达了目标位置
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx, vy, vtheta;
    auto targetPose_r = brain->data->field2robot(targetPose);
    double targetDir = atan2(targetPose_r.y, targetPose_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = brain->data->ball.yawToRobot;
    } else {
        vx = targetPose_r.x;
        vy = targetPose_r.y;
        vtheta = brain->data->ball.yawToRobot * 2.0; // 后面的乘数越大, 转身越快
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -0.25);     // 进一步限速, 不允许后退速度过快.
    vy = cap(vy, vyLimit, -vyLimit);     // 进一步限速
     

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus KickoffStand::tick()
{
    if (!kickoffTacticalPhase(brain)) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double distTolerance, thetaTolerance, vxLimit, vyLimit, holdVthetaLimit;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("hold_vtheta_limit", holdVthetaLimit);

    const auto fd = brain->config->fieldDimensions;
    const int rank = kickoffGroupRank(brain);
    const bool ownKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    const bool ballLocationKnown =
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    const auto ballPos = ballLocationKnown
        ? brain->data->ball.posToField
        : Point{0.0, 0.0, 0.0};
    const double ownGoalX = -fd.length / 2.0;
    const double oppGoalX = fd.length / 2.0;
    Pose2D targetPose{0.0, 0.0, 0.0};

    if (rank >= 2) {
        // 己方半区分组：在己方大禁区前沿左右分开待命。
        targetPose.x = ownGoalX + fd.penaltyAreaLength + 0.4;
        targetPose.y = rank == 2 ? 1.5 : -1.5;
    } else if (rank == 0) {
        // 对方半区分组的第一名：中圈向对方半场靠约 2 m。
        targetPose.x = fd.circleRadius + 2.0;
        targetPose.y = 0.0;
    } else {
        // 第二名：我方开球时守在对方小禁区前；对方开球时退到对方大禁区前。
        const double areaLength = ownKickoff ? fd.goalAreaLength : fd.penaltyAreaLength;
        targetPose.x = oppGoalX - areaLength - 0.5;
        targetPose.y = 0.0;
    }

    targetPose.x = cap(targetPose.x, oppGoalX - 0.5, ownGoalX + 0.5);
    targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.8, -fd.width / 2.0 + 0.8);
    targetPose.theta = atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);

    const auto robotPose = brain->data->robotPoseToField;
    const double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if (dist <= distTolerance) {
        // 到达待命点后继续原地转身。上游的定位和 CamFindAndTrackBall
        // 仍会在本帧先执行，因此旋转同时用于定位和扩大找球视野。
        holdVthetaLimit = std::clamp(
            holdVthetaLimit,
            -brain->config->vthetaLimit,
            brain->config->vthetaLimit);
        if (fabs(holdVthetaLimit) < 1e-3) {
            holdVthetaLimit = std::min(0.6, brain->config->vthetaLimit);
        }
        const double spinDirection = brain->config->playerId % 2 == 0 ? -1.0 : 1.0;
        brain->client->setVelocity(
            0.0, 0.0, spinDirection * holdVthetaLimit,
            false, false, false);
        return NodeStatus::SUCCESS;
    }

    brain->client->moveToPoseOnField2(
        targetPose.x, targetPose.y, targetPose.theta,
        1.0, 0.4, vxLimit, vyLimit, 1.2,
        distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, true);
    return NodeStatus::SUCCESS;
}


NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range;
    double stFar, stNear, vthetaFactor, nearThreshold, noTurnThreshold, turnFirstThreshold;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    getInput("tangential_speed_far", stFar);
    getInput("tangential_speed_near", stNear);
    getInput("vtheta_factor", vthetaFactor);
    getInput("near_threshold", nearThreshold);
    getInput("no_turn_threshold", noTurnThreshold);
    turnFirstThreshold = turnThreshold;
    getInput("turn_first_threshold", turnFirstThreshold);
    string position;
    getInput("position", position);

    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = (position == "defense") ?
        atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2)
        : brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    double st = stFar;
    if (fabs(deltaDir) * ballRange < nearThreshold) {
        st = stNear;
    }

    double thetaRobotField = brain->data->robotPoseToField.theta;
    double tangentialDirRobot = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - thetaRobotField;
    double radialDirRobot = dir_rb_f - thetaRobotField;
    double sr = cap(ballRange - range, 0.5, 0.0);

    vx = st * cos(tangentialDirRobot) + sr * cos(radialDirRobot);
    vy = st * sin(tangentialDirRobot) + sr * sin(radialDirRobot);
    vtheta = ballYaw * vthetaFactor;

    if (fabs(ballYaw) < noTurnThreshold) {
        vtheta = 0.0;
    }
    if (fabs(ballYaw) > turnFirstThreshold && fabs(deltaDir) < M_PI / 4) {
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.0);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, true, false);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1; //防止震荡

    if (!brain->tree->getEntry<bool>("ball_location_known") &&
        !brain->tree->getEntry<bool>("tm_ball_pos_reliable")) {
        return NodeStatus::SUCCESS;
    }

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    const double goalLineX = fd.length / 2.0;
    const double shootDir = kick_geometry::directionToOpponentGoal(
        bPos.x, bPos.y, goalLineX);

    string nextKickType = "shoot";
    double nextKickDir = shootDir;
    auto color = 0x00FF00FF;

    if (
        brain->data->realGameSubState == "CORNER_KICK"
        && brain->data->isFreekickKickingOff
    ) {
        // 我方角球：往中圈方向开, 而不是直接冲向球门。
        nextKickType = "cross";
        color = 0xFF00FFFF;
        nextKickDir = kick_geometry::directionToTarget(
            bPos.x, bPos.y, 0.0, 0.0, shootDir);
    }
    else if (kick_geometry::minorArcWidth(thetal, thetar) < crossThreshold &&
             bPos.x > fd.circleRadius) {
        nextKickType = "cross";
        color = 0xFF00FFFF;
        nextKickDir = kick_geometry::directionToTarget(
            bPos.x, bPos.y, goalLineX - fd.penaltyDist / 2.0, 0.0, shootDir);
    }
    else if (
        brain->data->isFreekickKickingOff 
        && brain->isPrimaryStriker() 
        && !brain->data->isDirectShoot
        && (bPos.x < fd.length/2.0 - fd.goalAreaWidth && bPos.x > -fd.length/2.0 + fd.penaltyAreaWidth)
    ) {
        // 在开球时, 决策要不要传中
        nextKickType = "cross";
        color = 0xFF00FFFF;
        // 中路任意球向球门推进，边线附近向场内横传。
        nextKickDir = kick_geometry::directionForIndirectSetPlay(
            bPos.x, bPos.y, goalLineX, fd.width);
    }
    else if (brain->isDefensing()) {
        nextKickType = "block";
        color = 0xFFFF00FF;
        nextKickDir = kick_geometry::directionToTarget(
            -fd.length / 2.0, 0.0, bPos.x, bPos.y, shootDir);
    }

    brain->data->kickType = nextKickType;
    brain->data->kickDir = kick_geometry::normalizeAngle(nextKickDir);

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };
    // 读取和处理参数
    bool enableBypass, enableShoot, enableDirectionalKick, kickAOUseShoot, enablePowerShoot, usePowerShootForKickoff;
    brain->get_parameter("strategy.power_shoot.enable", enablePowerShoot);
    brain->get_parameter("strategy.power_shoot.use_for_kickoff", usePowerShootForKickoff);
    brain->get_parameter("strategy.enable_shoot", enableShoot);
    brain->get_parameter("strategy.enable_bypass", enableBypass);
    brain->get_parameter("strategy.enable_directional_kick", enableDirectionalKick);
    brain->get_parameter("obstacle_avoidance.kick_ao_use_shoot", kickAOUseShoot);
    double KICK_RANGE = 1.0;
    brain->get_parameter("strategy.kick_range", KICK_RANGE);
    double KICK_THETA_RANGE = 1.5;
    brain->get_parameter("strategy.kick_theta_range", KICK_THETA_RANGE);

    const double bypassThreshold = 0.5;
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    bool enableAutoVisualKick = false;
    brain->get_parameter("strategy.enable_auto_visual_kick", enableAutoVisualKick);
    double autoVisualKickEnableDistMin = 0.2;
    double autoVisualKickEnableDistMax = 4.0;
    double autoVisualKickEnableAngle = 0.8;
    double autoVisualKickMaxDirectionError = 0.35;
    brain->get_parameter("strategy.auto_visual_kick_enable_dist_min", autoVisualKickEnableDistMin);
    brain->get_parameter("strategy.auto_visual_kick_enable_dist_max", autoVisualKickEnableDistMax);
    brain->get_parameter("strategy.auto_visual_kick_enable_angle", autoVisualKickEnableAngle);
    brain->get_parameter("strategy.auto_visual_kick_max_direction_error", autoVisualKickMaxDirectionError);
    autoVisualKickMaxDirectionError = std::clamp(
        autoVisualKickMaxDirectionError, 0.05, M_PI);

    const bool kickoffPhase = kickoffTacticalPhase(brain);
    const bool kickoffBallKnown =
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    const bool ballInOpponentHalf = kickoffBallInOpponentHalf(brain, kickoffBallKnown);
    const int kickoffRank = kickoffGroupRank(brain);
    const bool assignedOpponentHalf = kickoffRank >= 0 && kickoffRank < 2;
    const bool kickoffAttack = kickoffPhase &&
        (assignedOpponentHalf == ballInOpponentHalf);
    const bool kickoffHold = kickoffPhase && !kickoffAttack;

    // No strategy calculation below may consume a placeholder ball. During a
    // kickoff we can still choose the latched stand group without a live ball.
    if (!kickoffBallKnown) {
        const string newDecision = kickoffHold ? "kickoff_hold" : "find";
        const auto color = kickoffHold ? 0x00AAFFFF : 0xFFFFFFFF;
        brain->data->tmImInVisualKick = false;
        hasLastDeltaDir = false;
        setOutput("decision_out", newDecision);
        if (kickoffHold) {
            log(format(
                "kickoff stand without current ball: groupRank=%d ballHalf=%s",
                kickoffRank, ballInOpponentHalf ? "opponent" : "own"));
        }
        brain->log->logToScreen(
            "tree/Decide",
            format("Decision: %s (ball position unavailable)", newDecision.c_str()),
            color);
        brain->log->logToScreen(
            "tree/value_threat", "Threat Level: unavailable, Kick Value: unavailable",
            0xFFFFFFFF, 60);
        return NodeStatus::SUCCESS;
    }

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; // 机器人到球, field 坐标系中的方向
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;

    const double goalpostMargin = 0.3; // 计算角度时为门柱让出的距离
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    bool angleGoodForShoot = brain->isAngleGood(goalpostMargin, "shoot");
    bool angleGoodForDirectionalKick = brain->isAngleGoodForDirectionalKick(goalpostMargin);

    const double SHOOT_Y_RANGE = 0.3;
    const double SHOOT_X_MAX = 0.6;
    const double SHOOT_X_MIN = 0.2;
    bool shootPossible = angleGoodForShoot && fabs(ballY) < SHOOT_Y_RANGE && ballX < SHOOT_X_MAX && ballX > SHOOT_X_MIN;
    bool directionalKickPossible = angleGoodForDirectionalKick && fabs(ballY) < SHOOT_Y_RANGE && ballX < SHOOT_X_MAX && ballX > SHOOT_X_MIN;

    double powerShootXMin, powerShootXMax, powerShootYMin, powerShootYMax;
    brain->get_parameter("strategy.power_shoot.xmin", powerShootXMin);
    brain->get_parameter("strategy.power_shoot.xmax", powerShootXMax);
    brain->get_parameter("strategy.power_shoot.ymin", powerShootYMin);
    brain->get_parameter("strategy.power_shoot.ymax", powerShootYMax);
    bool powerShootPossible =
        enablePowerShoot
        && angleGoodForKick
        && ballX > powerShootXMin
        && ballX < powerShootXMax
        && ballY > powerShootYMin
        && ballY < powerShootYMax
        && (!brain->data->isFreekickKickingOff || usePowerShootForKickoff);


    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing // 是否在踢球时避免碰撞
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    // 判断是否穿过了 KickDir
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    const bool visualKickDirectionReady =
        fabs(deltaDir) <= autoVisualKickMaxDirectionError;
    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, deltaDir: %.2f, visualDirReady: %d, angleGoodForKick: %d, angleGoodForShoot: %d, shootPossible: %d, powerShootPossible: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, deltaDir,
        visualKickDirectionReady, angleGoodForKick, angleGoodForShoot,
        shootPossible, powerShootPossible));
    auto now = brain->get_clock()->now();
    const double dt = hasLastDeltaDir
        ? brain->msecsSince(timeLastTick)
        : std::numeric_limits<double>::infinity();
    const bool previousSampleIsRecent = hasLastDeltaDir &&
        dt >= 0.0 && dt < 100.0;
    bool reachedKickDir =
        previousSampleIsRecent &&
        deltaDir * lastDeltaDir <= 0 &&
        fabs(deltaDir) < M_PI / 6;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;
    hasLastDeltaDir = true;

    double kickValue = brain->kickValue(dir_rb_f);
    double threatLevel = brain->threatLevel();
    log(format("kickValue: %.1f, threatLevel: %.1f", kickValue, threatLevel));
     

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    const bool nearBallVisualKick =
        enableAutoVisualKick &&
        !brain->tree->getEntry<bool>("ball_out") &&
        visualKickDirectionReady &&
        visualKickNearRealBall(brain);

    if (nearBallVisualKick)
    {
        // 近球是本机的即时优先级，不再受 lead、cost rank 或开球待命分组限制。
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
        color = 0xFF00FFFF;
        log(format("near-ball visual kick: range=%.2f", ballRange));
    }
    else if (kickoffHold)
    {
        // 非球所在半区的两名前锋移动到预设待命点，仍由上层相机节点跟踪球。
        newDecision = "kickoff_hold";
        color = 0x00AAFFFF;
        log(format(
            "kickoff stand: groupRank=%d ballHalf=%s",
            kickoffRank, ballInOpponentHalf ? "opponent" : "own"));
    }
    else if (
        enableAutoVisualKick &&
        brain->data->tmImLead &&
        brain->data->tmMyCostRank == 0 &&
        !brain->tree->getEntry<bool>("ball_out") &&
        !brain->data->lose_ball &&
        !powerShootPossible &&
        brain->data->tmMyCost < 7.0 &&
        ballRange < autoVisualKickEnableDistMax &&
        ballRange > autoVisualKickEnableDistMin &&
        fabs(ballYaw) < autoVisualKickEnableAngle * 1.3 &&
        visualKickDirectionReady &&
        ball.posToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
        fabs(ball.posToField.y) < 5.0 &&
        brain->data->robotPoseToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
        fabs(brain->data->robotPoseToField.y) < 5.0
    ) {
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
        color = 0xFF00FFFF;
    } else if (!brain->data->tmImLead && !kickoffAttack) {
        newDecision = "assist";
        color = 0x00FFFFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && !avoidKick
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < KICK_THETA_RANGE
        && ball.range < KICK_RANGE
    )
    {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else if (powerShootPossible) newDecision = "power_kick";
        else { // kickType == kick
            double threatThreshold;
            brain->get_parameter("strategy.shoot.threat_threshold", threatThreshold);
            if (threatLevel < threatThreshold) newDecision = "safe_shoot";
            else newDecision = "kick";
        }        
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; // 只要进一次 kick, 就不算是 kickoff 阶段了.
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    if (newDecision != "auto_visual_kick") {
        brain->data->tmImInVisualKick = false;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d angleGoodForShoot: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, angleGoodForShoot, brain->data->tmImLead
        ),
        color
    );

    color = 0xFFFFFFFF;
    if (threatLevel >= 2.0) color = 0xFF0000FF;
    else if (threatLevel >= 1.0) color = 0xFFCC00FF;
    brain->log->logToScreen("tree/value_threat", format("Threat Level: %.1f, Kick Value: %.1f", threatLevel, kickValue), color, 60);
    return NodeStatus::SUCCESS;
}

namespace
{
struct GoalieBallSource
{
    bool usable = false;
    bool real = false;
    bool predicted = false;
    Point field{0.0, 0.0, 0.0};
    Pose2D robot{};
    double range = std::numeric_limits<double>::infinity();
};

struct GoaliePredictTarget
{
    bool predictionReliable = false;
    bool targetValid = false;
    bool interceptEntryReady = false;
    Point targetField{0.0, 0.0, 0.0};
    double targetRobotDist = std::numeric_limits<double>::infinity();
    double currentBallRobotDist = std::numeric_limits<double>::infinity();
    double targetCloserBy = -std::numeric_limits<double>::infinity();
    double ballToTargetDist = std::numeric_limits<double>::infinity();
};

GoalieBallSource getGoalieBallSource(Brain *brain)
{
    GoalieBallSource source;
    source.real = brain->data->ballDetected;
    source.predicted = !source.real && brain->data->reliableBallPredict;
    if (source.real) {
        source.field = brain->data->ball.posToField;
        source.robot = {brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, 0.0};
        source.range = brain->data->ball.range;
    } else if (source.predicted) {
        source.field = brain->data->ball.reliablePredictPosToField;
        source.robot = {brain->data->ball.reliablePredictPosToRobot.x,
                        brain->data->ball.reliablePredictPosToRobot.y, 0.0};
        source.range = std::hypot(source.robot.x, source.robot.y);
    }
    const auto &fd = brain->config->fieldDimensions;
    source.usable = (source.real || source.predicted) &&
        std::isfinite(source.field.x) && std::isfinite(source.field.y) &&
        std::isfinite(source.range) && source.range >= 0.0 &&
        source.field.x >= -fd.length / 2.0 - 0.7 &&
        source.field.x <= fd.length / 2.0 + 0.7 &&
        std::abs(source.field.y) <= fd.width / 2.0 + 0.7;
    return source;
}

GoaliePredictTarget getGoaliePredictTarget(Brain *brain, const GoalieBallSource &source)
{
    GoaliePredictTarget result;
    const auto &trajectory = brain->data->predictedBallPos;
    if (trajectory.empty() || brain->data->ballPosPredictTime.nanoseconds() <= 0) return result;

    const double stepMs = std::max(1.0,
        brain->get_parameter("ball_predictor.step_interval").as_double());
    const double ageMs = brain->msecsSince(brain->data->ballPosPredictTime);
    const double horizonMs = stepMs * static_cast<double>(trajectory.size() - 1);
    if (!std::isfinite(ageMs) || ageMs < 0.0 || ageMs > std::min(2000.0, horizonMs)) return result;

    const int startIdx = std::clamp(static_cast<int>(ageMs / stepMs), 0,
                                    static_cast<int>(trajectory.size()) - 1);
    double bestDist = std::numeric_limits<double>::infinity();
    for (int i = startIdx; i < static_cast<int>(trajectory.size()); ++i) {
        const auto &p = trajectory[i];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1])) continue;
        const auto robot = brain->data->field2robot({p[0], p[1], 0.0});
        const double dist = std::hypot(robot.x, robot.y);
        if (dist < bestDist) {
            bestDist = dist;
            result.targetField = {p[0], p[1], 0.0};
            result.targetValid = true;
        }
    }
    result.predictionReliable = result.targetValid;
    if (!result.targetValid) return result;

    result.targetRobotDist = bestDist;
    result.currentBallRobotDist = source.usable ? source.range : std::numeric_limits<double>::infinity();
    result.targetCloserBy = result.currentBallRobotDist - result.targetRobotDist;
    result.ballToTargetDist = source.usable
        ? std::hypot(source.field.x - result.targetField.x, source.field.y - result.targetField.y)
        : std::numeric_limits<double>::infinity();
    const bool moving = brain->data->ballEstIsRolling || brain->data->ballEstSpeed > 0.15;
    result.interceptEntryReady = source.usable && moving &&
        result.targetRobotDist < 1.5 && result.targetCloserBy >= 1.0;
    return result;
}
} // namespace

NodeStatus Analyze2::tick()
{
    const string currentMode = brain->tree->getEntry<string>("goalie_mode");
    const bool active = brain->tree->getEntry<bool>("goalie_block_kick_active");
    const bool visualKickMode = currentMode == "block_kick" || currentMode == "intercept_kick";
    const bool forceIdle = brain->tree->getEntry<bool>("wait_for_opponent_kickoff") ||
        brain->tree->getEntry<bool>("ball_out") ||
        brain->tree->getEntry<string>("gc_game_state") != "PLAY";

    const auto source = getGoalieBallSource(brain);
    const auto target = getGoaliePredictTarget(brain, source);
    double cooldownMs = 600.0;
    double maxRange = 5.0;
    double maxDirectionError = 1.2;
    brain->get_parameter("strategy.goalie_visual_kick_cooldown_ms", cooldownMs);
    brain->get_parameter("strategy.goalie_visual_kick_max_ball_range", maxRange);
    brain->get_parameter("strategy.goalie_visual_kick_max_direction_error", maxDirectionError);
    maxDirectionError = std::clamp(maxDirectionError, 0.2, M_PI);
    const bool cooldownOk = RLVisionKick::getLastExitTime().nanoseconds() == 0 ||
        brain->msecsSince(RLVisionKick::getLastExitTime()) >= cooldownMs;

    const double desiredClearanceDir = kick_geometry::directionToOpponentGoal(
        source.field.x, source.field.y, brain->config->fieldDimensions.length / 2.0);
    const double sourceDirField = kick_geometry::normalizeAngle(
        brain->data->robotPoseToField.theta + std::atan2(source.robot.y, source.robot.x));
    const double clearanceDirectionError = kick_geometry::angleDistance(
        desiredClearanceDir, sourceDirField);
    const bool clearanceDirectionReady = target.interceptEntryReady ||
        clearanceDirectionError <= maxDirectionError;

    bool teammateInVisualKick = false;
    const int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < brain->config->numOfPlayers && i < HL_MAX_NUM_PLAYERS; ++i) {
        if (i != selfIdx && brain->data->tmStatus[i].isAlive &&
            brain->data->tmStatus[i].isInVisualKick) {
            teammateInVisualKick = true;
            break;
        }
    }

    const bool baseReady = source.usable && brain->data->tmImLead &&
        !teammateInVisualKick &&
        (brain->data->tmMyCostRank == 0 || brain->data->tmMyCost < 2.0) &&
        source.range <= maxRange && cooldownOk &&
        clearanceDirectionReady &&
        source.field.x < -brain->config->fieldDimensions.circleRadius - 2.0;
    const double sessionMaxX = -brain->config->fieldDimensions.length / 2.0 +
        brain->config->fieldDimensions.penaltyAreaLength + 0.4;
    const bool sessionActive = active && visualKickMode && !forceIdle &&
        source.usable && source.field.x <= sessionMaxX;
    const bool visualKickReady = baseReady || sessionActive;

    if (!sessionActive && baseReady) {
        _interceptKickDisabled = false;
        _interceptKickCloseStartValid = false;
    }
    if (currentMode == "intercept_kick" && sessionActive) {
        bool disable = !target.predictionReliable || !target.targetValid || target.targetRobotDist > 2.0;
        if (target.targetValid && target.ballToTargetDist < 1.0) {
            if (!_interceptKickCloseStartValid) {
                _interceptKickCloseStartTime = brain->get_clock()->now();
                _interceptKickCloseStartValid = true;
            }
            disable = disable || brain->msecsSince(_interceptKickCloseStartTime) >= 1000.0;
        } else {
            _interceptKickCloseStartValid = false;
        }
        if (disable) _interceptKickDisabled = true;
    } else {
        _interceptKickCloseStartValid = false;
    }

    string nextMode;
    if (forceIdle) {
        nextMode = "guard_idle";
        _interceptKickDisabled = false;
    } else if (!visualKickReady) {
        nextMode = brain->data->ballDetected ? "guard_defend" : "guard_idle";
        _interceptKickDisabled = false;
    } else if (currentMode == "intercept_kick" && sessionActive && !_interceptKickDisabled) {
        nextMode = "intercept_kick";
    } else if (!_interceptKickDisabled && target.interceptEntryReady) {
        nextMode = "intercept_kick";
    } else {
        nextMode = "block_kick";
    }

    if (nextMode == "intercept_kick" && target.targetValid) {
        brain->data->goalieVisualKickSubState = "intercept_kick";
        brain->data->goalieVisualKickTargetValid = true;
        brain->data->goalieVisualKickTargetField = target.targetField;
    } else if (nextMode == "block_kick") {
        brain->data->goalieVisualKickSubState = "block_kick";
        brain->data->goalieVisualKickTargetValid = false;
    } else {
        brain->data->goalieVisualKickSubState = "none";
        brain->data->goalieVisualKickTargetValid = false;
    }
    brain->tree->setEntry<string>("goalie_mode", nextMode);
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieIdleHome::tick()
{
    double distToGoalLine = 1.0, vxLimit = 0.7, vyLimit = 0.5;
    double vthetaLimit = 1.2, posTolerance = 0.12, thetaTolerance = 0.25;
    getInput("dist_to_goalline", distToGoalLine);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("pos_tolerance", posTolerance);
    getInput("theta_tolerance", thetaTolerance);

    const double targetX = -brain->config->fieldDimensions.length / 2.0 + distToGoalLine;
    const auto &pose = brain->data->robotPoseToField;
    const double dist = std::hypot(targetX - pose.x, pose.y);
    const double thetaError = toPInPI(-pose.theta);
    const double releaseDist = std::max(posTolerance * 1.8, posTolerance + 0.35);
    const double releaseTheta = std::max(thetaTolerance * 2.0, thetaTolerance + 0.25);

    if (_positionLocked && dist <= releaseDist && std::abs(thetaError) <= releaseTheta) {
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        return NodeStatus::SUCCESS;
    }
    _positionLocked = false;
    if (dist <= posTolerance && std::abs(thetaError) <= thetaTolerance) {
        _positionLocked = true;
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        return NodeStatus::SUCCESS;
    }
    if (std::abs(thetaError) > thetaTolerance) {
        brain->client->setVelocity(0.0, 0.0,
            std::clamp(2.5 * thetaError, -vthetaLimit, vthetaLimit), false, false, false);
        return NodeStatus::SUCCESS;
    }

    const auto targetRobot = brain->data->field2robot({targetX, 0.0, 0.0});
    double vx = std::clamp(targetRobot.x, -vxLimit, vxLimit);
    double vy = std::clamp(targetRobot.y, -vyLimit, vyLimit);
    if (dist < 1.5) {
        const double scale = std::clamp(dist / 1.5, 0.25, 1.0);
        vx *= scale;
        vy *= scale;
    }
    brain->client->setVelocity(vx, vy, 0.0, false, false, false);
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;

    // 读取参数
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   // 用于调整 vx, vx *= vxFactor, 以补偿 x, y 方向的速度参数与实际速度比例的偏差, 使运动方向准确
    double yawOffset = brain->config->yawOffset; // 用于补偿定位角度的偏差

    // 计算速度指令
    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; // 移动的目标
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    { // 在可踢中的范围内, 尽量直走, 同时避免后面出现除 0 的问题.
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { // 否则计算出要向哪个方向移动, 并给出可实现的速度指令
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    // 估算移动所需时间
    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
     
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 0.1;
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 初始化 Node
    _startTime = brain->get_clock()->now();
    if (brain->config->enableStableKick && brain->threatLevel() < 0.5) _state = "stablize"; // false  开启后, 在 kick 时, 如风险较低, 则稳定一下再出脚.
    else _state = "kick";
     
    // 发布运动指令
    if (_state == "kick") {
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        bool softKickoff = true;
        double softKickoffSpeed = 0.3;
        brain->get_parameter("strategy.soft_kickoff", softKickoff);
        brain->get_parameter("strategy.soft_kickoff_speed", softKickoffSpeed);
        if (
            softKickoff
            && (brain->data->isFreekickKickingOff || brain->data->isKickingOff)
            ) speed = softKickoffSpeed;
        brain->client->crabWalk(angle, speed);
    } else if (_state == "stablize") {
        brain->client->setVelocity(-0.05, 0, 0, true, false, false);
    }
    // 返回运行中状态，表示节点将继续处理。
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };
    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    double MOVE_RANGE_THRESHOLD = 0.3;
    brain->get_parameter("strategy.abort_kick_ball_move_threshold", MOVE_RANGE_THRESHOLD);
    double KICK_RANGE = 1.0;
    brain->get_parameter("strategy.kick_range", KICK_RANGE);

    const double BALL_LOST_THRESHOLD = 1000;  // ms
    if (ballRange > KICK_RANGE){
        log("ball too far, abort kick");
        return NodeStatus::SUCCESS;
    }
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) // 球已经踢远
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD // 疑似丢球了
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }
    log(format("ballrange: %.1f, minRange: %.1f", ballRange, _minRange));
    if (ballRange < _minRange) _minRange = ballRange;    

    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (_state == "stablize") {
        double msecs = getInput<double>("msecs_stablize").value();
        if (brain->msecsSince(_startTime) > msecs) {
            _state = "kick";
            _startTime = brain->get_clock()->now();
            double angle = brain->data->ball.yawToRobot;
            double speed = getInput<double>("speed_limit").value();
            bool softKickoff = true;
            double softKickoffSpeed = 0.3;
            brain->get_parameter("strategy.soft_kickoff", softKickoff);
            brain->get_parameter("strategy.soft_kickoff_speed", softKickoffSpeed);
            if (
                softKickoff
                && (brain->data->isFreekickKickingOff || brain->data->isKickingOff)
                ) speed = softKickoffSpeed;
                brain->client->crabWalk(angle, speed);
            }
        return NodeStatus::RUNNING;
    } else if (_state == "kick") {
        double msecs = getInput<double>("min_msec_kick").value();
        double speed = getInput<double>("speed_limit").value();
        msecs = msecs + brain->data->ball.range / speed * 1000;
        if (brain->msecsSince(_startTime) > msecs) { // 完成踢球动作
            brain->client->setVelocity(0, 0, 0);
            return NodeStatus::SUCCESS;
        }
        // else
        if (brain->data->ballDetected) { // 如果还能看见球, 则修正方向
            double angle = brain->data->ball.yawToRobot;
            double speed = getInput<double>("speed_limit").value();
            _speed += 0.08;
            speed = min(speed, _speed);
            brain->client->crabWalk(angle, speed);
        }

        return NodeStatus::RUNNING;
    }

    // should not reach here
    prtErr("Kick: Reached impossible condition");
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}


rclcpp::Time RLVisionKick::_lastExitTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

NodeStatus RLVisionKick::onStart()
{
    const string goalieMode = brain->tree->getEntry<string>("goalie_mode");
    const bool goalieKick = goalieMode == "block_kick" ||
        goalieMode == "intercept_kick" ||
        brain->tree->getEntry<bool>("goalie_block_kick_active");
    _startTime = brain->get_clock()->now();
    _sessionStartTime = _startTime;
    _lastProgressTime = _startTime;
    _lastHeadCommandTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _headCommandSent = false;
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
    _pendingExitReason.clear();
    _extraRoundsUsed = 0;
    _badPoseConsecutiveCount = 0;
    _ballRangeOverLimitConsecutiveCount = 0;
    _lostOwnershipConsecutiveCount = 0;
    _progressBaselineValid = brain->data->ballDetected &&
        std::isfinite(brain->data->ball.range) &&
        std::isfinite(brain->data->ball.posToField.x) &&
        std::isfinite(brain->data->ball.posToField.y);
    if (_progressBaselineValid) {
        _progressBallRange = brain->data->ball.range;
        _progressBallField = brain->data->ball.posToField;
        _progressKickDir = brain->data->kickDir;
    }

    if (!visualKickBallUsable(brain, goalieKick)) {
        brain->data->tmImInVisualKick = false;
        brain->tree->setEntry<bool>("goalie_block_kick_active", false);
        brain->data->goalieVisualKickSubState = "none";
        brain->data->goalieVisualKickTargetValid = false;
        recordExitTime();
        return NodeStatus::SUCCESS;
    }

    brain->data->tmImInVisualKick = true;
    brain->tree->setEntry<bool>("goalie_block_kick_active",
        goalieKick);

    startDecelerate(visualKickDecelerateMs(brain, true));
    stepDecelerate();

    return NodeStatus::RUNNING;
}

NodeStatus RLVisionKick::onRunning()
{
    auto logExit = [=](const string &msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/RLVisionKick", rerun::TextLog(msg));
        std::cout << "[RLVisionKick] " << msg << std::endl;
    };

    const bool goalieKick = brain->tree->getEntry<bool>("goalie_block_kick_active");
    auto clearState = [&]() {
        brain->data->tmImInVisualKick = false;
        brain->tree->setEntry<bool>("goalie_block_kick_active", false);
        brain->data->goalieVisualKickSubState = "none";
        brain->data->goalieVisualKickTargetValid = false;
        brain->data->goalieVisualKickTargetField = {0.0, 0.0, 0.0};
    };
    auto beginExit = [&](const string &reason) -> NodeStatus {
        if (!_pendingRobocupWalk) {
            _pendingExitReason = reason;
            logExit("exit visual kick: " + reason);
            recordExitTime();
            startDecelerate(visualKickDecelerateMs(brain, false));
            _pendingRobocupWalk = true;
        }
        stepDecelerate();
        return NodeStatus::RUNNING;
    };

    if (brain->data->shouldExitRLVisionKick) {
        logExit("exit visual kick immediately: recovery requested");
        brain->data->shouldExitRLVisionKick = false;
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        clearState();
        recordExitTime();
        _isDecelerating = false;
        _pendingRobocupWalk = false;
        return NodeStatus::SUCCESS;
    }

    if (_isDecelerating) {
        if (!_pendingRobocupWalk && !visualKickBallUsable(brain, goalieKick)) {
            logExit("start cancelled: no usable ball source");
            clearState();
            recordExitTime();
            _isDecelerating = false;
            return NodeStatus::SUCCESS;
        }
        stepDecelerate();

        if (!_isDecelerating) {
            if (_pendingRobocupWalk) {
                if (_visionKickStarted) brain->client->robocupWalk();
                _pendingRobocupWalk = false;
                clearState();
                return NodeStatus::SUCCESS;
            } else if (!_visionKickStarted) {
                if (!visualKickBallUsable(brain, goalieKick)) {
                    clearState();
                    recordExitTime();
                    return NodeStatus::SUCCESS;
                }
                brain->client->RLVisionKick(true);
                _headScanStartTime = brain->get_clock()->now();
                _visionKickStarted = true;
            }
        }
        return NodeStatus::RUNNING;
    }

    if (_visionKickStarted) {
        double headMsec = brain->msecsSince(_headScanStartTime);
        // 先沿用 robocup_blue 的两段找球动作，再持续低头。CamFindAndTrackBall
        // 每帧都会先发头部命令，因此这里必须在 VisualKick 的整个会话中保持接管。
        const double targetPitch = headMsec < 300.0
            ? kVisionKickHeadScanPitch
            : kVisionKickHeadDownPitch;
        const bool commandDue = !_headCommandSent ||
            brain->msecsSince(_lastHeadCommandTime) >= kVisionKickHeadCommandIntervalMs;
        if (commandDue) {
            brain->client->moveHead(targetPitch, 0.0);
            _lastHeadCommandTime = brain->get_clock()->now();
            _headCommandSent = true;
        }
    }

    if (!visualKickBallUsable(brain, goalieKick)) {
        return beginExit("no usable/fresh ball source");
    }

    double elapsed = brain->msecsSince(_startTime);
    const double sessionElapsed = brain->msecsSince(_sessionStartTime);
    double minMsecKick = getInput<double>("min_msec_kick").value();
    double maxMsecKick = getInput<double>("max_msec_kick").value();
    double rangeThreshold = getInput<double>("range").value();

    double ballRange = brain->data->ball.range;
    double ballRobotX = brain->data->ball.posToRobot.x;
    double ballYaw = brain->data->ball.yawToRobot;
    Point ballField = brain->data->ball.posToField;
    if (!visualKickCurrentRealBallFresh(brain) && brain->data->reliableBallPredict) {
        ballRange = std::hypot(brain->data->ball.reliablePredictPosToRobot.x,
                               brain->data->ball.reliablePredictPosToRobot.y);
        ballRobotX = brain->data->ball.reliablePredictPosToRobot.x;
        ballYaw = std::atan2(brain->data->ball.reliablePredictPosToRobot.y,
                            brain->data->ball.reliablePredictPosToRobot.x);
        ballField = brain->data->ball.reliablePredictPosToField;
    }

    _ballRangeOverLimitConsecutiveCount =
        std::isfinite(ballRange) && ballRange > rangeThreshold
            ? _ballRangeOverLimitConsecutiveCount + 1
            : 0;
    _badPoseConsecutiveCount =
        std::isfinite(ballRange) && ballRange > 0.10 &&
        (ballRobotX < kVisionKickBadPoseBackX || std::fabs(ballYaw) > kVisionKickBadPoseYawLimit)
            ? _badPoseConsecutiveCount + 1
            : 0;
    const bool nearRealBall = visualKickNearRealBall(brain);
    _lostOwnershipConsecutiveCount = (brain->data->tmImLead || nearRealBall)
        ? 0
        : _lostOwnershipConsecutiveCount + 1;

    double progressRangeDelta = 0.12;
    double progressBallForwardDelta = 0.15;
    double noProgressTimeout = 3500.0;
    double maxTotal = 14000.0;
    brain->get_parameter("strategy.auto_visual_kick.progress_range_delta", progressRangeDelta);
    brain->get_parameter("strategy.auto_visual_kick.progress_ball_forward_delta", progressBallForwardDelta);
    brain->get_parameter("strategy.auto_visual_kick.no_progress_timeout_ms", noProgressTimeout);
    brain->get_parameter("strategy.auto_visual_kick.max_total_ms", maxTotal);
    if (!_progressBaselineValid && std::isfinite(ballRange) &&
        std::isfinite(ballField.x) && std::isfinite(ballField.y)) {
        _progressBaselineValid = true;
        _progressBallRange = ballRange;
        _progressBallField = ballField;
        _progressKickDir = brain->data->kickDir;
        _lastProgressTime = brain->get_clock()->now();
    } else if (_progressBaselineValid) {
        const double forwardMove =
            (ballField.x - _progressBallField.x) * std::cos(_progressKickDir) +
            (ballField.y - _progressBallField.y) * std::sin(_progressKickDir);
        if (_progressBallRange - ballRange >= progressRangeDelta ||
            forwardMove >= progressBallForwardDelta) {
            _progressBallRange = ballRange;
            _progressBallField = ballField;
            _progressKickDir = brain->data->kickDir;
            _lastProgressTime = brain->get_clock()->now();
        }
    }

    bool teammateWinsVisualKick = false;
    double minTeammateVisualKickCost = std::numeric_limits<double>::infinity();
    int minTeammateVisualKickId = 0;
    const int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < brain->config->numOfPlayers && i < HL_MAX_NUM_PLAYERS; ++i) {
        if (i == selfIdx) continue;
        const auto &tm = brain->data->tmStatus[i];
        if (!tm.isAlive || !tm.isInVisualKick) continue;
        if (tm.cost < minTeammateVisualKickCost) {
            minTeammateVisualKickCost = tm.cost;
            minTeammateVisualKickId = i + 1;
        }
    }
    if (minTeammateVisualKickId > 0) {
        teammateWinsVisualKick =
            minTeammateVisualKickCost < brain->data->tmMyCost - 0.05 ||
            (std::fabs(minTeammateVisualKickCost - brain->data->tmMyCost) <= 0.05 &&
             minTeammateVisualKickId < brain->config->playerId);
    }
    int maxParallel = 1;
    double yieldMyCostMin = 4.0;
    brain->get_parameter("strategy.auto_visual_kick.max_parallel_count", maxParallel);
    brain->get_parameter("strategy.auto_visual_kick.yield_my_cost_min", yieldMyCostMin);
    const bool shouldYield = !nearRealBall && teammateWinsVisualKick &&
        (maxParallel <= 1 || brain->data->tmMyCost > yieldMyCostMin);

    const bool elapsedEnough = elapsed > minMsecKick;
    if (brain->tree->getEntry<bool>("ball_out")) return beginExit("ball out");
    if (_badPoseConsecutiveCount >= kVisionKickBadPoseMaxTicks) return beginExit("bad ball pose");
    if (shouldYield) return beginExit("teammate wins visual-kick slot");
    if (_lostOwnershipConsecutiveCount >= 3) return beginExit("lost team ball ownership");
    if (sessionElapsed >= maxTotal) return beginExit("session timeout");
    if (brain->msecsSince(_lastProgressTime) >= noProgressTimeout) return beginExit("no progress");
    if (elapsedEnough && _ballRangeOverLimitConsecutiveCount >= 4) return beginExit("ball range over limit");
    if (elapsedEnough && !nearRealBall && brain->data->tmMyCost > 8.0) return beginExit("cost too high");

    if (elapsed > maxMsecKick) {
        int maxExtraRounds = 1;
        brain->get_parameter("strategy.auto_visual_kick.max_extra_rounds", maxExtraRounds);
        maxExtraRounds = std::clamp(maxExtraRounds, 0, 4);
        if (_extraRoundsUsed < maxExtraRounds && visualKickBallUsable(brain, goalieKick)) {
            ++_extraRoundsUsed;
            _startTime = brain->get_clock()->now();
            return NodeStatus::RUNNING;
        }
        return beginExit("maximum kick duration reached");
    }

    return NodeStatus::RUNNING;
}

void RLVisionKick::onHalted()
{
    const string haltMsg = "halted by behavior tree, force exit visual kick";
    brain->log->setTimeNow();
    brain->log->log("debug/RLVisionKick", rerun::TextLog(haltMsg));
    std::cout << "[RLVisionKick] " << haltMsg << std::endl;
    brain->data->tmImInVisualKick = false;
    brain->tree->setEntry<bool>("goalie_block_kick_active", false);
    brain->data->goalieVisualKickSubState = "none";
    brain->data->goalieVisualKickTargetValid = false;
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
    if (_visionKickStarted) brain->client->robocupWalk();
    recordExitTime();

    _isDecelerating = false;
    _visionKickStarted = false;
    _headCommandSent = false;
    _lastHeadCommandTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    _pendingRobocupWalk = false;
    _pendingExitReason.clear();
}

bool RLVisionKick::isMinIntervalSatisfied(double minIntervalMsec)
{
    if (_lastExitTime.nanoseconds() == 0) return true;
    const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    return (now - _lastExitTime).nanoseconds() / 1e6 >= minIntervalMsec;
}

void RLVisionKick::recordExitTime()
{
    _lastExitTime = brain->get_clock()->now();
}

void RLVisionKick::startDecelerate(double durationMs)
{
    if (_isDecelerating) {
        return;
    }

    _isDecelerating = true;
    _decelStartTime = brain->get_clock()->now();
    _decelDurationMs = durationMs;
}

bool RLVisionKick::stepDecelerate()
{
    if (!_isDecelerating) {
        return true;
    }

    double elapsed = brain->msecsSince(_decelStartTime);
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);

    if (elapsed >= _decelDurationMs) {
        _isDecelerating = false;
        return true;
    }

    return false;
}


NodeStatus Intercept2::tick()
{
    double distToGoalLine = 1.0;
    getInput("dist_to_goalline", distToGoalLine);

    const auto &fd = brain->config->fieldDimensions;
    const double halfGoalWidth = fd.goalWidth / 2.0;
    const double goalLineX = -fd.length / 2.0;
    const double anchorX = goalLineX + distToGoalLine;
    const auto &pose = brain->data->robotPoseToField;
    const auto &ball = brain->data->ball.posToField;
    const double ballVx = brain->data->ballEstVelX;
    const double ballVy = brain->data->ballEstVelY;

    double threatVx = -0.03, minPredictVx = 0.06;
    double maxGoalTime = 8.0, minPlaneTime = 0.02, maxPlaneTime = 6.0;
    brain->get_parameter("strategy.goalie_simple_threat_vxf", threatVx);
    brain->get_parameter("strategy.goalie_simple_predict_min_vxf", minPredictVx);
    brain->get_parameter("strategy.goalie_simple_t_goal_max", maxGoalTime);
    brain->get_parameter("strategy.goalie_simple_t_plane_min", minPlaneTime);
    brain->get_parameter("strategy.goalie_simple_t_plane_max", maxPlaneTime);

    bool threateningGoal = false;
    if (ballVx < threatVx) {
        const double timeToGoal = (goalLineX - ball.x) / ballVx;
        if (timeToGoal >= 0.0 && timeToGoal < maxGoalTime) {
            threateningGoal = std::abs(ball.y + timeToGoal * ballVy) <= halfGoalWidth;
        }
    }

    double targetY = std::clamp(ball.y, -halfGoalWidth, halfGoalWidth);
    if (threateningGoal && std::abs(ballVx) > minPredictVx) {
        const double timeToAnchor = (anchorX - ball.x) / ballVx;
        if (timeToAnchor > minPlaneTime && timeToAnchor < maxPlaneTime) {
            targetY = std::clamp(ball.y + timeToAnchor * ballVy,
                                 -halfGoalWidth, halfGoalWidth);
        }
    }

    double xKp = 0.9, yKp = 2.2, vxMax = 1.2, vyMax = 0.8;
    double thetaKp = 1.4, thetaMax = 1.5;
    brain->get_parameter("strategy.goalie_simple_x_kp", xKp);
    brain->get_parameter("strategy.goalie_simple_y_kp", yKp);
    brain->get_parameter("strategy.goalie_simple_vx_max", vxMax);
    brain->get_parameter("strategy.goalie_simple_vy_max", vyMax);
    brain->get_parameter("strategy.goalie_simple_vtheta_kp", thetaKp);
    brain->get_parameter("strategy.goalie_simple_vtheta_limit", thetaMax);

    const double vxField = std::clamp(xKp * (anchorX - pose.x), -vxMax, vxMax);
    const double vyField = std::clamp(yKp * (targetY - pose.y), -vyMax, vyMax);
    double vxRobot = vxField * std::cos(pose.theta) + vyField * std::sin(pose.theta);
    double vyRobot = -vxField * std::sin(pose.theta) + vyField * std::cos(pose.theta);
    vxRobot = std::clamp(vxRobot, -vxMax, vxMax);
    vyRobot = std::clamp(vyRobot, -vyMax, vyMax);
    const double vtheta = std::clamp(brain->data->ball.yawToRobot * thetaKp,
                                     -thetaMax, thetaMax);

    brain->client->setVelocity(vxRobot, vyRobot, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus StandStill::onStart()
{
    // 初始化 Node
    _startTime = brain->get_clock()->now();

    // 发布运动指令
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning()
{
    double msecs;
    getInput("msecs", msecs);
    if (brain->msecsSince(_startTime) < msecs) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    // else
    return NodeStatus::SUCCESS;
}

void StandStill::onHalted()
{
    double msecs;
    getInput("msecs", msecs);
    _startTime -= rclcpp::Duration(- 2 * msecs, 0);
}


NodeStatus RobotFindBall::onStart()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onStart");

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
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onRunning");

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
    if (brain->data->ball.range < 0.3)
    { // 记忆中的球位置太近了, 后退一点
      // vx = cap(-brain->data->ball.posToRobot.x, 0.2, -0.2);
      // vy = cap(-brain->data->ball.posToRobot.y, 0.2, -0.2);
    }
    // vtheta = _turnDir > 0 ? vyawLimit : -vyawLimit;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onHalted");
    _turnDir = 1.0;
}

NodeStatus CamFastScan::onStart()
{
    _cmdIndex = 0;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(_timeLastCmd) < interval) return NodeStatus::RUNNING;

    // else 
    if (_cmdIndex >= 6) return NodeStatus::SUCCESS;

    // else
    _cmdIndex++;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    // brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
    //      "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
    //      _angle, _cumAngle, deltaAngle, turnTime
    // )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else 
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}

// ... (此处为 SelfLocate 系列函数的完整实现，因篇幅过长未全部展开，但包含所有逻辑，如 SelfLocate, SelfLocateEnterField, SelfLocateLine 等，保持原文件不变) ...

NodeStatus SelfLocate::tick()
{
    // ... (保留原文件 SelfLocate::tick 的完整实现) ...
    // 为确保完整性，这里列出 SelfLocate::tick
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/SelfLocate", rerun::TextLog(msg));
    };
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    string mode = getInput<string>("mode").value();
    double xMin, xMax, yMin, yMax, thetaMin, thetaMax; // 结束条件
    auto markers = brain->data->getMarkersForLocator();

    // 计算约束条件
    if (mode == "enter_field")
    {
        // x 范围: 已方半场，不超过中圈的边界
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;

        // y 范围: 边线外，距离边线 1 m 以内
        // yMin = - brain->config->fieldDimensions.width / 2 - 1.0;
        // yMax = brain->config->fieldDimensions.width / 2 + 1.0;
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

        // Theta 范围: 面向赛场，左右偏移不超过 15 度
        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 12;
            thetaMax = -M_PI / 2 + M_PI / 12;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 12;
            thetaMax = M_PI / 2 + M_PI / 12;
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
        double maxDriftSpeed = 0.1;                       // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = max(-brain->config->fieldDimensions.length / 2 - 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2 + 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2 - 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2 + 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }
    else if (mode == "fall_recovery")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                       // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }

    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    // 调用粒子滤波定位器
    auto res = brain->locator->locateRobot(markers, constraints);

    brain->log->setTimeNow();
    string mstring = "";
    for (int i = 0; i < markers.size(); i++) {
        auto m = markers[i];
        mstring += format("type: %c  x: %.1f y: %.1f", m.type, m.x, m.y);
    }
    if (res.success) {
        
        brain->log->log(
            "field/recal",
            rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(res.success ? 0x00FF00FF : 0xFF0000FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"pf"})
        );
    }
    log(
        format(
            "success: %d  residual: %.2f  marker.size: %d  minMarkerCnt: %d  resTolerance: %.2f marker: %s",
            res.success,
            res.residual,
            markers.size(),
            brain->locator->minMarkerCnt,
            brain->locator->residualTolerance,
            mstring.c_str()
        )
    );
    
    // 定位失败
    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    // else 定位成功
    // trust_direction/fall_recovery 的约束以当前位姿为中心，其结果也属于
    // 当前时刻基准；不能再叠加从视觉帧到当前的 odom 转角。
    const bool resultPoseIsAtVisionTime =
        mode == "enter_field" || mode == "face_forward";
    brain->calibrateOdom(
        res.pose.x, res.pose.y, res.pose.theta, resultPoseIsAtVisionTime);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("定位成功: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

// ... (其余 SelfLocate 函数如 SelfLocateEnterField, SelfLocateLine 等保持原文件逻辑) ...
// SelfLocateEnterField: 自动检测左右入场位置
NodeStatus SelfLocateEnterField::tick()
{
    auto log = [=](string msg, bool success) {
        brain->log->setTimeNow();
        brain->log->log("debug/SelfLocateEnterField", rerun::TextLog(msg).with_level(success? rerun::TextLogLevel::Info : rerun::TextLogLevel::Error));
    };
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    auto markers = brain->data->getMarkersForLocator();
    auto fd = brain->config->fieldDimensions;
    
    // 定义左右两边的约束条件
    PoseBox2D cEnterLeft = {-fd.length / 2, -fd.circleRadius, fd.width / 2, fd.width / 2 + 1, -M_PI / 2 - M_PI / 12, -M_PI / 2 + M_PI / 12};
    PoseBox2D cEnterRight = {-fd.length / 2, -fd.circleRadius, -fd.width / 2 - 1, -fd.width / 2, M_PI / 2 - M_PI / 12, M_PI / 2 + M_PI / 12};

    // 分别尝试左右两边定位
    auto resLeft = brain->locator->locateRobot(markers, cEnterLeft);
    auto resRight = brain->locator->locateRobot(markers, cEnterRight);
    LocateResult res;

    static string lastReport = "";
    string report = lastReport;
    
    // 根据定位结果选择最佳位置
    if (resLeft.success && !resRight.success) {
        res = resLeft;
        report = "Entering Left";
    }
    else if (!resLeft.success && resRight.success) {
        res = resRight;
        report = "Entering Right";
    }
    else if (resLeft.success && resRight.success) {
        // 两边都成功，选择残差更小的
        if (resLeft.residual < resRight.residual) {
            res = resLeft;
            report = "Entering Left";
        }
        else {
            res = resRight;
            report = "Entering Right";
        }
    } else {
        res = resLeft; // 两边都失败，默认使用左边结果
    }

    if (report != lastReport) {
        brain->speak(report);
        lastReport = report;
    }

    brain->log->setTimeNow();
    string logPath = res.success ? "debug/locator_enter_field/success" : "debug/locator_enter_field/fail";
    log(
            format(
                "%s left success: %d  left residual: %.2f  right success %d  right residual %.2f resTolerance: %.2f markers: %d minMarkerCnt: %d ",
                report.c_str(),
                resLeft.success, 
                resLeft.residual,
                resRight.success,
                resRight.residual,
                brain->locator->residualTolerance,
                markers.size(),
                brain->locator->minMarkerCnt
            ),
            res.success
        );

    brain->log->log(
        "field/recal_enter_field", 
        rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(res.success ? 0x00FF00FF: 0xFF0000FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"pfe"})
    );

    if (!res.success) return NodeStatus::SUCCESS; // 不阻塞后面的节点

    // 定位成功
    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta, true);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("定位成功: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " +  to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));


    return NodeStatus::SUCCESS;
}

bool SelfLocateLocal::_singlePenalty() {
    auto penaltyPoints = brain->data->getMarkingsByType({"PenaltyPoint"});
    if (penaltyPoints.size() != 1) {
        brain->log->log("SelfLocateLocal/SinglePenalty",
            rerun::TextLog(format("Failed, penaltyPoints.size() = %d", penaltyPoints.size()))
        );
        return false;
    }

    // int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
    // double maxDriftSpeed = 0.1;                      // m/s
    // double maxDrift = msec / 1000.0 * maxDriftSpeed; // 在这个时间内, odom 最多漂移了多少距离

    double maxDrift = 2.0;

    brain->log->setTimeNow();

    auto pp = penaltyPoints[0]; // 观察到的 penalty point
    if (pp.range > 5.0) {
        brain->log->log("SelfLocateLocal/SinglePenalty", rerun::TextLog(format("Failed, Penalty point is too far (%.2f)", pp.range)));
        return false;
    }

    // 判断是哪一个罚球点
    auto fd = brain->config->fieldDimensions;
    Point2D ppo({fd.length / 2 - fd.penaltyDist, 0}); // 对方 penalty point 的地图位置
    Point2D pps({-fd.length / 2 + fd.penaltyDist, 0}); // 我方 penalty point 的地图位置
    Point2D ppt; // t:target, 经过判断, 看到的是哪一个 penalty point

    double disto = norm(pp.posToField.x - ppo.x, pp.posToField.y - ppo.y); // 与对方 pp 的观察值与理论值的距离
    double dists = norm(pp.posToField.x - pps.x, pp.posToField.y - pps.y); // 与已方 pp 的观察值与理论值的距离

    if (disto < dists && disto < maxDrift)  ppt = ppo;
    else if (dists < disto && dists < maxDrift) ppt = pps;
    else {
        brain->log->log("SelfLocateLocal/SinglePenalty",
            rerun::TextLog(format("Failed: disto= %.2f  dists=%.2f maxDrift=%.2f", disto, dists, maxDrift))
        );
        return false;
    }

    // if a valid ppt is found, 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += ppt.x - pp.posToField.x;
    hypoPose.y += ppt.y - pp.posToField.y;

    // validate the hypo with other 
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() < 3) {
        brain->log->log("SelfLocateLocal/SinglePenalty",
            rerun::TextLog("Failed. Not enough markers for validation.")
        );
        return false;        
    }
    double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
    if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
        brain->log->log("SelfLocateLocal/SinglePenalty",
            rerun::TextLog("Failed, validation failed. Possible misdetection.")
        );
        return false;
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    brain->log->log("SelfLocateLocal/SinglePenalty", rerun::TextLog(format("Success. Residual = %.2f", residual)));
    brain->log->log(
        "field/recal",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    return true;
}

bool SelfLocateLocal::_doubleX() {
    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        brain->log->log("SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, points.size() = %d", points.size()))
        );
        return false;
    }

    auto p0 = points[0]; auto p1 = points[1];

    if (
        fabs(p0.posToField.y - p1.posToField.y) > 0.3 // 方向不对
        || fabs(fabs(p0.posToField.y - p1.posToField.y) - brain->config->fieldDimensions.circleRadius * 2.0) > 0.5 // 距离不对
        || p0.range > 5.0 || p1.range > 5.0 // 太远
    ) {
        brain->log->log("SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, did not pass feature validation. dy = %.2f, dist = %.2f, range = [%.2f, %.2f]",
                p0.posToField.y - p1.posToField.y,
                fabs(p0.posToField.y - p1.posToField.y) - brain->config->fieldDimensions.circleRadius * 2.0,
                p0.range, p1.range
        ))
        );
        return false;
    }

    // 观察到的球场中心点的坐标
    double xc = (p0.posToField.x + p1.posToField.x) / 2.0;
    double yc = (p1.posToField.y + p1.posToField.y) / 2.0;

    double maxDrift = 2.0;
    if (norm(xc, yc) > maxDrift) {
        brain->log->log("SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, dist = %.2f > maxDrift(%.2f)", norm(xc, yc), maxDrift))
        );
        return false; 
    }
    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    brain->log->setTimeNow();
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x -= xc;
    hypoPose.y -= yc;

    // validate the hypo with other markings
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() < 3) {
        brain->log->log("SelfLocateLocal/DoubleX",
            rerun::TextLog("Failed. Not enough markers for validation.")
        );
        return false;        
    }
    double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
    if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
        brain->log->log("SelfLocateLocal/DoubleX",
            rerun::TextLog("Failed, validation failed. Possible misdetection.")
        );
        return false;
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    brain->log->log("SelfLocateLocal/DoubleX", rerun::TextLog(format("Success. Residual = %.2f", residual)));
    brain->log->log(
        "field/recal",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2x"})
    );
    return true;
}

NodeStatus SelfLocateLocal::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    if (_singlePenalty()) return NodeStatus::SUCCESS;
    if (_doubleX()) return NodeStatus::SUCCESS;
    // TODO other features

    // All Features failed
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1P::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/1p/success";
    string logPathF = "/locate/1p/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto penaltyPoints = brain->data->getMarkingsByType({"PenaltyPoint"});
    if (penaltyPoints.size() != 1) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, penaltyCnt(%d) != 1", penaltyPoints.size()))
        );
        return NodeStatus::SUCCESS;
    }

    auto pp = penaltyPoints[0];
    if (pp.range > maxDist) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, penalty Dist(%.2f) > maxDist(%.2f)", pp.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    if (!brain->isBoundingBoxInCenter(pp.boundingBox)) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, boundingbox is not in the center area"))
        );
        return NodeStatus::SUCCESS;
    }

    // 判断是哪一个罚球点
    auto fd = brain->config->fieldDimensions;
    Point2D ppo({fd.length / 2 - fd.penaltyDist, 0}); // 对方 penalty point 的地图位置
    Point2D pps({-fd.length / 2 + fd.penaltyDist, 0}); // 我方 penalty point 的地图位置
    double dx, dy; // 偏移量, 理论 - 实际

    double disto = norm(pp.posToField.x - ppo.x, pp.posToField.y - ppo.y); // 与对方 pp 的观察值与理论值的距离
    double dists = norm(pp.posToField.x - pps.x, pp.posToField.y - pps.y); // 与已方 pp 的观察值与理论值的距离

    if (disto < dists && disto < maxDrift) { // 是敌方罚球点
        dx = ppo.x - pp.posToField.x;
        dy = ppo.y - pp.posToField.y;
    }
    else if (dists < disto && dists < maxDrift) {
        dx = pps.x - pp.posToField.x;
        dy = pps.y - pp.posToField.y;
    }
    else {
        log->log(logPathF,
            rerun::TextLog(format("Failed: disto= %.2f dists=%.2f maxDrift=%.2f", disto, dists, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }

    // if a valid ppt is found, 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    double drift = norm(dx, dy);
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/1p/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1M::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/1m/success";
    string logPathF = "/locate/1m/fail";

    // 避免过于频繁定位
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // find nearest marker
    int markerIndex = -1;
    GameObject marker;
    MapMarking mapMarker; 
    double minDist = 100;
    auto markings = brain->data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];

        // 排除掉容易误识别引起误判的点
        if (m.name == "LOLG" || m.name == "LORG" || m.name == "LSLG" || m.name == "LSRG") continue; 

        if (m.range < minDist) {
            minDist = m.range;
            markerIndex = i;
            marker = m;
        }
    }
    
    if (
        markerIndex < 0 || markerIndex >= markings.size()
        || marker.id < 0 || marker.id >= brain->config->mapMarkings.size()
    ) {
        log->log(logPathF, rerun::TextLog("Failed, No markings Found. Or marker id invalid."));
        return NodeStatus::SUCCESS;
    }
    mapMarker = brain->config->mapMarkings[marker.id];

    // 检查距离, 太远不用. (因为远了测距不准)
    if (marker.range > maxDist) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, min marker Dist(%.2f) > maxDist(%.2f)", marker.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    // 检查视野中的位置, 太偏的不用. (因为可能有扭曲, 导致测距有问题)
    if (!brain->isBoundingBoxInCenter(marker.boundingBox)) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, boundingbox is not in the center area"))
        );
        return NodeStatus::SUCCESS;
    }

    double dx, dy; // 偏移量, 理论 - 实际
    dx = mapMarker.x - marker.posToField.x;
    dy = mapMarker.y - marker.posToField.y;

    // 偏量太大, 不用. 可能是误识别导致的.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    brain->log->log(
        "field/recal/1m/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({marker.name})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2X::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2x/success";
    string logPathF = "/locate/2x/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, point cnt(%d) != 2", points.size()))
        );
        return NodeStatus::SUCCESS;
    }

    auto p0 = points[0]; auto p1 = points[1];
    
    if (p0.range > maxDist || p1.range > maxDist) { // 太远
        log->log(logPathF,
            rerun::TextLog(format("Failed, p0 range (%.2f) or p1 range (%.2f) > maxDist(%.2f)", p0.range, p1.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    double xDist = fabs(p0.posToField.x - p1.posToField.x);
    if (xDist > 0.5) { // 方向不对
        log->log(logPathF,
            rerun::TextLog(format("Failed, xDist(%.2f) > maxDist(%.2f)", xDist, 0.5))
        );
        return NodeStatus::SUCCESS;
    }

    double yDist = fabs(p0.posToField.y - p1.posToField.y);
    double mapYDist = brain->config->fieldDimensions.circleRadius * 2.0;
    if (fabs(yDist - mapYDist) > 0.5) { // 距离不对
        log->log(logPathF,
            rerun::TextLog(format("Failed, yDist(%.2f) too far (%.2f) from mapYDist(%.2f)", yDist, 0.5, mapYDist))
        );
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值
    double dx = - (p0.posToField.x + p1.posToField.x) / 2.0;
    double dy = - (p1.posToField.y + p1.posToField.y) / 2.0;
    double drift = norm(dx, dy);

    if (drift > maxDrift) { // 修正量过大
        log->log(logPathF,
            rerun::TextLog(format("Failed, dirft(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2x/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2T::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2t/success";
    string logPathF = "/locate/2t/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto markers = brain->data->getMarkingsByType({"TCross"});
    GameObject m1, m2;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < markers.size(); i++) {
        m1 = markers[i];
        
        if (m1.range > maxDist) continue; // 太远

        for (int j = i + 1; j < markers.size(); j++) {
            m2 = markers[j];

            if (m2.range > maxDist) continue; // 太远

            if (
                fabs(m1.posToField.x - m2.posToField.x) < 0.3
                && fabs(fabs(m1.posToField.y - m2.posToField.y) - fabs(fd.goalAreaWidth - fd.penaltyAreaWidth)/2.0)< 0.3
            ) {
                found = true;
                break;
            }
        }
        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (m1.posToField.x + m2.posToField.x)/2,
        (m1.posToField.y + m2.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0), 
                side * (fd.penaltyAreaWidth + fd.goalAreaWidth) / 4.0
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }

    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2t/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateLT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/lt/success";
    string logPathF = "/locate/lt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    auto lMarkers = brain->data->getMarkingsByType({"LCross"});

    GameObject t, l;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < tMarkers.size(); i++) {
        t = tMarkers[i];
        
        if (t.range > maxDist) continue; // 太远

        for (int j = i + 1; j < lMarkers.size(); j++) {
            l = lMarkers[j];

            if (l.range > maxDist) continue;

            if (
                fabs(t.posToField.y - l.posToField.y) < 0.3
                && fabs(fabs(t.posToField.x - l.posToField.x) - fd.goalAreaLength)< 0.3
            ) {
                found = true;
                break;
            }
        }

        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within MaxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (t.posToField.x + l.posToField.x)/2,
        (t.posToField.y + l.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0 - fd.goalAreaLength / 2.0), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/lt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocatePT::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/pt/success";
    string logPathF = "/locate/pt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto posts = brain->data->getGoalposts();
    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    
    GameObject p, t;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < posts.size(); i++) {
        p = posts[i];
        if (p.range > maxDist) continue;

        for (int j = i + 1; j < tMarkers.size(); j++) {
            t = tMarkers[j];
            if (t.range > maxDist) continue;
            if (
                fabs(t.posToField.x - p.posToField.x) < 0.5
                && fabs(fabs(t.posToField.x - p.posToField.x) - fabs(fd.goalAreaWidth - fd.goalWidth) / 2.0) < 0.3
            ) {
                found = true;
                break;
            }
        }
        
        if (found) break;
    }


    if (!found) {
        log->log(logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        t.posToField.x,
        t.posToField.y
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->log(logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 理论与实际的差值, 理论 - 实际
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // 到此为止, 一切看起来 ok 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/pt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateBorder::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/border/success";
    string logPathF = "/locate/border/fail";

    // 避免过于频繁定位
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // // 非静止状态线的定位不稳定
    // if (!brain->client->isStandingStill(1000)) {
    //     log->log(logPathF, rerun::TextLog(format("Failed, Not Standing Still")));
    //     return NodeStatus::SUCCESS;
    // }
    
    // find best touchline and best goalline
    bool touchLineFound = false;
    FieldLine touchLine;
    double bestConfidenceTouchline = 0.0;
    bool goalLineFound = false;
    FieldLine goalLine;
    double bestConfidenceGoalline = 0.0;
    bool middleLineFound = false;
    FieldLine middleLine;
    double bestConfidenceMiddleLine = 0.0;

    auto fieldLines = brain->data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (line.type != LineType::TouchLine && line.type != LineType::GoalLine && line.type != LineType::MiddleLine) continue;
        if (line.confidence < 0.8) continue;
        
        double dist = pointMinDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}), 
            line.posToField
        );
        if (dist > maxDist) continue;

        if (line.type == LineType::TouchLine) {
           if (line.confidence > bestConfidenceTouchline) {
               bestConfidenceTouchline = line.confidence;
               touchLine = line;
               touchLineFound = true;
           }
        } else if (line.type == LineType::GoalLine) {
            if (line.confidence > bestConfidenceGoalline) {
                bestConfidenceGoalline = line.confidence;
                goalLine = line;
                goalLineFound = true;
            }
        } else if (line.type == LineType::MiddleLine) {
            if (line.confidence > bestConfidenceMiddleLine) {
                bestConfidenceMiddleLine = line.confidence;
                middleLine = line;
                middleLineFound = true;
            }
        }
    }

    // 计算校正量
    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    if (touchLineFound) {
       double y_m = touchLine.side == LineSide::Left ? fd.width / 2.0 : - fd.width / 2.0;
       double perpDist = pointPerpDistToLine(
           Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
           touchLine.posToField
       );
       double y_o = touchLine.side == LineSide::Left ? 
           brain->data->robotPoseToField.y - perpDist :
           brain->data->robotPoseToField.y + perpDist;
       dy = y_m - y_o;
    }
    if (goalLineFound) {
        double x_m = goalLine.half == LineHalf::Opponent ? fd.length / 2.0: - fd.length / 2.0;
        double perpDist = pointPerpDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
            goalLine.posToField
        );
        double x_o = goalLine.half == LineHalf::Opponent?
            brain->data->robotPoseToField.x - perpDist :
            brain->data->robotPoseToField.x + perpDist;
        dx = x_m - x_o;
    } else if (middleLineFound) {
        double x_m = 0;
        auto linePos = middleLine.posToField;
        auto robotPose = brain->data->robotPoseToField;
        vector<double> pointA(2);
        vector<double> pointB(2);
        vector<double> pointR = {robotPose.x, robotPose.y};

        if (linePos.y0 > linePos.y1) {
            pointA = {linePos.x0, linePos.y0};
            pointB = {linePos.x1, linePos.y1};
        } else {
            pointA = {linePos.x1, linePos.y1};
            pointB = {linePos.x0, linePos.y0};
        }

        vector<double> vl = {pointB[0] - pointA[0], pointB[1] - pointA[1]};
        vector<double> vr = {pointR[0] - pointA[0], pointR[1] - pointA[1]};

        double normvl = norm(vl);
        double normvr = norm(vr);
        if (normvl < 1e-3 || normvr < 1e-3) {
            dx = 10000; // a large enough number that will certainly be bigger than max drift
        } else {
            double dist = crossProduct(vr, vl) / normvl;
            double x_o = robotPose.x + dist;
            dx = x_m - x_o;
        }
    }

    // 没找到
    if ((!touchLineFound && !goalLineFound && !middleLineFound)) {
        log->log(logPathF,
            rerun::TextLog("No touchline or goalline or middleLine found.")
        );
        return NodeStatus::SUCCESS;
    }

    // 偏量太大, 不用. 可能是误识别导致的.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->log(logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // validation failed. 可能看到的 penalty mark 为误识别
            log->log(logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    string label = "";
    if (touchLineFound) label += "TouchLine";
    if (touchLineFound && (goalLineFound || middleLineFound)) label += " ";
    if (goalLineFound) label += "GoalLine";
    if (middleLineFound) label += "MiddleLine";
    brain->log->log(
        "field/recal/border/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({label})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}
double SelfLocateLine::lineToLineAvgDist(const FieldLine& a, const FieldLineRef& b, int samples) {
    double sum = 0.0;
    for (int i = 0; i <= samples; ++i) {
        double t = double(i) / samples;
        double x = a.posToField.x0 + t * (a.posToField.x1 - a.posToField.x0);
        double y = a.posToField.y0 + t * (a.posToField.y1 - a.posToField.y0);
        // 点(x, y)到线段B的最短距离
        double dx = b.x1 - b.x0;
        double dy = b.y1 - b.y0;
        double len2 = dx*dx + dy*dy;
        double t_proj = ((x - b.x0) * dx + (y - b.y0) * dy) / (len2 + 1e-8);
        t_proj = std::max(0.0, std::min(1.0, t_proj));
        double px = b.x0 + t_proj * dx;
        double py = b.y0 + t_proj * dy;
        double dist = sqrt((x - px)*(x - px) + (y - py)*(y - py));
        sum += dist;
    }
    return sum / (samples + 1);
}

NodeStatus SelfLocateLine::tick()
{
    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // 静态下, 允许更大的距离
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/line/success";
    string logPathF = "/locate/line/fail";

    // 避免过于频繁定位
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->log(logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto fieldLines = brain->data->getFieldLines();
    // 计算每条线到机器人的距离
    double dist0;
    double dist1;
    std::vector<std::pair<double, FieldLine>> lineWithDist;
    for (const auto& line : fieldLines) {
        dist0 = norm(
            line.posToField.x0 - brain->data->robotPoseToField.x,
            line.posToField.y0 - brain->data->robotPoseToField.y
        );
        dist1 = norm(
            line.posToField.x1 - brain->data->robotPoseToField.x,
            line.posToField.y1 - brain->data->robotPoseToField.y
        );
        if (dist0 > maxDist || dist1 > maxDist) continue; // max_dist为2，只有线段两端距机器人的距离都在两米范围内的线才会被考虑，不合理
        double dist = norm(
            (line.posToField.x0+line.posToField.x1)/2 - brain->data->robotPoseToField.x,
            (line.posToField.y0+line.posToField.y1)/2 - brain->data->robotPoseToField.y
        );
        lineWithDist.emplace_back(dist, line);
    }
    // 按距离排序
    std::sort(lineWithDist.begin(), lineWithDist.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; }
    );
    // 排序后的 fieldLines
    std::vector<FieldLine> sortedFieldLines;
    for (const auto& p : lineWithDist) {
        sortedFieldLines.push_back(p.second);
    }

    // 计算校正量
    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    // 之后用 sortedFieldLines 即可
    

    std::vector<FieldLineRef> standardLines = {
        {"bottom", -fd.length/2, -fd.width/2, -fd.length/2, fd.width/2, false}, // 底线
        {"top", fd.length/2, -fd.width/2, fd.length/2, fd.width/2, false},      // 顶线
        {"left", -fd.length/2, -fd.width/2, fd.length/2, -fd.width/2, true},   // 左边线
        {"right", -fd.length/2, fd.width/2, fd.length/2, fd.width/2, true},    // 右边线
        {"middle", 0, -fd.width/2, 0, fd.width/2, false},                       // 中线
        {"top_penalty_left", fd.length / 2, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, true},
        {"top_penalty_right", fd.length / 2, fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2, true},
        {"top_penalty_middle", fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2, false},
        {"top_goal_left", fd.length / 2, -fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, true},
        {"top_goal_right", fd.length / 2, fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, true},
        {"top_goal_middle", (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, false},
        {"bottom_penalty_left", -fd.length / 2, -fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, true},
        {"bottom_penalty_right", -fd.length / 2, fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, true},
        {"bottom_penalty_middle", -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, false},
        {"bottom_goal_left", -fd.length / 2, -fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, true},
        {"bottom_goal_right", -fd.length / 2, fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, true},
        {"bottom_goal_middle",  -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, false},
    };
    
    // 计算两条线的距离和夹角
    auto lineDistanceAngle = [this](const FieldLine& obs, const FieldLineRef& def) -> std::pair<double, double> {
        double dist = this->lineToLineAvgDist(obs, def, 20); // 采样10个点

        // 计算方向夹角
        double angle = angleBetweenLines(obs.posToField, {def.x0, def.y0, def.x1, def.y1});
        return std::make_pair(dist, angle);
    };

    FieldLineRef bestLine;
    FieldLineRef secondBestLine;
    FieldLine firstLine;
    FieldLine secondLine;
    bool firstLineFound = false;
    bool secondLineFound = false;
    // 匹配每条观测到的线
    for (const auto& line : sortedFieldLines) {
        double minScore = 1e6;
        double angleThreshold = 0.2; // 约11度，可根据需求调整
        for (const auto& def : standardLines) {
            auto [dist, angle] = lineDistanceAngle(line, def);
            double score;
            if (angle < angleThreshold) {
                score = dist; // 距离和角度加权
            } else {
                score = 1e6; // 角度不对，不匹配
            }
            if (score < minScore) {
                minScore = score;
                if (!firstLineFound) {
                    bestLine = def;
                    firstLine = line;
                } else if (!secondLineFound) {
                    secondBestLine = def;
                    secondLine = line;
                }
            }
        }
        if (minScore < maxDrift) {
            if (!firstLineFound) {
                firstLineFound = true;
            } else if (!secondLineFound && bestLine.name != secondBestLine.name) {
                secondLineFound = true;
            }
        }
    }

    if (!firstLineFound) {
        log->log(logPathF, rerun::TextLog("Failed, no matching line found"));
        return NodeStatus::SUCCESS;
    }

    // 计算校正量
    // 根据线的方向性计算dx或dy 
    if (bestLine.isVertical) {
        // 竖线，调整 y
        double dy0 = bestLine.y0 - firstLine.posToField.y0;
        double dy1 = bestLine.y0 - firstLine.posToField.y1;
        dy = fabs(dy0) < fabs(dy1) ? dy0 : dy1;
    } else {
        // 水平线，调整 x
        double dx0 = bestLine.x0 - firstLine.posToField.x0;
        double dx1 = bestLine.x0 - firstLine.posToField.x1;
        dx = fabs(dx0) < fabs(dx1) ? dx0 : dx1;
    }

    // 如果找到第二条线，可以进一步校正 (必须方向不同才有用)
    if (secondLineFound && secondBestLine.isVertical != bestLine.isVertical) {
        if (secondBestLine.isVertical) {
            double dy0 = secondBestLine.y0 - secondLine.posToField.y0;
            double dy1 = secondBestLine.y0 - secondLine.posToField.y1;
            dy = fabs(dy0) < fabs(dy1) ? dy0 : dy1;
        } else {
            double dx0 = secondBestLine.x0 - secondLine.posToField.x0;
            double dx1 = secondBestLine.x0 - secondLine.posToField.x1;
            dx = fabs(dx0) < fabs(dx1) ? dx0 : dx1;
        }
    }


    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->log(logPathF, rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // 利用这个点计算一个假设的新 Pose
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    if (validate) {
        auto allMarkers = brain->data->getMarkersForLocator();
        if (allMarkers.size() > 0) {
            double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
            if (residual > brain->locator->residualTolerance) {
                log->log(logPathF,
                    rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
                );
                return NodeStatus::SUCCESS;
            }
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    brain->log->log(
        "field/recal/line/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({bestLine.name})
    );
    brain->calibrateOdom(hypoPose.x, hypoPose.y, hypoPose.theta);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}


NodeStatus MoveToPoseOnField::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/Move", rerun::TextLog(msg));
    };
    log("Move ticked");

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
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
    };
    log("GoToReadyPosition ticked");

    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    string role = brain->tree->getEntry<string>("player_role");
    const string gameState = brain->tree->getEntry<string>("gc_game_state");
    bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    auto fd = brain->config->fieldDimensions;

    // default values, override with different conditions
    double tx = 0, ty = 0, ttheta = 0; 
    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    if (brain->distToBorder() > - 1.0) { // near border
        vxLimit = 0.6;
        vyLimit = 0.4;
    }
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;

    if (_lastGameState != gameState && gameState == "READY") {
        _positionLocked = false;
        _lockedTargetValid = false;
    }
    _lastGameState = gameState;

    // READY 期间定位结果会有小幅抖动。只要机器人已经进入可接受范围，
    // 就锁住站位；只有明显偏离时才重新修正，避免在目标点附近来回走动。
    if (gameState != "READY") {
        _positionLocked = false;
        _lockedTargetValid = false;
    }

    if (role == "striker") {
        int strikerRank = brain->data->myStrikerIDRank;
        if (gameState == "READY") {
            strikerRank = 0;
            const int goalieId = brain->data->activeGoaliePlayerId > 0
                ? brain->data->activeGoaliePlayerId
                : brain->data->goalkeeperPlayerId;
            for (int id = 1; id < brain->config->playerId; ++id) {
                if (id != goalieId && id <= brain->config->numOfPlayers) ++strikerRank;
            }
            strikerRank = std::clamp(strikerRank, 0, 3);
        }
        if (strikerRank == 0) {
            tx = isKickoff ? - fd.circleRadius : - fd.circleRadius * 2;
            ty = 0.0;
        } else if (strikerRank == 1) {
            tx = isKickoff ? - fd.circleRadius : - fd.circleRadius * 2;
            ty = -1.5;
        } else if (strikerRank == 2) {
            //tx = - fd.length / 2.0 + fd.penaltyDist;
            //ty = fd.goalAreaWidth / 2.0;
            tx = - fd.length / 2.0 + fd.penaltyAreaLength;
            ty = fd.circleRadius;
        } else if (strikerRank == 3) {
            tx = - fd.length / 2.0 + fd.penaltyDist;
            ty = - fd.circleRadius - 1.0;
            //ty = - fd.goalAreaWidth / 2.0;
        }
    } else if (role == "goal_keeper") {
        // 与 PLAY 阶段 GoalieIdleHome 的门线前约 1m 深度保持一致。
        tx = -fd.length / 2.0 + fd.goalAreaLength / 2.0;
        ty = 0;
        ttheta = 0;
    }

    const Pose2D targetPose{tx, ty, ttheta};
    const auto robotPose = brain->data->robotPoseToField;
    const double targetDistance = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    const double targetThetaError = fabs(toPInPI(targetPose.theta - robotPose.theta));

    // 站位目标可能因角色/开球方变化而改变，目标改变时必须解除旧锁存。
    if (!_lockedTargetValid
        || norm(targetPose.x - _lockedTargetPose.x, targetPose.y - _lockedTargetPose.y) > 0.05
        || fabs(toPInPI(targetPose.theta - _lockedTargetPose.theta)) > 0.05) {
        _lockedTargetPose = targetPose;
        _lockedTargetValid = true;
        _positionLocked = false;
    }

    const double unlockDistance = std::max(distTolerance + 0.25, distTolerance * 1.5);
    const double unlockTheta = std::max(thetaTolerance + 0.10, thetaTolerance * 1.5);
    if (_positionLocked) {
        if (targetDistance <= unlockDistance && targetThetaError <= unlockTheta) {
            brain->client->setVelocity(0, 0, 0);
            return NodeStatus::SUCCESS;
        }
        _positionLocked = false;
    }

    if (targetDistance <= distTolerance && targetThetaError <= thetaTolerance) {
        _positionLocked = true;
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
    };
    log("GoBackInField ticked");

    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { // 没出界
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 出界了, 往回走
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
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

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}





NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 2) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && // 倒了就直接尝试RL起身，（不需要关注是否recoveryAailable）
        brain->data->currentRobotModeIndex == 1 && // is damping
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->data->shouldExitRLVisionKick = true;
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 10) { // recover
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog(format("Add retry count: %d", brain->data->recoveryPerformedRetryCount)));
    }

    // 机器人站着且是robocup步态，可以重置跌到爬起的状态
    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        (brain->data->currentRobotModeIndex == 8 || brain->data->currentRobotModeIndex == 20)) { // in robocup gait
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->data->shouldExitRLVisionKick = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}




NodeStatus RoleSwitchIfNeeded::tick()
{

    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RoleSwitchIfNeeded", rerun::TextLog(msg));
    };
    log("RoleSwitchIfNeeded ticked");

    int aliveCount = 0;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++)
    {
        if (brain->data->penalty[i] == PENALTY_NONE)
            aliveCount++;
    }
     
    string oldRole = brain->tree->getEntry<string>("player_role");
    string newRole = oldRole;
    /**
     * 策略是，只有满员的时候，才会有守门员，其他时候都参与进攻
     */
    if ((aliveCount < brain->config->numOfPlayers) && brain->data->penalty[brain->config->playerId - 1] == PENALTY_NONE)
    {
        brain->tree->setEntry<string>("player_role", "striker");
        newRole = "striker";
    }
    else if (aliveCount == brain->config->numOfPlayers - 1) {
        brain->tree->setEntry<string>("player_role", "goal_keeper");
        newRole = "goal_keeper";
    }
     
    if (brain->tree->getEntry<string>("gc_game_state") == "INITIAL") {
        brain->tree->setEntry<string>("player_role", brain->config->playerRole);
        newRole = brain->config->playerRole;
    }

    if (newRole != oldRole) {
        brain->speak("Switch to " + newRole);
    }
    // std::cout << "[MSG] " << brain->tree->getEntry<string>("player_role") << std::endl;

    return NodeStatus::SUCCESS;
}

/* ------------------------------------ 节点实现: 调试用 ------------------------------------*/

double AutoCalibrateVision::_calcResidual() {
    double res = 0;
    auto markers = brain->data->getMarkings();
    for (auto marker: markers) {
       double minDist = 100;
       for (auto mapMarker : brain->config->mapMarkings) {
           if (marker.label != mapMarker.type) continue;
               
           double dist = norm(marker.posToField.x - mapMarker.x, marker.posToField.y - mapMarker.y);
           if (dist < minDist) minDist = dist;
       }
       res += minDist;
    }
    if (markers.size() > 0) return res / markers.size();
    // else
    return 100;
}

NodeStatus AutoCalibrateVision::onStart()
{
    auto fd = brain->config->fieldDimensions;
    brain->calibrateOdom(-fd.length / 2.0 + fd.penaltyAreaLength, -fd.width / 2.0, M_PI / 2.0);

    _res = {};
    _index = -1;
    _paramChangeTime = brain->get_clock()->now();
    int steps = 10;

    string state = brain->tree->getEntry<string>("calibrate_state"); 
    if (state == "pitch") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_pitch_center");
            double step = brain->tree->getEntry<double>("calibrate_pitch_step");
            double p = center + step * (i - steps);
            double y = brain->tree->getEntry<double>("calibrate_yaw_center");
            double z = brain->tree->getEntry<double>("calibrate_z_center");
            _res.push_back({p, y, z, 0, 0});
        }
    } else if (state == "yaw") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_yaw_center");
            double step = brain->tree->getEntry<double>("calibrate_yaw_step");
            double p = brain->tree->getEntry<double>("calibrate_pitch_center");
            double y = center + step * (i - steps);
            double z = brain->tree->getEntry<double>("calibrate_z_center");
            _res.push_back({p, y, z, 0, 0});
        }
    } else if (state == "z") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_z_center");
            double step = brain->tree->getEntry<double>("calibrate_z_step");
            double p = brain->tree->getEntry<double>("calibrate_pitch_center");
            double y = brain->tree->getEntry<double>("calibrate_yaw_center");
            double z = center + step * (i - steps);
            _res.push_back({p, y, z, 0, 0});
        }
    } else {
        prtErr("Invalid calibrate state: " + state);
        return NodeStatus::SUCCESS;
    }
    cout << "Calibration started, pitch: " << brain->tree->getEntry<double>("calibrate_pitch_center") << ", yaw: " << brain->tree->getEntry<double>("calibrate_yaw_center") << ", z: " << brain->tree->getEntry<double>("calibrate_z_center") << endl;
    brain->speak(format("Calibration started"));

    return NodeStatus::RUNNING;
}

NodeStatus AutoCalibrateVision::onRunning()
{
    cout << "pitch_step: " << brain->tree->getEntry<double>("calibrate_pitch_step") << endl;
    //  cout << "Calibration running, pitch: " << brain->tree->getEntry<double>("calibrate_pitch_center") << ", yaw: " << brain->tree->getEntry<double>("calibrate_yaw_center") << ", z: " << brain->tree->getEntry<double>("calibrate_z_center") << endl;
    if (brain->tree->getEntry<double>("calibrate_pitch_step") < 0.1) { 
        // cout << RED_CODE << "should stop" << endl;
        prtDebug(
            format("Calibration finished, pitch: %.2f, yaw: %.2f, z: %.2f, best residual: %.2f",
            brain->tree->getEntry<double>("calibrate_pitch_center"),
            brain->tree->getEntry<double>("calibrate_yaw_center"),
            brain->tree->getEntry<double>("calibrate_z_center"),
            _bestResidual),
            GREEN_CODE
        );
        double pitch = brain->tree->getEntry<double>("calibrate_pitch_center"); 
        double yaw = brain->tree->getEntry<double>("calibrate_yaw_center");
        double z = brain->tree->getEntry<double>("calibrate_z_center");
        brain->pubCalParamMsg(pitch, yaw, z);
        brain->log->setTimeNow();
        brain->log->log(
            "/field/vision_param", 
            rerun::Points2D({{0, 5}})
            .with_labels({format("BEST: P: %.2f Y: %.2f Z: %.2f RES: %.2f", 
                pitch, yaw, z, _bestResidual)})
            .with_colors(0xFFFFFFFF)
        );
        return NodeStatus::SUCCESS;
    }
    
    double t = brain->msecsSince(_paramChangeTime);
    if (t > 100) { // 发新的参数
        _index++;
        // cout << "Calibration index: " << _index << " size: " << _res.size() << endl;
        if (_index >= _res.size()) { // 所有参数测试结束
            if (brain->tree->getEntry<string>("calibrate_state") == "pitch") {
                // 直接遍历找到最小误差值，但要求计算次数不为0
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];
                        minIndex = i;
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_pitch_center", _res[minIndex][0]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("找到最佳pitch值: %.2f，误差: %.4f，计算次数: %.0f", 
                              _res[minIndex][0], _res[minIndex][3], _res[minIndex][4]));
                }
                brain->tree->setEntry<double>("calibrate_pitch_step", brain->tree->getEntry<double>("calibrate_pitch_step") / 2.0);
                brain->tree->setEntry<string>("calibrate_state", "yaw");
            } else if (brain->tree->getEntry<string>("calibrate_state") == "yaw") {
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];    
                        minIndex = i;
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_yaw_center", _res[minIndex][1]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("找到最佳yaw值: %.2f，误差: %.4f，计算次数: %.0f", 
                              _res[minIndex][1], _res[minIndex][3], _res[minIndex][4]));
                }
                brain->tree->setEntry<double>("calibrate_yaw_step", brain->tree->getEntry<double>("calibrate_yaw_step") / 2.0); 
                brain->tree->setEntry<string>("calibrate_state", "z");
            } else if (brain->tree->getEntry<string>("calibrate_state") == "z") {
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];    
                        minIndex = i;        
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_z_center", _res[minIndex][2]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("找到最佳z值: %.2f，误差: %.4f，计算次数: %.0f", 
                              _res[minIndex][2], _res[minIndex][3], _res[minIndex][4]));
                }
                double z_step = brain->tree->getEntry<double>("calibrate_z_step");
                if (z_step > 0.005) 
                    brain->tree->setEntry<double>("calibrate_z_step", z_step / 2.0); 
                brain->tree->setEntry<string>("calibrate_state", "pitch");
            }
            return NodeStatus::SUCCESS;
        }

        // else
        brain->pubCalParamMsg(_res[_index][0], _res[_index][1], _res[_index][2]);
        _paramChangeTime = brain->get_clock()->now();
        _subTotal = 0;
        _count = 0;
        return NodeStatus::RUNNING;
    } else if (t < 50) {
        return NodeStatus::RUNNING; // wait for param to take effect
    }
    
    // else 
    double res = _calcResidual();
    cout << format(
        "Calibrate [%d/%d] P:\t%.2f\tY:\t%.2f\tZ:\t%.2f\tRES:\t%.2f\tCNT:\t%d", 
        _index + 1, _res.size(), _res[_index][0], _res[_index][1], _res[_index][2], res, brain->data->getMarkings().size()
    ) << endl;

    _subTotal += res;
    _count++;
    if (_count > 0) {
        _res[_index][3] = _subTotal / _count;
        _res[_index][4] = _count;  
    } 
    brain->log->setTimeNow();
    brain->log->log(
        "/field/vision_param", 
        rerun::Points2D({{0, 5}})
        .with_labels({format("P: %.2f Y: %.2f Z: %.2f RES: %.2f CNT: %.f", 
            _res[_index][0], _res[_index][1], _res[_index][2], _res[_index][3], _res[_index][4])})
        .with_colors(0xFFFFFFFF)
    );
    return NodeStatus::RUNNING;
}

NodeStatus CrabWalk::tick()
{
    double angle, speed;
    getInput("angle", angle);
    getInput("speed", speed);
    brain->client->crabWalk(angle, speed);
    return NodeStatus::SUCCESS;
}

NodeStatus CalibrateOdom::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->calibrateOdom(x, y, theta);
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

NodeStatus PlaySound::tick()
{
    string sound;
    getInput("sound", sound);
    bool allowRepeat;
    getInput("allow_repeat", allowRepeat);
    brain->playSound(sound, allowRepeat);
    return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}
