#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "locator.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "locator.h"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <ios>

/**
 * Here we use a macro definition to reduce the code for RegisterBuilder. The effect of REGISTER_BUILDER(Test) after expansion is
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
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
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(RobocupWalk)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RLVisionKick)
    REGISTER_BUILDER(Assist)

    // Register Locator related nodes
    brain->registerLocatorNodes(factory);

    // Action Nodes for debug
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->get_tree_file_path());
    tree = factory.createTree("MainTree");

    // After construction, initialize blackboard entries
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->get_player_role());
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_real_game_sub_state", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<bool>("local_freekick_use_custom", false);
    setEntry<string>("local_freekick_phase", "NONE");
    setEntry<bool>("local_freekick_target_valid", false);
    setEntry<double>("local_freekick_target_error", 999.0);
    setEntry<bool>("local_freekick_move_stable", false);
    setEntry<bool>("local_freekick_stop_retreat", false);
    setEntry<bool>("local_freekick_released_by_ball", false);
    setEntry<bool>("local_freekick_plan_valid", false);
    setEntry<string>("local_freekick_plan_stage", "none");
    setEntry<int>("local_freekick_plan_count", 0);
    setEntry<double>("local_freekick_plan_target_x", 0.0);
    setEntry<double>("local_freekick_plan_target_y", 0.0);
    setEntry<double>("local_freekick_plan_ball_x", 0.0);
    setEntry<double>("local_freekick_plan_ball_y", 0.0);
    setEntry<double>("local_freekick_plan_contact_radius", 0.30);
    setEntry<double>("local_freekick_plan_required_radius", 0.0);
    setEntry<double>("local_freekick_plan_path_min_ball_dist", 999.0);
    for (int i = 0; i < 6; ++i) {
        setEntry<double>("local_freekick_plan_x" + std::to_string(i), 0.0);
        setEntry<double>("local_freekick_plan_y" + std::to_string(i), 0.0);
    }

    setEntry<bool>("need_check_behind", false);

    setEntry<bool>("is_lead", true); 
    setEntry<string>("goalie_mode", "attack"); 

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // Automatic vision calibration related
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
    const double pixToleranceX = brain->config->cameraImageWidth * 3 / 10.; // If the pixel difference between the ball and the center of the field of view is less than this tolerance, it is considered to be at the center of the field of view.
    const double pixToleranceY = brain->config->cameraImageHeight * 3 / 10.;
    const double xCenter = brain->config->cameraImageWidth / 2;
    const double yCenter = brain->config->cameraImageHeight / 2; 


    bool iSeeBall = brain->data->ballDetected;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
        return NodeStatus::SUCCESS;

    if (!iSeeBall)
    { 
        if (iKnowBallPos) {
            // moving with smooth to last known ball position from vision
            pitch = brain->data->headPitch + (brain->data->ball.pitchToRobot - brain->data->headPitch) * 0.01;
            yaw = brain->data->headYaw + (brain->data->ball.yawToRobot - brain->data->headYaw) * 0.01;
        } else if (tmBallPosReliable) {
            pitch =  brain->data->headPitch + (brain->data->tmBall.pitchToRobot - brain->data->headPitch) * 0.01;
            yaw = brain->data->headYaw + (brain->data->tmBall.yawToRobot - brain->data->headYaw) * 0.01;
        } else {
            brain->log->error("CamTrackBall", "reached impossible condition");
        }
    }
    else {      
        ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
        ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter; 
        
        if (std::fabs(deltaX) < pixToleranceX && std::fabs(deltaY) < pixToleranceY)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 3.5;
        double deltaYaw = deltaX / brain->config->cameraImageWidth * brain->config->depthCameraFovX / smoother;
        double deltaPitch = deltaY / brain->config->cameraImageHeight * brain->config->depthCameraFovY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
    }

    brain->client->moveHead(pitch, yaw);
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
    _cmdIntervalMSec = 1000;
    _cmdRestartIntervalMSec = 60000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    } // Currently, all nodes return Success. Returning Failure would affect the execution of subsequent nodes.

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    } // Not yet time for the next command
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {                  // Exceeded a certain time, consider this as restarting from the beginning
        _cmdIndex = 0; // Note that we don't return here
    }
    else
    { // Reached the time, execute the next command, also do not return
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
    auto log = [=](string msg) {
        brain->log->debug("Chase4", msg);
    };
    log("ticked");
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle = brain->config->get_avoid_during_chase();
    double oaSafeDist = brain->config->get_chase_ao_safe_dist();

    if (
        brain->config->get_limit_near_ball_speed()
        && brain->data->ball.range < brain->config->get_near_ball_range()
    ) {
        vxLimit = min(brain->config->get_near_ball_speed_limit(), vxLimit);
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


    // Calculate target point
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
    } else {
        vx = min(vxLimit, brain->data->ball.range);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid((fabs(vtheta)), 1, 3); 
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.7 + vx * 0.3;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

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
    double vtheta = brain->data->ball.yawToRobot * 4.0; 

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -1.0);    
    vy = cap(vy, vyLimit, -vyLimit); 

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}


struct FreekickDefenseParams
{
    double exitMargin;
    double primaryBuffer;
    double goalKickPrimaryBuffer;
    double secondaryBuffer;
    double secondaryLateralOffset;
    double fieldMarginX;
    double fieldMarginY;
    double ownPenaltyExitMargin;
    double opponentGoalKickPenaltyExitMargin;
    double opponentGoalKickBallClearance;
    double arriveDistTolerance;
    double arriveThetaTolerance;
    double ballClearance;
    double pathBallClearance;
    double goalLineExceptionXMargin;
    double goalLineExceptionYMargin;
    double fastExitBackSpeed;
    double fastExitReleaseMargin;
    double fastExitTurnGain;
    double fastExitMaxLateral;
    double fastExitPathBallClearance;
    double boundaryArcSpeed;
    double boundaryArcRadiusMargin;
    double boundaryArcStep;
    double boundaryArcMinAngle;
    double boundaryArcDirectDist;
};

FreekickDefenseParams getFreekickDefenseParams(Brain *brain)
{
    FreekickDefenseParams p{};
    brain->get_parameter("strategy.freekick_defense.exit_margin", p.exitMargin);
    brain->get_parameter("strategy.freekick_defense.primary_buffer", p.primaryBuffer);
    brain->get_parameter("strategy.freekick_defense.goal_kick_primary_buffer", p.goalKickPrimaryBuffer);
    brain->get_parameter("strategy.freekick_defense.secondary_buffer", p.secondaryBuffer);
    brain->get_parameter("strategy.freekick_defense.secondary_lateral_offset", p.secondaryLateralOffset);
    brain->get_parameter("strategy.freekick_defense.field_margin_x", p.fieldMarginX);
    brain->get_parameter("strategy.freekick_defense.field_margin_y", p.fieldMarginY);
    brain->get_parameter("strategy.freekick_defense.own_penalty_exit_margin", p.ownPenaltyExitMargin);
    brain->get_parameter("strategy.freekick_defense.opponent_goal_kick_penalty_exit_margin", p.opponentGoalKickPenaltyExitMargin);
    brain->get_parameter("strategy.freekick_defense.opponent_goal_kick_ball_clearance", p.opponentGoalKickBallClearance);
    brain->get_parameter("strategy.freekick_defense.arrive_dist_tolerance", p.arriveDistTolerance);
    brain->get_parameter("strategy.freekick_defense.arrive_theta_tolerance", p.arriveThetaTolerance);
    brain->get_parameter("strategy.freekick_defense.ball_clearance", p.ballClearance);
    brain->get_parameter("strategy.freekick_defense.path_ball_clearance", p.pathBallClearance);
    brain->get_parameter("strategy.freekick_defense.goal_line_exception_x_margin", p.goalLineExceptionXMargin);
    brain->get_parameter("strategy.freekick_defense.goal_line_exception_y_margin", p.goalLineExceptionYMargin);
    brain->get_parameter("strategy.freekick_defense.fast_exit_back_speed", p.fastExitBackSpeed);
    brain->get_parameter("strategy.freekick_defense.fast_exit_release_margin", p.fastExitReleaseMargin);
    brain->get_parameter("strategy.freekick_defense.fast_exit_turn_gain", p.fastExitTurnGain);
    brain->get_parameter("strategy.freekick_defense.fast_exit_max_lateral", p.fastExitMaxLateral);
    brain->get_parameter("strategy.freekick_defense.fast_exit_path_ball_clearance", p.fastExitPathBallClearance);
    brain->get_parameter("strategy.freekick_defense.boundary_arc_speed", p.boundaryArcSpeed);
    brain->get_parameter("strategy.freekick_defense.boundary_arc_radius_margin", p.boundaryArcRadiusMargin);
    brain->get_parameter("strategy.freekick_defense.boundary_arc_step", p.boundaryArcStep);
    brain->get_parameter("strategy.freekick_defense.boundary_arc_min_angle", p.boundaryArcMinAngle);
    brain->get_parameter("strategy.freekick_defense.boundary_arc_direct_dist", p.boundaryArcDirectDist);
    return p;
}


NodeStatus GoToFreekickPosition::onStart() {
    _isInFinalAdjust = false;
    _defenseTargetLocked = false;
    _opponentGoalKickBallFrozen = false;
    int raw = brain->data->tmMyCostRank;
    _stableCostRank = raw;
    _costRankCandidate = raw;
    _costRankCandidateStartNs = brain->get_clock()->now().nanoseconds();
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    string side;
    getInput("side", side);
    if (side !="attack" && side != "defense") {
        brain->tree->setEntry<bool>("local_freekick_target_valid", false);
        brain->tree->setEntry<bool>("local_freekick_plan_valid", false);
        brain->tree->setEntry<int>("local_freekick_plan_count", 0);
        brain->data->localFreekickTargetUpdateTime = brain->get_clock()->now();
        _defenseTargetLocked = false;
        return NodeStatus::SUCCESS;
    }
    if (side != "defense") {
        brain->tree->setEntry<bool>("local_freekick_plan_valid", false);
        brain->tree->setEntry<int>("local_freekick_plan_count", 0);
        _defenseTargetLocked = false;
        _opponentGoalKickBallFrozen = false;
    }

    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    const auto liveBallPos = ballPos;
    auto robotPose = brain->data->robotPoseToField;
    const FreekickDefenseParams defenseParams = getFreekickDefenseParams(brain);
    const string realSubState = brain->data->realGameSubState;
    const bool opponentGoalKickDefense =
        side == "defense"
        && realSubState == "GOAL_KICK"
        && !brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    if (!opponentGoalKickDefense) {
        _opponentGoalKickBallFrozen = false;
    } else if (!_opponentGoalKickBallFrozen) {
        const bool liveBallReliable =
            brain->data->ballDetected
            || brain->tree->getEntry<bool>("ball_location_known")
            || brain->tree->getEntry<bool>("tm_ball_pos_reliable");
        const bool liveBallNearOpponentGoalArea =
            liveBallReliable
            &&
            std::isfinite(liveBallPos.x)
            && std::isfinite(liveBallPos.y)
            && liveBallPos.x > fd.length / 2.0 - fd.penaltyAreaLength - 0.45
            && fabs(liveBallPos.y) < fd.penaltyAreaWidth / 2.0 + 0.45;
        const double fallbackCornerX = fd.length / 2.0 - fd.goalAreaLength;
        double fallbackCornerY = 0.0;
        if (std::isfinite(liveBallPos.y) && fabs(liveBallPos.y) > 0.25) {
            fallbackCornerY = liveBallPos.y > 0.0
                ? fd.goalAreaWidth / 2.0
                : -fd.goalAreaWidth / 2.0;
        } else if (fabs(robotPose.y) > 0.25) {
            fallbackCornerY = robotPose.y > 0.0
                ? fd.goalAreaWidth / 2.0
                : -fd.goalAreaWidth / 2.0;
        } else {
            fallbackCornerY = (brain->config->get_player_id() % 2 == 0)
                ? fd.goalAreaWidth / 2.0
                : -fd.goalAreaWidth / 2.0;
        }
        _frozenOpponentGoalKickBallPos.x = liveBallNearOpponentGoalArea
            ? liveBallPos.x
            : fallbackCornerX;
        _frozenOpponentGoalKickBallPos.y = liveBallNearOpponentGoalArea ? liveBallPos.y : fallbackCornerY;
        _frozenOpponentGoalKickBallPos.z = 0.0;
        _opponentGoalKickBallFrozen = true;
    }
    if (opponentGoalKickDefense) {
        ballPos = _frozenOpponentGoalKickBallPos;
    }
    const double opponentPenaltyFrontX = fd.length / 2.0 - fd.penaltyAreaLength;
    const double opponentPenaltyExitMargin = max(0.0, defenseParams.opponentGoalKickPenaltyExitMargin);
    const double opponentPenaltyExitX = opponentPenaltyFrontX - opponentPenaltyExitMargin;
    const double opponentPenaltyExitY = fd.penaltyAreaWidth / 2.0 + opponentPenaltyExitMargin;
    const double selfGoalX = -fd.length / 2.0;
    const double defenseSideXMargin = max(0.12, defenseParams.fastExitReleaseMargin + 0.06);
    const double minDefenseFieldX = selfGoalX + defenseParams.fieldMarginX;
    const bool hasDefenseSideRoom = ballPos.x - defenseSideXMargin >= minDefenseFieldX;
    const double opponentGoalKickFieldGuardX = opponentGoalKickDefense
        ? min(max(0.05, fd.length / 2.0 - 0.20),
            max(0.55, defenseParams.fieldMarginX + max(0.18, defenseParams.fieldMarginX * 0.45)))
        : defenseParams.fieldMarginX;
    const double opponentGoalKickFieldGuardY = opponentGoalKickDefense
        ? min(max(0.05, fd.width / 2.0 - 0.20),
            max(0.85, defenseParams.fieldMarginY + max(0.22, defenseParams.fieldMarginY * 0.35)))
        : defenseParams.fieldMarginY;
    const double safeFieldMinX = -fd.length / 2.0 + opponentGoalKickFieldGuardX;
    const double safeFieldMaxX =  fd.length / 2.0 - opponentGoalKickFieldGuardX;
    const double safeFieldMinY = -fd.width / 2.0 + opponentGoalKickFieldGuardY;
    const double safeFieldMaxY =  fd.width / 2.0 - opponentGoalKickFieldGuardY;

    auto isInsideOpponentGoalKickPenalty = [&](const Pose2D &pose) {
        return opponentGoalKickDefense
            && pose.x > opponentPenaltyExitX
            && fabs(pose.y) < opponentPenaltyExitY;
    };

    const double opponentPenaltyRouteMarginX = 0.12;
    const double opponentPenaltyRouteMarginY = 0.12;
    auto isInsideOpponentGoalKickRoutePenalty = [&](const Pose2D &pose) {
        return opponentGoalKickDefense
            && pose.x > opponentPenaltyFrontX - opponentPenaltyRouteMarginX
            && fabs(pose.y) < fd.penaltyAreaWidth / 2.0 + opponentPenaltyRouteMarginY;
    };

    auto forceOutsideOpponentGoalKickPenalty = [&](Pose2D pose) {
        if (!isInsideOpponentGoalKickPenalty(pose)) return pose;

        const double exitNudge = 0.10;
        const double sideExitY = cap(opponentPenaltyExitY + exitNudge, safeFieldMaxY, 0.0);
        const bool sideExitAvailable = sideExitY >= opponentPenaltyExitY + 0.02;
        const double distToFrontExit = max(0.0, pose.x - opponentPenaltyExitX);
        const double distToSideExit = sideExitAvailable
            ? max(0.0, sideExitY - fabs(pose.y))
            : 1e9;

        if (distToSideExit + 0.15 < distToFrontExit) {
            pose.y = pose.y >= 0.0 ? sideExitY : -sideExitY;
        } else {
            pose.x = opponentPenaltyExitX - exitNudge;
        }

        pose.x = cap(pose.x, safeFieldMaxX, safeFieldMinX);
        pose.y = cap(pose.y, safeFieldMaxY, safeFieldMinY);
        pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
        return pose;
    };

    double kickDir = brain->data->kickDir;
    double defenseDir = atan2(ballPos.y, ballPos.x + fd.length / 2);

    // 直接用 cost rank 决定站位, 防震荡: rank 需持续稳定 COST_RANK_STABLE_MS 才切换
    int raw = brain->data->tmMyCostRank;
    int64_t nowNs = brain->get_clock()->now().nanoseconds();
    if (raw != _costRankCandidate) {
        _costRankCandidate = raw;
        _costRankCandidateStartNs = nowNs;
    }
    if (nowNs - _costRankCandidateStartNs >= COST_RANK_STABLE_MS * 1000000LL) {
        _stableCostRank = _costRankCandidate;
    }
    int rank = _stableCostRank;
    if (side == "attack") {
        brain->data->freekickOffenseKickerActive = (rank == 0);
    }

    if (side == "attack") {
       double dist;
       getInput("attack_dist", dist);

       if (rank == 0) {
        targetPose.x = ballPos.x - dist * cos(kickDir);
        targetPose.y = ballPos.y - dist * sin(kickDir);
        targetPose.theta = kickDir;
       } else if (rank == 1) {
        targetPose.x = ballPos.x - 2.0 * cos(defenseDir);
        targetPose.y = ballPos.y - 2.0 * sin(defenseDir);
        targetPose.theta = defenseDir;
        } else if (rank == 2) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = fd.goalAreaWidth / 2.0;
        } else {  // rank >= 3
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = - fd.goalAreaWidth / 2.0;
        }
        brain->log->log("debug/freekick_rank", format("rank: %d, targetPose: (%.2f, %.2f, %.2f)", rank, targetPose.x, targetPose.y, targetPose.theta));
    } else if (side == "defense") {
        const double ownPenaltyFrontX = selfGoalX + fd.penaltyAreaLength;
        const double opponentGoalKickSafeRadius = max(
            0.60,
            max(defenseParams.opponentGoalKickBallClearance, defenseParams.ballClearance));
        const double requiredBallDist = opponentGoalKickDefense
            ? opponentGoalKickSafeRadius
            : max(fd.circleRadius + defenseParams.exitMargin, defenseParams.ballClearance);
        const double minPrimaryBuffer = max(0.12, defenseParams.fastExitReleaseMargin + 0.04);
        const double primaryBuffer = realSubState == "GOAL_KICK"
            ? max(defenseParams.goalKickPrimaryBuffer, minPrimaryBuffer)
            : max(defenseParams.primaryBuffer, minPrimaryBuffer);
        const double secondaryBuffer = max(defenseParams.secondaryBuffer, primaryBuffer + 0.22);
        const double primaryTargetDist = requiredBallDist + primaryBuffer;
        const double secondaryTargetDist = requiredBallDist + secondaryBuffer;

        const double goalDx = selfGoalX - ballPos.x;
        const double goalDy = -ballPos.y;
        const double goalNorm = norm(goalDx, goalDy);
        const double dirToGoalX = goalNorm > 1e-6 ? goalDx / goalNorm : -1.0;
        const double dirToGoalY = goalNorm > 1e-6 ? goalDy / goalNorm : 0.0;
        const double tangentX = -dirToGoalY;
        const double tangentY = dirToGoalX;
        const double robotBallDist = norm(robotPose.x - ballPos.x, robotPose.y - ballPos.y);
        const bool goalLineException =
            fabs(robotPose.x - selfGoalX) < defenseParams.goalLineExceptionXMargin
            && fabs(robotPose.y) < fd.goalWidth / 2.0 + defenseParams.goalLineExceptionYMargin;
        auto clampDefenseTarget = [&](Pose2D pose) {
            const double maxFieldX = safeFieldMaxX;
            const double minFieldX = opponentGoalKickDefense ? safeFieldMinX : minDefenseFieldX;
            const double minFieldY = safeFieldMinY;
            const double maxFieldY = safeFieldMaxY;
            pose.x = cap(pose.x, maxFieldX, minFieldX);
            pose.y = cap(pose.y, maxFieldY, minFieldY);

            const bool ballInOwnPenaltyArea =
                ballPos.x < ownPenaltyFrontX
                && fabs(ballPos.y) < fd.penaltyAreaWidth / 2.0;
            if (ballInOwnPenaltyArea
                && pose.x < ownPenaltyFrontX + defenseParams.ownPenaltyExitMargin
                && fabs(pose.y) < fd.penaltyAreaWidth / 2.0 + defenseParams.ownPenaltyExitMargin) {
                const double frontExitX = ownPenaltyFrontX + defenseParams.ownPenaltyExitMargin;
                if (hasDefenseSideRoom && frontExitX > ballPos.x - defenseSideXMargin) {
                    const double sideExitY = fd.penaltyAreaWidth / 2.0 + defenseParams.ownPenaltyExitMargin;
                    const double sideSign = fabs(pose.y) > 0.10
                        ? (pose.y > 0.0 ? 1.0 : -1.0)
                        : (ballPos.y >= 0.0 ? 1.0 : -1.0);
                    pose.x = min(pose.x, ballPos.x - defenseSideXMargin);
                    pose.y = sideSign > 0.0
                        ? cap(sideExitY, maxFieldY, 0.0)
                        : cap(-sideExitY, 0.0, minFieldY);
                } else {
                    pose.x = frontExitX;
                }
            }

            if (hasDefenseSideRoom) {
                pose.x = min(pose.x, ballPos.x - defenseSideXMargin);
            }
            pose.x = cap(pose.x, maxFieldX, minFieldX);
            pose.y = cap(pose.y, maxFieldY, minFieldY);
            pose = forceOutsideOpponentGoalKickPenalty(pose);
            if (hasDefenseSideRoom) {
                pose.x = min(pose.x, ballPos.x - defenseSideXMargin);
            }
            pose.x = cap(pose.x, maxFieldX, minFieldX);
            pose.y = cap(pose.y, maxFieldY, minFieldY);
            pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
            return pose;
        };

        auto makeLineTarget = [&](double ballDist, double lateralOffset) {
            Pose2D pose;
            pose.x = ballPos.x + dirToGoalX * ballDist + tangentX * lateralOffset;
            pose.y = ballPos.y + dirToGoalY * ballDist + tangentY * lateralOffset;
            pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
            return clampDefenseTarget(pose);
        };

        auto lateralSignForRank = [&](int rankIndex) {
            double baseSign;
            if (fabs(ballPos.y) > 0.20) {
                baseSign = ballPos.y > 0.0 ? -1.0 : 1.0;
            } else {
                baseSign = (brain->config->get_player_id() % 2 == 0) ? 1.0 : -1.0;
            }
            return (rankIndex % 2 == 1) ? baseSign : -baseSign;
        };

        auto makeOpponentGoalKickLineTarget = [&](int rankIndex, const Pose2D &fallbackPose) {
            const double minX = safeFieldMinX;
            const double maxX = safeFieldMaxX;
            const double minY = safeFieldMinY;
            const double maxY = safeFieldMaxY;
            const double exitNudge = 0.05;
            const double baseDist = requiredBallDist + 0.03;
            const int spreadRank = max(0, rankIndex);
            const double lateralSign = lateralSignForRank(max(1, spreadRank));
            const double lateralOffset = spreadRank == 0
                ? 0.0
                : lateralSign * min(1.05, max(0.45, defenseParams.secondaryLateralOffset)
                    + 0.18 * static_cast<double>((spreadRank - 1) / 2));
            const double rankDistOffset = spreadRank == 0
                ? 0.0
                : 0.24 * static_cast<double>(spreadRank - 1);

            auto poseAt = [&](double distance) {
                Pose2D pose;
                pose.x = ballPos.x + dirToGoalX * distance + tangentX * lateralOffset;
                pose.y = ballPos.y + dirToGoalY * distance + tangentY * lateralOffset;
                pose.x = cap(pose.x, maxX, minX);
                pose.y = cap(pose.y, maxY, minY);
                pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
                return pose;
            };

            const double exitLineX = cap(opponentPenaltyExitX, maxX, minX);
            double startDist = baseDist + rankDistOffset;
            if (fabs(dirToGoalX) > 1e-6) {
                const double distToExitLine =
                    (exitLineX - ballPos.x - tangentX * lateralOffset) / dirToGoalX;
                if (std::isfinite(distToExitLine) && distToExitLine > 0.0) {
                    startDist = max(startDist, distToExitLine);
                }
            }

            Pose2D best = poseAt(startDist);
            bool found = false;
            const double maxSearchDist = fd.length + fd.width;
            for (double distAlongLine = startDist;
                 distAlongLine <= maxSearchDist;
                 distAlongLine += 0.10) {
                Pose2D candidate = poseAt(distAlongLine);
                if (isInsideOpponentGoalKickPenalty(candidate)) continue;
                if (norm(candidate.x - ballPos.x, candidate.y - ballPos.y) < baseDist) continue;
                best = candidate;
                found = true;
                break;
            }

            if (!found) {
                best = forceOutsideOpponentGoalKickPenalty(poseAt(baseDist + rankDistOffset));
            }
            if (isInsideOpponentGoalKickPenalty(best)) {
                best.x = min(best.x, cap(opponentPenaltyExitX - exitNudge, maxX, minX));
            }
            best.x = cap(best.x, maxX, minX);
            best.y = cap(best.y, maxY, minY);
            best.theta = atan2(ballPos.y - best.y, ballPos.x - best.x);
            if (!std::isfinite(best.x) || !std::isfinite(best.y)) {
                best = forceOutsideOpponentGoalKickPenalty(fallbackPose);
                best.theta = atan2(ballPos.y - best.y, ballPos.x - best.x);
            }
            return best;
        };

        if (rank == 0) {
            targetPose = makeLineTarget(primaryTargetDist, 0.0);
        } else {
            const int spreadRank = max(1, rank);
            const double lateralSign = lateralSignForRank(spreadRank);
            const bool lineDefenseSetPlay = realSubState == "THROW_IN" || realSubState == "CORNER_KICK";
            const double lateralScale = lineDefenseSetPlay
                ? (0.45 + 0.15 * static_cast<double>((spreadRank - 1) / 2))
                : (1.0 + 0.25 * static_cast<double>((spreadRank - 1) / 2));
            const double lateralBase = lineDefenseSetPlay
                ? max(0.20, defenseParams.secondaryLateralOffset)
                : max(0.35, defenseParams.secondaryLateralOffset);
            const double maxLateral = lineDefenseSetPlay ? 0.55 : 1.25;
            const double lateralOffset = lateralSign * min(maxLateral, lateralBase * lateralScale);
            const double rankDistOffset = (lineDefenseSetPlay ? 0.14 : 0.22) * static_cast<double>(spreadRank - 1);
            targetPose = makeLineTarget(
                secondaryTargetDist + rankDistOffset,
                lateralOffset);
        }

        if (opponentGoalKickDefense) {
            targetPose = makeOpponentGoalKickLineTarget(rank, targetPose);
        }
    }

    if (side == "defense") {
        const double ballMoveForReplan = opponentGoalKickDefense ? 0.32 : 0.24;
        const bool defenseTargetContextChanged =
            !_defenseTargetLocked
            || _lockedDefenseSubState != realSubState
            || _lockedDefenseOpponentGoalKick != opponentGoalKickDefense
            || (!opponentGoalKickDefense && _lockedDefenseRank != rank);
        const double lockedBallMove = _defenseTargetLocked
            ? norm(ballPos.x - _lockedDefenseBallPos.x, ballPos.y - _lockedDefenseBallPos.y)
            : 999.0;
        const bool shouldRelockDefenseTarget =
            defenseTargetContextChanged
            || (!opponentGoalKickDefense && lockedBallMove > ballMoveForReplan)
            || (opponentGoalKickDefense
                && ( _lockedDefenseTarget.x < safeFieldMinX
                    || _lockedDefenseTarget.x > safeFieldMaxX
                    || _lockedDefenseTarget.y < safeFieldMinY
                    || _lockedDefenseTarget.y > safeFieldMaxY))
            || isInsideOpponentGoalKickPenalty(_lockedDefenseTarget);

        if (shouldRelockDefenseTarget) {
            _lockedDefenseTarget = targetPose;
            _lockedDefenseBallPos = ballPos;
            _lockedDefenseSubState = realSubState;
            _lockedDefenseOpponentGoalKick = opponentGoalKickDefense;
            _lockedDefenseRank = rank;
            _defenseTargetLocked = true;
        } else {
            ballPos = _lockedDefenseBallPos;
            targetPose = _lockedDefenseTarget;
            targetPose.theta = atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);
        }
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);
    brain->tree->setEntry<bool>("local_freekick_target_valid", true);
    brain->tree->setEntry<double>("local_freekick_target_error", dist);
    brain->data->localFreekickTargetUpdateTime = brain->get_clock()->now();

    if (side == "defense") {
        auto poseOnDefenseSide = [&](const Pose2D &pose) {
            return !hasDefenseSideRoom || pose.x <= ballPos.x - defenseSideXMargin;
        };

        if (brain->tree->getEntry<bool>("local_freekick_released_by_ball")) {
            brain->client->setVelocity(0, 0, 0, false, false, false);
            brain->tree->setEntry<bool>("local_freekick_plan_valid", false);
            brain->tree->setEntry<string>("local_freekick_plan_stage", "released_by_ball");
            brain->tree->setEntry<int>("local_freekick_plan_count", 0);
            return NodeStatus::SUCCESS;
        }

        const bool ballInOwnHalf = ballPos.x < 0.0;
        Pose2D planBallPose;
        planBallPose.x = ballPos.x;
        planBallPose.y = ballPos.y;
        planBallPose.theta = 0.0;
        const Pose2D planBallRobot = brain->data->field2robot(planBallPose);
        const double planBallYawToRobot = atan2(planBallRobot.y, planBallRobot.x);
        const double motionBallYawToRobot =
            opponentGoalKickDefense && std::isfinite(planBallYawToRobot)
                ? planBallYawToRobot
                : brain->data->ball.yawToRobot;
        const double thetaError = opponentGoalKickDefense
            ? fabs(deltaDir)
            : fabs(motionBallYawToRobot);
        auto commandOpponentGoalKickLookAtBall = [&]() {
            if (!opponentGoalKickDefense) return;

            Pose2D lookBallPose;
            const bool useLiveLookBall =
                brain->data->ballDetected
                && std::isfinite(liveBallPos.x)
                && std::isfinite(liveBallPos.y);
            lookBallPose.x = useLiveLookBall ? liveBallPos.x : ballPos.x;
            lookBallPose.y = useLiveLookBall ? liveBallPos.y : ballPos.y;
            lookBallPose.theta = 0.0;
            Pose2D ballRobot = brain->data->field2robot(lookBallPose);
            const double ballRange = norm(ballRobot.x, ballRobot.y);
            const double yaw = useLiveLookBall && std::isfinite(brain->data->ball.yawToRobot)
                ? brain->data->ball.yawToRobot
                : atan2(ballRobot.y, ballRobot.x);
            const double pitch = useLiveLookBall && std::isfinite(brain->data->ball.pitchToRobot)
                ? brain->data->ball.pitchToRobot
                : atan2(brain->config->get_robot_height(), max(0.10, ballRange));
            if (std::isfinite(yaw) && std::isfinite(pitch)) {
                brain->client->moveHead(pitch, yaw);
            }
        };
        commandOpponentGoalKickLookAtBall();

        if (!isInsideOpponentGoalKickPenalty(robotPose)
            && poseOnDefenseSide(robotPose)
            && dist < defenseParams.arriveDistTolerance
            && thetaError < defenseParams.arriveThetaTolerance) {
            brain->client->setVelocity(0, 0, 0);
            brain->tree->setEntry<bool>("local_freekick_plan_valid", false);
            brain->tree->setEntry<string>("local_freekick_plan_stage", "arrived");
            brain->tree->setEntry<int>("local_freekick_plan_count", 0);
            return NodeStatus::SUCCESS;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);

        const double opponentGoalKickSafeRadius = max(
            0.60,
            max(defenseParams.opponentGoalKickBallClearance, defenseParams.ballClearance));
        const double requiredBallDist = opponentGoalKickDefense
            ? opponentGoalKickSafeRadius
            : max(fd.circleRadius + defenseParams.exitMargin, defenseParams.ballClearance);
        const double robotBallDist = norm(robotPose.x - ballPos.x, robotPose.y - ballPos.y);
        const double contactClearance = opponentGoalKickDefense
            ? opponentGoalKickSafeRadius
            : max(0.30, defenseParams.ballClearance);
        const bool goalLineException =
            fabs(robotPose.x - selfGoalX) < defenseParams.goalLineExceptionXMargin
            && fabs(robotPose.y) < fd.goalWidth / 2.0 + defenseParams.goalLineExceptionYMargin;
        const bool robotOnDefenseSide = poseOnDefenseSide(robotPose);
        // Keep enter-field mode until the robot reaches the same safe in-field
        // margin used by the generated entry targets. Releasing earlier can make
        // a partly out-of-field robot switch to ball-radius retreat and step out.
        const bool robotInsideField =
            robotPose.x >= safeFieldMinX
            && robotPose.x <= safeFieldMaxX
            && robotPose.y >= safeFieldMinY
            && robotPose.y <= safeFieldMaxY;
        const bool mustFastExit =
            !goalLineException
            && robotBallDist < requiredBallDist + defenseParams.fastExitReleaseMargin;
        const double opponentGoalSideRiskMargin = max(0.22, defenseSideXMargin);
        const bool robotFacingOwnGoal =
            fabs(toPInPI(robotPose.theta - M_PI)) < deg2rad(65.0);
        const bool opponentGoalSideOrbitRisk =
            opponentGoalKickDefense
            && robotPose.x > ballPos.x + opponentGoalSideRiskMargin
            && (robotPose.x > opponentPenaltyFrontX - 0.25 || robotFacingOwnGoal);
        const double opponentGoalKickFastSpeed =
            max(1.15, max(defenseParams.boundaryArcSpeed, defenseParams.fastExitBackSpeed));
        const double opponentGoalKickNoTouchRadius = max(requiredBallDist, contactClearance);
        const double opponentGoalKickPathClearance = max(requiredBallDist, contactClearance);
        const double opponentGoalKickWidenReleaseClearance = max(requiredBallDist, contactClearance);
        const bool liveGoalKickBallForSafety =
            opponentGoalKickDefense
            && brain->data->ballDetected
            && std::isfinite(liveBallPos.x)
            && std::isfinite(liveBallPos.y)
            && liveBallPos.x > fd.length / 2.0 - fd.penaltyAreaLength - 0.60
            && fabs(liveBallPos.y) < fd.penaltyAreaWidth / 2.0 + 0.60;
        const double liveRobotBallDist = liveGoalKickBallForSafety
            ? norm(robotPose.x - liveBallPos.x, robotPose.y - liveBallPos.y)
            : 999.0;
        const double activeRobotBallDist = min(robotBallDist, liveRobotBallDist);
        auto activeSafetyBallPose = [&]() {
            Pose2D safetyBall;
            safetyBall.x = ballPos.x;
            safetyBall.y = ballPos.y;
            safetyBall.theta = 0.0;
            if (liveGoalKickBallForSafety && liveRobotBallDist < robotBallDist) {
                safetyBall.x = liveBallPos.x;
                safetyBall.y = liveBallPos.y;
            }
            return safetyBall;
        };
        auto activeSafetyBallYaw = [&]() {
            if (liveGoalKickBallForSafety
                && liveRobotBallDist < robotBallDist
                && std::isfinite(brain->data->ball.yawToRobot)) {
                return brain->data->ball.yawToRobot;
            }
            return motionBallYawToRobot;
        };
        auto opponentGoalKickSafetyBalls = [&]() {
            vector<Pose2D> balls;
            Pose2D frozenBall;
            frozenBall.x = ballPos.x;
            frozenBall.y = ballPos.y;
            frozenBall.theta = 0.0;
            balls.push_back(frozenBall);

            if (liveGoalKickBallForSafety
                && norm(liveBallPos.x - ballPos.x, liveBallPos.y - ballPos.y) > 0.03) {
                Pose2D liveBall;
                liveBall.x = liveBallPos.x;
                liveBall.y = liveBallPos.y;
                liveBall.theta = 0.0;
                balls.push_back(liveBall);
            }
            return balls;
        };
        auto opponentGoalKickRetreatSpeed = [&](double pathMinBallDist) {
            const double safeClearance = min(activeRobotBallDist, pathMinBallDist) - contactClearance;
            const double speedRatio = std::clamp(safeClearance / 0.45, 0.0, 1.0);
            const double sprintSpeed = max(opponentGoalKickFastSpeed, min(1.30, brain->config->get_vx_limit()));
            return opponentGoalKickFastSpeed + (sprintSpeed - opponentGoalKickFastSpeed) * speedRatio;
        };
        auto opponentGoalKickNearestPostSign = [&]() {
            const double upperPostY = fd.goalWidth / 2.0;
            const double lowerPostY = -fd.goalWidth / 2.0;
            if (fabs(ballPos.y - upperPostY) < fabs(ballPos.y - lowerPostY)) return 1.0;
            if (fabs(ballPos.y - lowerPostY) < fabs(ballPos.y - upperPostY)) return -1.0;
            return ballPos.y >= 0.0 ? 1.0 : -1.0;
        };
        auto isInsideOpponentGoalKickInnerLane = [&](const Pose2D &pose) {
            if (!opponentGoalKickDefense || !isInsideOpponentGoalKickPenalty(pose)) return false;

            const double postSign = opponentGoalKickNearestPostSign();
            const double postX = fd.length / 2.0;
            const double postY = postSign * fd.goalWidth / 2.0;
            const double dx = postX - ballPos.x;
            double lineY = ballPos.y;
            if (fabs(dx) > 1e-5) {
                const double t = (pose.x - ballPos.x) / dx;
                lineY = ballPos.y + t * (postY - ballPos.y);
            } else {
                lineY = postY;
            }

            const double innerHalfWidth = max(fd.goalWidth / 2.0, fabs(lineY));
            const double margin = 0.05;
            return fabs(pose.y) <= innerHalfWidth + margin;
        };
        auto opponentGoalKickOuterSideSign = [&]() {
            return opponentGoalKickNearestPostSign();
        };
        auto opponentGoalAwayOrbitSign = [&]() {
            const double halfWidth = fd.width / 2.0;
            const double edgeGuardY = max(0.30, defenseParams.fieldMarginY + 0.15);
            if (robotPose.y > halfWidth - edgeGuardY || ballPos.y > halfWidth - edgeGuardY) return -1.0;
            if (robotPose.y < -halfWidth + edgeGuardY || ballPos.y < -halfWidth + edgeGuardY) return 1.0;

            if (opponentGoalKickDefense
                && isInsideOpponentGoalKickRoutePenalty(robotPose)
                && !isInsideOpponentGoalKickInnerLane(robotPose)) {
                return opponentGoalKickOuterSideSign();
            }

            if (fabs(robotPose.y) > 0.18) return robotPose.y > 0.0 ? 1.0 : -1.0;
            if (fabs(ballPos.y) > 0.18) return ballPos.y > 0.0 ? 1.0 : -1.0;

            const double relY = robotPose.y - ballPos.y;
            if (fabs(relY) > 0.08) return relY > 0.0 ? 1.0 : -1.0;

            return (brain->config->get_player_id() % 2 == 0) ? 1.0 : -1.0;
        };

        auto setFreekickPlan = [&](const string &stage, const vector<Pose2D> &points, const Pose2D &planTarget, double pathMinDist) {
            const int maxPlanPoints = 6;
            const int pointCount = min(static_cast<int>(points.size()), maxPlanPoints);
            brain->tree->setEntry<bool>("local_freekick_plan_valid", true);
            brain->tree->setEntry<string>("local_freekick_plan_stage", stage);
            brain->tree->setEntry<int>("local_freekick_plan_count", pointCount);
            brain->tree->setEntry<double>("local_freekick_plan_target_x", planTarget.x);
            brain->tree->setEntry<double>("local_freekick_plan_target_y", planTarget.y);
            brain->tree->setEntry<double>("local_freekick_plan_ball_x", ballPos.x);
            brain->tree->setEntry<double>("local_freekick_plan_ball_y", ballPos.y);
            brain->tree->setEntry<double>("local_freekick_plan_contact_radius", contactClearance);
            brain->tree->setEntry<double>("local_freekick_plan_required_radius", requiredBallDist);
            brain->tree->setEntry<double>("local_freekick_plan_path_min_ball_dist", pathMinDist);
            for (int i = 0; i < maxPlanPoints; ++i) {
                const double x = i < pointCount ? points[i].x : 0.0;
                const double y = i < pointCount ? points[i].y : 0.0;
                brain->tree->setEntry<double>("local_freekick_plan_x" + std::to_string(i), x);
                brain->tree->setEntry<double>("local_freekick_plan_y" + std::to_string(i), y);
            }
        };

        auto insideField = [&](const Pose2D &pose) {
            return pose.x >= safeFieldMinX
                && pose.x <= safeFieldMaxX
                && pose.y >= safeFieldMinY
                && pose.y <= safeFieldMaxY;
        };

        auto poseBallDist = [&](const Pose2D &pose) {
            double minDist = norm(pose.x - ballPos.x, pose.y - ballPos.y);
            if (liveGoalKickBallForSafety) {
                minDist = min(minDist, norm(pose.x - liveBallPos.x, pose.y - liveBallPos.y));
            }
            return minDist;
        };

        auto poseLegal = [&](const Pose2D &pose, double minBallDist) {
            return insideField(pose)
                && poseBallDist(pose) >= minBallDist
                && !isInsideOpponentGoalKickPenalty(pose);
        };

        auto obstacleClearTo = [&](const Pose2D &pose) {
            Pose2D poseRobot = brain->data->field2robot(pose);
            double targetRange = norm(poseRobot.x, poseRobot.y);
            if (targetRange < 0.05) return true;
            double targetYaw = atan2(poseRobot.y, poseRobot.x);
            double checkDist = min(targetRange + 0.20, max(0.90, defenseParams.pathBallClearance + 0.25));
            return brain->distToObstacle(targetYaw) > checkDist;
        };

        auto clampToField = [&](Pose2D pose) {
            pose.x = cap(pose.x, safeFieldMaxX, safeFieldMinX);
            pose.y = cap(pose.y, safeFieldMaxY, safeFieldMinY);
            pose = forceOutsideOpponentGoalKickPenalty(pose);
            pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
            return pose;
        };

        auto goalLineFallbackPose = [&]() {
            Pose2D pose;
            const double goalHalfY = max(0.10, fd.goalWidth / 2.0 - 0.18);
            const double lineInset = cap(defenseParams.goalLineExceptionXMargin * 0.5, 0.10, 0.04);
            const double lineX = selfGoalX + lineInset;
            const double ballSideSign = ballPos.y >= 0.0 ? -1.0 : 1.0;

            pose.x = lineX;
            pose.y = cap(ballPos.y, goalHalfY, -goalHalfY);

            if (rank > 0) {
                Pose2D frontPose;
                frontPose.x = selfGoalX + max(0.25, min(0.55, defenseParams.fieldMarginX + 0.15));
                frontPose.y = cap(ballPos.y + ballSideSign * 0.35, goalHalfY, -goalHalfY);
                if (poseBallDist(frontPose) >= requiredBallDist - 0.05) {
                    pose = frontPose;
                }
            }

            pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
            return pose;
        };

        auto bestCirclePose = [&](double centerAngle, double radius, const Pose2D &preferPose) {
            const double minBallDist = max(requiredBallDist + 0.03, contactClearance);
            vector<double> offsets = {
                0.0, 0.20, -0.20, 0.40, -0.40, 0.65, -0.65,
                0.95, -0.95, 1.30, -1.30, 1.70, -1.70, M_PI, -M_PI
            };
            Pose2D bestAny = clampToField(preferPose);
            Pose2D bestClear = bestAny;
            double bestAnyCost = 1e9;
            double bestClearCost = 1e9;
            bool hasAny = false;
            bool hasClear = false;
            for (double offset : offsets) {
                double angle = toPInPI(centerAngle + offset);
                Pose2D pose;
                pose.x = ballPos.x + radius * cos(angle);
                pose.y = ballPos.y + radius * sin(angle);
                pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
                if (!poseLegal(pose, minBallDist)) continue;

                double cost =
                    norm(pose.x - robotPose.x, pose.y - robotPose.y)
                    + 0.45 * norm(pose.x - preferPose.x, pose.y - preferPose.y)
                    + 0.10 * fabs(offset);
                if (!poseOnDefenseSide(pose)) {
                    cost += 8.0 + 4.0 * max(0.0, pose.x - (ballPos.x - defenseSideXMargin));
                }
                if (opponentGoalSideOrbitRisk) {
                    const double safeSign = opponentGoalAwayOrbitSign();
                    if (fabs(offset) > 1e-6 && offset * safeSign < 0.0) {
                        cost += 6.0 + 1.5 * fabs(offset);
                    }
                    const double maxAllowedGoalSideX = robotPose.x - 0.03;
                    if (pose.x > maxAllowedGoalSideX) {
                        cost += 14.0 + 8.0 * (pose.x - maxAllowedGoalSideX);
                    }
                }
                if (cost < bestAnyCost) {
                    bestAny = pose;
                    bestAnyCost = cost;
                    hasAny = true;
                }
                if (obstacleClearTo(pose) && cost < bestClearCost) {
                    bestClear = pose;
                    bestClearCost = cost;
                    hasClear = true;
                }
            }
            const bool ballNearOwnGoal =
                ballPos.x < selfGoalX + requiredBallDist + defenseParams.fieldMarginX
                && fabs(ballPos.y) < fd.goalWidth / 2.0 + requiredBallDist;
            if (ballNearOwnGoal) {
                Pose2D fallbackPose = goalLineFallbackPose();
                double fallbackCost =
                    norm(fallbackPose.x - robotPose.x, fallbackPose.y - robotPose.y)
                    + 0.35 * norm(fallbackPose.x - preferPose.x, fallbackPose.y - preferPose.y);
                if (!hasClear || fallbackCost < bestClearCost + 0.25) {
                    return fallbackPose;
                }
            }
            if (hasClear) return bestClear;
            if (hasAny) return bestAny;
            return bestAny;
        };

        auto segmentMinDistToBall = [&](const Pose2D &from, const Pose2D &to) {
            Line path = {from.x, from.y, to.x, to.y};
            double minDist = pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path);
            if (liveGoalKickBallForSafety) {
                minDist = min(minDist, pointMinDistToLine(Point2D({liveBallPos.x, liveBallPos.y}), path));
            }
            return minDist;
        };

        auto pathMinDistToBall = [&](const Pose2D &pose) {
            return segmentMinDistToBall(robotPose, pose);
        };

        auto planMinDistToBall = [&](const vector<Pose2D> &points) {
            if (points.empty()) return 999.0;
            double minDist = poseBallDist(points.front());
            for (size_t i = 1; i < points.size(); ++i) {
                minDist = min(minDist, segmentMinDistToBall(points[i - 1], points[i]));
            }
            return minDist;
        };

        auto pathKeepsBallClear = [&](const Pose2D &pose, double clearance) {
            return pathMinDistToBall(pose) >= clearance;
        };

        auto segmentIntersectsRect = [&](const Pose2D &from,
                                         const Pose2D &to,
                                         double rectMinX,
                                         double rectMaxX,
                                         double rectMinY,
                                         double rectMaxY) {
            const double dx = to.x - from.x;
            const double dy = to.y - from.y;
            double t0 = 0.0;
            double t1 = 1.0;

            auto clip = [&](double p, double q) {
                if (fabs(p) < 1e-9) return q >= 0.0;
                const double r = q / p;
                if (p < 0.0) {
                    if (r > t1) return false;
                    if (r > t0) t0 = r;
                } else {
                    if (r < t0) return false;
                    if (r < t1) t1 = r;
                }
                return true;
            };

            return clip(-dx, from.x - rectMinX)
                && clip(dx, rectMaxX - from.x)
                && clip(-dy, from.y - rectMinY)
                && clip(dy, rectMaxY - from.y)
                && t0 <= t1;
        };

        auto segmentCrossesOpponentGoalKickPenalty = [&](const Pose2D &from, const Pose2D &to) {
            if (!opponentGoalKickDefense) return false;
            if (isInsideOpponentGoalKickRoutePenalty(from) || isInsideOpponentGoalKickRoutePenalty(to)) return true;

            return segmentIntersectsRect(
                from,
                to,
                opponentPenaltyFrontX - opponentPenaltyRouteMarginX,
                fd.length / 2.0,
                -fd.penaltyAreaWidth / 2.0 - opponentPenaltyRouteMarginY,
                fd.penaltyAreaWidth / 2.0 + opponentPenaltyRouteMarginY);
        };
        auto directOpponentGoalKickPenaltyRouteAllowed = [&](const Pose2D &from, const Pose2D &to) {
            if (!opponentGoalKickDefense) return true;
            if (from.x <= opponentPenaltyFrontX - opponentPenaltyRouteMarginX + 0.02
                && to.x <= opponentPenaltyFrontX - opponentPenaltyRouteMarginX + 0.02) {
                return true;
            }
            if (!segmentCrossesOpponentGoalKickPenalty(from, to)) return true;

            if (isInsideOpponentGoalKickRoutePenalty(from) && isInsideOpponentGoalKickInnerLane(from)) return true;

            const double outerSign = opponentGoalKickOuterSideSign();
            const bool toOuterSide = outerSign * to.y >= fd.penaltyAreaWidth / 2.0 + opponentPenaltyRouteMarginY - 0.02;
            return isInsideOpponentGoalKickRoutePenalty(from) && toOuterSide;
        };

        const double opponentGoalFrontMinX =
            fd.length / 2.0 - max(1.55, opponentPenaltyExitMargin + 1.05);
        const double opponentGoalFrontMaxX = fd.length / 2.0 + 0.70;
        const double opponentGoalFrontHalfY =
            fd.goalWidth / 2.0 + max(0.85, defenseParams.fieldMarginY + 0.25);

        auto isInsideOpponentGoalFrontCorridor = [&](const Pose2D &pose) {
            return opponentGoalKickDefense
                && pose.x > opponentGoalFrontMinX
                && pose.x < opponentGoalFrontMaxX
                && fabs(pose.y) < opponentGoalFrontHalfY;
        };

        auto segmentCrossesOpponentGoalFrontCorridor = [&](const Pose2D &from, const Pose2D &to) {
            if (!opponentGoalKickDefense) return false;
            const bool goalSideRoute =
                from.x > ballPos.x + opponentGoalSideRiskMargin
                || to.x > ballPos.x + opponentGoalSideRiskMargin
                || isInsideOpponentGoalFrontCorridor(from)
                || isInsideOpponentGoalFrontCorridor(to);
            if (!goalSideRoute) return false;

            return segmentIntersectsRect(
                from,
                to,
                opponentGoalFrontMinX,
                opponentGoalFrontMaxX,
                -opponentGoalFrontHalfY,
                opponentGoalFrontHalfY);
        };

        auto shouldRouteAroundOpponentGoalFront = [&](const Pose2D &from, const Pose2D &to) {
            return opponentGoalKickDefense
                && !isInsideOpponentGoalFrontCorridor(from)
                && segmentCrossesOpponentGoalFrontCorridor(from, to);
        };

        auto opponentGoalKickPenaltyCornerPose = [&](const Pose2D &from, const Pose2D &to) {
            const double minX = safeFieldMinX;
            const double maxX = safeFieldMaxX;
            const double minY = safeFieldMinY;
            const double maxY = safeFieldMaxY;
            const double exitNudge = 0.10;
            double sideSign = 1.0;

            if (from.x <= opponentPenaltyExitX) {
                if (fabs(to.y) > opponentPenaltyExitY) {
                    sideSign = to.y >= 0.0 ? 1.0 : -1.0;
                } else if (fabs(from.y) > opponentPenaltyExitY) {
                    sideSign = from.y >= 0.0 ? 1.0 : -1.0;
                } else {
                    sideSign = (from.y + to.y) >= 0.0 ? 1.0 : -1.0;
                }
            } else if (fabs(from.y) > opponentPenaltyExitY) {
                sideSign = from.y >= 0.0 ? 1.0 : -1.0;
            } else if (fabs(to.y) > opponentPenaltyExitY) {
                sideSign = to.y >= 0.0 ? 1.0 : -1.0;
            } else {
                sideSign = (from.y + to.y) >= 0.0 ? 1.0 : -1.0;
            }

            Pose2D pose;
            pose.x = cap(opponentPenaltyExitX - exitNudge, maxX, minX);
            const double sideY = cap(opponentPenaltyExitY + exitNudge, maxY, 0.0);
            pose.y = sideSign > 0.0 ? sideY : -sideY;
            pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
            return pose;
        };

        auto opponentGoalKickGoalFrontAvoidancePose = [&](const Pose2D &from, const Pose2D &to) {
            const double minX = safeFieldMinX;
            const double maxX = safeFieldMaxX;
            const double minY = safeFieldMinY;
            const double maxY = safeFieldMaxY;
            const double exitNudge = 0.18;
            const double safeX = cap(
                min(opponentPenaltyExitX - exitNudge,
                    min(opponentGoalFrontMinX - exitNudge, ballPos.x - defenseSideXMargin)),
                maxX,
                minX);
            const double safeYAbs = cap(
                max(opponentGoalFrontHalfY + 0.30, opponentPenaltyExitY + 0.10),
                maxY,
                0.0);
            const double preferredSign = fabs(from.y) > fd.goalWidth / 2.0 + 0.08
                ? (from.y > 0.0 ? 1.0 : -1.0)
                : opponentGoalAwayOrbitSign();

            Pose2D bestPose;
            double bestScore = -1e9;
            for (int signIndex = 0; signIndex < 2; ++signIndex) {
                const double sign = signIndex == 0 ? preferredSign : -preferredSign;
                Pose2D candidate;
                candidate.x = safeX;
                candidate.y = sign > 0.0 ? safeYAbs : -safeYAbs;
                candidate.y = cap(candidate.y, maxY, minY);
                candidate.theta = atan2(ballPos.y - candidate.y, ballPos.x - candidate.x);

                const double pathBallClearance = segmentMinDistToBall(from, candidate);
                const double nextPathBallClearance = segmentMinDistToBall(candidate, to);
                const bool stillInGoalFront = isInsideOpponentGoalFrontCorridor(candidate);
                const bool stillInPenalty = isInsideOpponentGoalKickPenalty(candidate);
                double score =
                    5.0 * min(pathBallClearance, nextPathBallClearance)
                    - 0.45 * norm(candidate.x - from.x, candidate.y - from.y)
                    - 0.20 * norm(candidate.x - to.x, candidate.y - to.y);
                if (sign == preferredSign) score += 0.35;
                if (fabs(from.y) > fd.goalWidth / 2.0 + 0.08 && sign * from.y < 0.0) {
                    score -= 30.0;
                }
                if (stillInGoalFront) score -= 20.0;
                if (stillInPenalty) score -= 12.0;
                if (score > bestScore) {
                    bestPose = candidate;
                    bestScore = score;
                }
            }
            return bestPose;
        };

        auto fieldToRobotVelocity = [&](double fieldVx, double fieldVy) {
            return pair<double, double>{
                fieldVx * cos(robotPose.theta) + fieldVy * sin(robotPose.theta),
                -fieldVx * sin(robotPose.theta) + fieldVy * cos(robotPose.theta)};
        };

        auto limitOpponentGoalKickFieldBounds = [&](double &fieldVx,
                                                    double &fieldVy,
                                                    double speedLimit,
                                                    bool addInwardAssist) {
            if (!opponentGoalKickDefense) return;

            const double fallbackLimit = max(0.20, max(defenseParams.fastExitBackSpeed, defenseParams.boundaryArcSpeed));
            const double usableSpeedLimit = speedLimit > 1e-4 ? speedLimit : fallbackLimit;
            const double horizon = 0.42;

            auto capAxis = [&](double pos, double minPos, double maxPos, double &vel) {
                if (vel > 0.0) {
                    const double allowed = (maxPos - pos) / horizon;
                    vel = allowed <= 0.0 ? min(vel, 0.0) : min(vel, allowed);
                } else if (vel < 0.0) {
                    const double allowed = (minPos - pos) / horizon;
                    vel = allowed >= 0.0 ? max(vel, 0.0) : max(vel, allowed);
                }
            };

            auto assistAxis = [&](double pos,
                                  double minPos,
                                  double maxPos,
                                  double nearBand,
                                  double &vel) {
                const double assist = min(usableSpeedLimit, max(0.18, usableSpeedLimit * 0.38));
                if (pos > maxPos) {
                    const double ratio = std::clamp((pos - maxPos) / max(nearBand, 0.05) + 0.35, 0.35, 1.0);
                    vel = min(vel, -assist * ratio);
                } else if (pos > maxPos - nearBand && vel > -0.03) {
                    const double ratio = std::clamp((pos - (maxPos - nearBand)) / max(nearBand, 0.05), 0.15, 1.0);
                    vel = min(vel, -assist * ratio);
                } else if (pos < minPos) {
                    const double ratio = std::clamp((minPos - pos) / max(nearBand, 0.05) + 0.35, 0.35, 1.0);
                    vel = max(vel, assist * ratio);
                } else if (pos < minPos + nearBand && vel < 0.03) {
                    const double ratio = std::clamp((minPos + nearBand - pos) / max(nearBand, 0.05), 0.15, 1.0);
                    vel = max(vel, assist * ratio);
                }
            };

            const double nearBandX = max(0.08, opponentGoalKickFieldGuardX * 0.12);
            const double nearBandY = max(0.10, opponentGoalKickFieldGuardY * 0.12);
            capAxis(robotPose.x, safeFieldMinX, safeFieldMaxX, fieldVx);
            capAxis(robotPose.y, safeFieldMinY, safeFieldMaxY, fieldVy);
            if (addInwardAssist) {
                assistAxis(robotPose.x, safeFieldMinX, safeFieldMaxX, nearBandX, fieldVx);
                assistAxis(robotPose.y, safeFieldMinY, safeFieldMaxY, nearBandY, fieldVy);
                capAxis(robotPose.x, safeFieldMinX, safeFieldMaxX, fieldVx);
                capAxis(robotPose.y, safeFieldMinY, safeFieldMaxY, fieldVy);
            }

            const double finalSpeed = norm(fieldVx, fieldVy);
            if (finalSpeed > usableSpeedLimit && finalSpeed > 1e-6) {
                const double scale = usableSpeedLimit / finalSpeed;
                fieldVx *= scale;
                fieldVy *= scale;
            }
        };

        auto limitOpponentGoalKickNoApproach = [&](double &fieldVx,
                                                   double &fieldVy,
                                                   double speedLimit,
                                                   bool forceAwayWhenTouching) {
            if (!opponentGoalKickDefense || goalLineException) return;

            const double originalVx = fieldVx;
            const double originalVy = fieldVy;
            const double desiredSpeed = norm(originalVx, originalVy);
            const double fallbackLimit = max(0.20, max(defenseParams.fastExitBackSpeed, defenseParams.boundaryArcSpeed));
            const double usableSpeedLimit = speedLimit > 1e-4 ? speedLimit : fallbackLimit;

            auto enforceForBall = [&](const Pose2D &safetyBall) {
                double toBallX = safetyBall.x - robotPose.x;
                double toBallY = safetyBall.y - robotPose.y;
                const double ballDist = norm(toBallX, toBallY);
                if (ballDist < 1e-4) return;
                toBallX /= ballDist;
                toBallY /= ballDist;
                const double awayX = -toBallX;
                const double awayY = -toBallY;

                const double towardBall = fieldVx * toBallX + fieldVy * toBallY;
                const double projectedX = robotPose.x + fieldVx * 0.45;
                const double projectedY = robotPose.y + fieldVy * 0.45;
                const double projectedBallDist = norm(projectedX - safetyBall.x, projectedY - safetyBall.y);
                const bool mustBlockApproach =
                    ballDist < contactClearance + 0.18
                    || projectedBallDist < contactClearance + 0.03;
                if (towardBall > 0.0 && mustBlockApproach) {
                    fieldVx -= towardBall * toBallX;
                    fieldVy -= towardBall * toBallY;
                }

                double minAwaySpeed = 0.0;
                if (forceAwayWhenTouching && ballDist < contactClearance) {
                    minAwaySpeed = min(
                        usableSpeedLimit,
                        min(0.34, max(0.10, 0.08 + (contactClearance - ballDist) * 0.90)));
                } else if (forceAwayWhenTouching && ballDist < contactClearance + 0.10) {
                    minAwaySpeed = min(usableSpeedLimit, 0.05);
                }

                const double awaySpeed = fieldVx * awayX + fieldVy * awayY;
                if (awaySpeed < minAwaySpeed) {
                    const double correction = minAwaySpeed - awaySpeed;
                    fieldVx += correction * awayX;
                    fieldVy += correction * awayY;
                }
            };

            const auto safetyBalls = opponentGoalKickSafetyBalls();
            for (int pass = 0; pass < 2; ++pass) {
                for (const Pose2D &safetyBall : safetyBalls) {
                    enforceForBall(safetyBall);
                }
            }

            if (norm(fieldVx, fieldVy) < 0.04) {
                Pose2D safetyBall = activeSafetyBallPose();
                double awayX = robotPose.x - safetyBall.x;
                double awayY = robotPose.y - safetyBall.y;
                double awayNorm = norm(awayX, awayY);
                if (awayNorm < 1e-4) {
                    awayX = -1.0;
                    awayY = 0.0;
                    awayNorm = 1.0;
                }
                awayX /= awayNorm;
                awayY /= awayNorm;
                double tangentX = -awayY;
                double tangentY = awayX;
                if (originalVx * tangentX + originalVy * tangentY < 0.0) {
                    tangentX = -tangentX;
                    tangentY = -tangentY;
                }
                const double tangentSpeed = min(
                    usableSpeedLimit,
                    max(0.18, min(max(0.26, desiredSpeed), 0.55)));
                fieldVx = tangentX * tangentSpeed;
                fieldVy = tangentY * tangentSpeed;
                for (int pass = 0; pass < 2; ++pass) {
                    for (const Pose2D &candidateBall : safetyBalls) {
                        enforceForBall(candidateBall);
                    }
                }
            }

            const double finalSpeed = norm(fieldVx, fieldVy);
            if (finalSpeed > usableSpeedLimit && finalSpeed > 1e-6) {
                const double scale = usableSpeedLimit / finalSpeed;
                fieldVx *= scale;
                fieldVy *= scale;
            }
        };

        auto commandFieldVelocity = [&](double fieldVx,
                                         double fieldVy,
                                         double vtheta,
                                         double speedLimit,
                                         bool preserveNearBallExitSpeed = false) {
            if (opponentGoalKickDefense) {
                const Pose2D safetyBall = activeSafetyBallPose();
                const double safetyRobotBallDist = norm(robotPose.x - safetyBall.x, robotPose.y - safetyBall.y);
                const double yawAbs = fabs(activeSafetyBallYaw());
                double yawScale = 1.0;
                if (!preserveNearBallExitSpeed && safetyRobotBallDist < requiredBallDist + 0.35) {
                    if (yawAbs > 0.90) yawScale = 0.20;
                    else if (yawAbs > 0.65) yawScale = 0.40;
                    else if (yawAbs > 0.40) yawScale = 0.65;
                    else if (yawAbs > 0.25) yawScale = 0.82;
                }
                fieldVx *= yawScale;
                fieldVy *= yawScale;

                if (!goalLineException && safetyRobotBallDist < contactClearance + 0.30) {
                    double radialX = robotPose.x - safetyBall.x;
                    double radialY = robotPose.y - safetyBall.y;
                    const double radialNorm = norm(radialX, radialY);
                    if (radialNorm > 1e-4) {
                        radialX /= radialNorm;
                        radialY /= radialNorm;
                        const double awaySpeed = fieldVx * radialX + fieldVy * radialY;
                        const double minAwaySpeed = safetyRobotBallDist < contactClearance
                            ? min(0.18, max(0.06, (contactClearance - safetyRobotBallDist) * 0.35))
                            : 0.0;
                        if (awaySpeed < minAwaySpeed) {
                            const double correction = minAwaySpeed - awaySpeed;
                            fieldVx += correction * radialX;
                            fieldVy += correction * radialY;
                        }
                    }
                }

                limitOpponentGoalKickNoApproach(fieldVx, fieldVy, speedLimit, true);
                limitOpponentGoalKickFieldBounds(fieldVx, fieldVy, speedLimit, true);
            }

            auto [vx, vy] = fieldToRobotVelocity(fieldVx, fieldVy);
            const double vxCap = min(brain->config->get_vx_limit(), max(vxLimit, speedLimit));
            const double vyCap = min(brain->config->get_vy_limit(), max(vyLimit, speedLimit));
            auto scaleVelocityToCaps = [&]() {
                double scale = 1.0;
                if (vx > vxCap && vx > 1e-6) scale = min(scale, vxCap / vx);
                if (vx < -vxCap && vx < -1e-6) scale = min(scale, -vxCap / vx);
                if (vy > vyCap && vy > 1e-6) scale = min(scale, vyCap / vy);
                if (vy < -vyCap && vy < -1e-6) scale = min(scale, -vyCap / vy);
                if (scale < 1.0) {
                    vx *= max(0.0, scale);
                    vy *= max(0.0, scale);
                }
            };
            scaleVelocityToCaps();
            if (opponentGoalKickDefense) {
                double finalFieldVx = vx * cos(robotPose.theta) - vy * sin(robotPose.theta);
                double finalFieldVy = vx * sin(robotPose.theta) + vy * cos(robotPose.theta);
                limitOpponentGoalKickNoApproach(finalFieldVx, finalFieldVy, speedLimit, true);
                limitOpponentGoalKickFieldBounds(finalFieldVx, finalFieldVy, speedLimit, true);
                auto [limitedVx, limitedVy] = fieldToRobotVelocity(finalFieldVx, finalFieldVy);
                vx = limitedVx;
                vy = limitedVy;
                scaleVelocityToCaps();
            }
            vtheta = cap(vtheta, 1.5, -1.5);
            brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        };

        auto commandOpponentGoalKickGoalFrontAvoidance = [&](const Pose2D &finalTarget) {
            if (!opponentGoalKickDefense) return false;

            Pose2D avoidPose = opponentGoalKickGoalFrontAvoidancePose(robotPose, finalTarget);
            const bool avoidUsable =
                !isInsideOpponentGoalFrontCorridor(avoidPose)
                && !isInsideOpponentGoalKickPenalty(avoidPose)
                && poseLegal(avoidPose, max(contactClearance, requiredBallDist + 0.03))
                && pathKeepsBallClear(avoidPose, max(contactClearance, defenseParams.fastExitPathBallClearance));
            if (!avoidUsable) return false;

            vector<Pose2D> planPoints = {robotPose, avoidPose};
            if (norm(finalTarget.x - avoidPose.x, finalTarget.y - avoidPose.y) > 0.20) {
                planPoints.push_back(finalTarget);
            }
            setFreekickPlan("avoid_opponent_goal_front", planPoints, avoidPose, planMinDistToBall(planPoints));

            double fieldVx = avoidPose.x - robotPose.x;
            double fieldVy = avoidPose.y - robotPose.y;
            const double moveNorm = norm(fieldVx, fieldVy);
            if (moveNorm < 1e-4) return false;
            fieldVx = fieldVx / moveNorm * opponentGoalKickFastSpeed;
            fieldVy = fieldVy / moveNorm * opponentGoalKickFastSpeed;

            const double maxFieldX = safeFieldMaxX;
            const double minFieldX = safeFieldMinX;
            const double maxFieldY = safeFieldMaxY;
            if (robotPose.x < minFieldX + 0.10 && fieldVx < 0.0) fieldVx = 0.0;
            if (robotPose.x > maxFieldX - 0.05 && fieldVx > 0.0) fieldVx = 0.0;
            if (robotPose.y > maxFieldY - 0.10 && fieldVy > 0.0) fieldVy = 0.0;
            if (robotPose.y < -maxFieldY + 0.10 && fieldVy < 0.0) fieldVy = 0.0;
            if (fabs(fieldVx) < 1e-4 && fabs(fieldVy) < 1e-4) return false;

            commandFieldVelocity(
                fieldVx,
                fieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                opponentGoalKickFastSpeed);
            return true;
        };

        auto fieldInteriorUnitFromBall = [&]() {
            const double halfLength = fd.length / 2.0;
            const double halfWidth = fd.width / 2.0;
            double dirX = 0.0;
            double dirY = 0.0;

            if (ballPos.x < -halfLength + opponentGoalKickFieldGuardX) dirX += 1.0;
            else if (ballPos.x > halfLength - opponentGoalKickFieldGuardX) dirX -= 1.0;

            if (ballPos.y < -halfWidth + opponentGoalKickFieldGuardY) dirY += 1.0;
            else if (ballPos.y > halfWidth - opponentGoalKickFieldGuardY) dirY -= 1.0;

            double dirNorm = norm(dirX, dirY);
            if (dirNorm < 1e-6) {
                dirX = targetPose.x - ballPos.x;
                dirY = targetPose.y - ballPos.y;
                dirNorm = norm(dirX, dirY);
            }
            if (dirNorm < 1e-6) {
                dirX = -ballPos.x;
                dirY = -ballPos.y;
                dirNorm = norm(dirX, dirY);
            }
            if (dirNorm < 1e-6) {
                dirX = -1.0;
                dirY = 0.0;
                dirNorm = 1.0;
            }
            return pair<double, double>{dirX / dirNorm, dirY / dirNorm};
        };

        const auto [fieldInwardX, fieldInwardY] = fieldInteriorUnitFromBall();
        const double contactExitRadius = contactClearance + 0.05;
        const double contactOrbitRadius = contactClearance + 0.06;
        const double nearBallAngleTolerance = 0.22;
        const double infieldSideReleaseMargin = max(0.03, contactClearance * 0.12);

        auto poseOnInfieldSideOfBall = [&](const Pose2D &pose) {
            const double dx = pose.x - ballPos.x;
            const double dy = pose.y - ballPos.y;
            const double inwardProgress = dx * fieldInwardX + dy * fieldInwardY;
            if (inwardProgress > infieldSideReleaseMargin) return true;

            const double axisMargin = 0.02;
            if (fabs(fieldInwardY) > 0.35) {
                if (fieldInwardY > 0.0 && pose.y > ballPos.y + axisMargin) return true;
                if (fieldInwardY < 0.0 && pose.y < ballPos.y - axisMargin) return true;
            }
            if (fabs(fieldInwardX) > 0.35) {
                if (fieldInwardX > 0.0 && pose.x > ballPos.x + axisMargin) return true;
                if (fieldInwardX < 0.0 && pose.x < ballPos.x - axisMargin) return true;
            }
            return false;
        };
        const bool robotOnInfieldSideOfBall = poseOnInfieldSideOfBall(robotPose);

        auto commandLeaveBallContact = [&]() {
            const Pose2D safetyBall = activeSafetyBallPose();
            double awayX = robotPose.x - safetyBall.x;
            double awayY = robotPose.y - safetyBall.y;
            double awayNorm = norm(awayX, awayY);
            if (awayNorm < 1e-4) {
                awayX = fieldInwardX;
                awayY = fieldInwardY;
                awayNorm = 1.0;
            }
            awayX /= awayNorm;
            awayY /= awayNorm;

            double exitX = awayX + 0.45 * fieldInwardX;
            double exitY = awayY + 0.45 * fieldInwardY;
            const double exitNorm = norm(exitX, exitY);
            if (exitNorm > 1e-6) {
                exitX /= exitNorm;
                exitY /= exitNorm;
            } else {
                exitX = awayX;
                exitY = awayY;
            }
            const double radialComponent = exitX * awayX + exitY * awayY;
            if (radialComponent < 0.35) {
                exitX = awayX;
                exitY = awayY;
            }

            Pose2D exitTarget;
            const double safetyBallDist = norm(robotPose.x - safetyBall.x, robotPose.y - safetyBall.y);
            const double safetyDeficit = max(0.0, contactExitRadius - safetyBallDist);
            const double speed = min(0.36, max(0.16, 0.12 + safetyDeficit * 1.8));
            const double exitTargetRadius = max(contactExitRadius, safetyBallDist + max(0.08, safetyDeficit + 0.08));
            exitTarget.x = safetyBall.x + exitX * exitTargetRadius;
            exitTarget.y = safetyBall.y + exitY * exitTargetRadius;
            exitTarget = clampToField(exitTarget);
            exitTarget.theta = atan2(safetyBall.y - exitTarget.y, safetyBall.x - exitTarget.x);
            vector<Pose2D> planPoints = {robotPose, exitTarget};
            setFreekickPlan("leave_30cm", planPoints, exitTarget, planMinDistToBall(planPoints));
            commandFieldVelocity(
                exitX * speed,
                exitY * speed,
                activeSafetyBallYaw() * defenseParams.fastExitTurnGain,
                speed,
                opponentGoalKickDefense);
        };

        auto commandOpponentGoalKickImmediateBallExit = [&]() {
            if (!opponentGoalKickDefense || goalLineException) return false;

            const Pose2D safetyBall = activeSafetyBallPose();
            const double safetyBallDist = norm(robotPose.x - safetyBall.x, robotPose.y - safetyBall.y);
            if (safetyBallDist >= contactClearance) return false;

            double awayX = robotPose.x - safetyBall.x;
            double awayY = robotPose.y - safetyBall.y;
            double awayNorm = norm(awayX, awayY);
            if (awayNorm < 1e-4) {
                awayX = fieldInwardX;
                awayY = fieldInwardY;
                awayNorm = norm(awayX, awayY);
            }
            if (awayNorm < 1e-4) {
                awayX = -1.0;
                awayY = 0.0;
                awayNorm = 1.0;
            }
            awayX /= awayNorm;
            awayY /= awayNorm;

            const bool mustUseOuterLane =
                isInsideOpponentGoalKickRoutePenalty(robotPose)
                && !isInsideOpponentGoalKickInnerLane(robotPose);
            if (mustUseOuterLane) {
                const double outerSign = opponentGoalKickOuterSideSign();
                double sideX = 0.0;
                double sideY = outerSign;
                const double sideComponent = awayX * sideX + awayY * sideY;
                if (sideComponent < 0.25) {
                    awayX += (0.25 - sideComponent) * sideX;
                    awayY += (0.25 - sideComponent) * sideY;
                }
            } else {
                const double inwardComponent = awayX * fieldInwardX + awayY * fieldInwardY;
                if (inwardComponent < 0.15) {
                    awayX += (0.15 - inwardComponent) * fieldInwardX;
                    awayY += (0.15 - inwardComponent) * fieldInwardY;
                }
            }

            awayNorm = norm(awayX, awayY);
            if (awayNorm < 1e-4) return false;
            awayX /= awayNorm;
            awayY /= awayNorm;

            const double deficit = max(0.0, contactClearance - safetyBallDist);
            const double exitSpeed = min(
                opponentGoalKickFastSpeed,
                max(0.32, defenseParams.fastExitBackSpeed + deficit * 1.6));

            Pose2D exitTarget;
            const double targetRadius = max(contactClearance + 0.06, safetyBallDist + deficit + 0.12);
            exitTarget.x = safetyBall.x + awayX * targetRadius;
            exitTarget.y = safetyBall.y + awayY * targetRadius;
            exitTarget = clampToField(exitTarget);
            exitTarget.theta = atan2(safetyBall.y - exitTarget.y, safetyBall.x - exitTarget.x);

            vector<Pose2D> planPoints = {robotPose, exitTarget};
            setFreekickPlan("goal_kick_immediate_ball_exit", planPoints, exitTarget, planMinDistToBall(planPoints));
            commandFieldVelocity(
                awayX * exitSpeed,
                awayY * exitSpeed,
                activeSafetyBallYaw() * defenseParams.fastExitTurnGain,
                exitSpeed,
                true);
            return true;
        };

        auto commandOpponentGoalKickFallbackRetreat = [&](const string &stage) {
            if (!opponentGoalKickDefense) return false;

            const double maxFieldX = safeFieldMaxX;
            const double minFieldX = safeFieldMinX;
            const double maxFieldY = safeFieldMaxY;
            const double safeYAbs = cap(
                max(opponentPenaltyExitY + 0.20, fabs(ballPos.y) + contactClearance + 0.35),
                maxFieldY,
                0.0);
            const double sideSign = isInsideOpponentGoalKickPenalty(robotPose)
                ? opponentGoalKickOuterSideSign()
                : opponentGoalAwayOrbitSign();
            const double signedSafeY = sideSign > 0.0 ? safeYAbs : -safeYAbs;

            double fieldVx = 0.0;
            double fieldVy = 0.0;
            const double radialXRaw = robotPose.x - ballPos.x;
            const double radialYRaw = robotPose.y - ballPos.y;
            const double radialNorm = norm(radialXRaw, radialYRaw);
            if (radialNorm > 1e-4) {
                const double radialX = radialXRaw / radialNorm;
                const double radialY = radialYRaw / radialNorm;
                const double awaySpeed = robotBallDist < opponentGoalKickWidenReleaseClearance
                    ? min(opponentGoalKickFastSpeed * 0.75, 0.24 + (opponentGoalKickWidenReleaseClearance - robotBallDist) * 0.65)
                    : 0.18;
                fieldVx += radialX * awaySpeed;
                fieldVy += radialY * awaySpeed;
            } else {
                fieldVx -= 0.24;
            }

            const double leavePenaltySpeed = isInsideOpponentGoalKickPenalty(robotPose)
                ? min(opponentGoalKickFastSpeed, 0.55)
                : min(opponentGoalKickFastSpeed, 0.28);
            if (robotPose.x > opponentPenaltyExitX + 0.05) {
                fieldVx -= leavePenaltySpeed;
            }
            if (fabs(robotPose.y) < safeYAbs - 0.08) {
                fieldVy += (signedSafeY > robotPose.y ? 1.0 : -1.0) * min(opponentGoalKickFastSpeed, 0.42);
            }

            if (robotPose.x < minFieldX + 0.10 && fieldVx < 0.0) fieldVx = 0.0;
            if (robotPose.x > maxFieldX - 0.05 && fieldVx > 0.0) fieldVx = 0.0;
            if (robotPose.y > maxFieldY - 0.10 && fieldVy > 0.0) fieldVy = 0.0;
            if (robotPose.y < -maxFieldY + 0.10 && fieldVy < 0.0) fieldVy = 0.0;

            const double moveNorm = norm(fieldVx, fieldVy);
            if (moveNorm < 1e-4) {
                fieldVx = robotPose.x > opponentPenaltyExitX ? -min(opponentGoalKickFastSpeed, 0.35) : 0.0;
                fieldVy = fabs(robotPose.y) < safeYAbs - 0.08
                    ? (signedSafeY > robotPose.y ? 1.0 : -1.0) * min(opponentGoalKickFastSpeed, 0.35)
                    : 0.0;
            }
            if (fabs(fieldVx) < 1e-4 && fabs(fieldVy) < 1e-4) return false;

            Pose2D retreatTarget;
            retreatTarget.x = cap(robotPose.x + fieldVx * 0.80, maxFieldX, minFieldX);
            retreatTarget.y = cap(robotPose.y + fieldVy * 0.80, maxFieldY, -maxFieldY);
            retreatTarget = forceOutsideOpponentGoalKickPenalty(retreatTarget);
            retreatTarget.theta = atan2(ballPos.y - retreatTarget.y, ballPos.x - retreatTarget.x);
            vector<Pose2D> planPoints = {robotPose, retreatTarget};
            setFreekickPlan(stage, planPoints, retreatTarget, planMinDistToBall(planPoints));

            commandFieldVelocity(
                fieldVx,
                fieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                opponentGoalKickFastSpeed);
            return true;
        };

        auto commandOrbitToFieldInteriorSide = [&]() {
            double radialX = robotPose.x - ballPos.x;
            double radialY = robotPose.y - ballPos.y;
            double radialNorm = norm(radialX, radialY);
            if (radialNorm < 1e-4) return false;

            radialX /= radialNorm;
            radialY /= radialNorm;

            const double currentAngle = atan2(radialY, radialX);
            const double targetAngle = atan2(fieldInwardY, fieldInwardX);
            const double angleDelta = toPInPI(targetAngle - currentAngle);
            if (fabs(angleDelta) < nearBallAngleTolerance) return false;

            const double turnSign = angleDelta > 0.0 ? 1.0 : -1.0;
            const double tangentX = -radialY * turnSign;
            const double tangentY = radialX * turnSign;
            const double radialCorrection = cap((contactOrbitRadius - robotBallDist) * 2.8, 0.18, 0.0);
            const double tangentSpeed = min(0.42, max(0.24, fabs(angleDelta) * 0.24));
            const double fieldVx = tangentX * tangentSpeed + radialX * radialCorrection;
            const double fieldVy = tangentY * tangentSpeed + radialY * radialCorrection;
            const double orbitPlanDelta = cap(angleDelta, 0.55, -0.55);
            const int arcSteps = min(3, max(1, static_cast<int>(ceil(fabs(orbitPlanDelta) / 0.22))));
            const double planRadius = max(contactOrbitRadius, robotBallDist);
            vector<Pose2D> planPoints;
            planPoints.push_back(robotPose);
            Pose2D planTarget = robotPose;
            for (int i = 1; i <= arcSteps; ++i) {
                const double angle = currentAngle + orbitPlanDelta * (static_cast<double>(i) / arcSteps);
                Pose2D point;
                point.x = ballPos.x + planRadius * cos(angle);
                point.y = ballPos.y + planRadius * sin(angle);
                point.theta = atan2(ballPos.y - point.y, ballPos.x - point.x);
                planPoints.push_back(point);
                planTarget = point;
            }
            setFreekickPlan("orbit_to_infield_side", planPoints, planTarget, planMinDistToBall(planPoints));
            commandFieldVelocity(
                fieldVx,
                fieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                max(0.42, tangentSpeed + fabs(radialCorrection)));
            return true;
        };

        auto commandOpponentGoalKickOrbitAwayFromGoal = [&]() {
            if (!opponentGoalSideOrbitRisk) return false;

            double radialX = robotPose.x - ballPos.x;
            double radialY = robotPose.y - ballPos.y;
            const double radialNorm = norm(radialX, radialY);
            if (radialNorm < 1e-4) return false;
            radialX /= radialNorm;
            radialY /= radialNorm;

            const double safeSign = opponentGoalAwayOrbitSign();
            const double targetRadius = max(
                requiredBallDist + max(0.12, defenseParams.boundaryArcRadiusMargin),
                robotBallDist + 0.16);
            const double currentAngle = atan2(radialY, radialX);
            const double maxGoalSideX = robotPose.x - 0.03;
            vector<double> stepCandidates = {0.75, 1.00, 1.25, 1.55, 1.85, 2.15};
            Pose2D orbitTarget = robotPose;
            bool hasOrbitTarget = false;
            for (double step : stepCandidates) {
                const double angle = currentAngle + safeSign * step;
                Pose2D candidate;
                candidate.x = ballPos.x + targetRadius * cos(angle);
                candidate.y = ballPos.y + targetRadius * sin(angle);
                candidate.theta = atan2(ballPos.y - candidate.y, ballPos.x - candidate.x);
                if (candidate.x > maxGoalSideX) continue;
                if (!insideField(candidate)) continue;
                orbitTarget = candidate;
                hasOrbitTarget = true;
                break;
            }
            if (!hasOrbitTarget) {
                const double angle = currentAngle + safeSign * stepCandidates.back();
                orbitTarget.x = min(maxGoalSideX, ballPos.x + targetRadius * cos(angle));
                orbitTarget.y = ballPos.y + targetRadius * sin(angle);
                orbitTarget = clampToField(orbitTarget);
            }

            double moveX = orbitTarget.x - robotPose.x;
            double moveY = orbitTarget.y - robotPose.y;
            double moveNorm = norm(moveX, moveY);
            if (moveNorm < 1e-4) {
                moveX = -radialY * safeSign;
                moveY = radialX * safeSign;
                moveNorm = norm(moveX, moveY);
            }
            moveX /= moveNorm;
            moveY /= moveNorm;

            const double outwardComponent = moveX * radialX + moveY * radialY;
            if (outwardComponent < 0.0) {
                moveX -= outwardComponent * radialX;
                moveY -= outwardComponent * radialY;
                moveNorm = norm(moveX, moveY);
                if (moveNorm > 1e-6) {
                    moveX /= moveNorm;
                    moveY /= moveNorm;
                }
            }

            const double tangentSpeed = min(
                defenseParams.boundaryArcSpeed,
                max(0.36, 0.22 + max(0.0, targetRadius - robotBallDist) * 0.45));

            double fieldVx = moveX * tangentSpeed;
            double fieldVy = moveY * tangentSpeed;
            if (fieldVx > 0.0) fieldVx = 0.0;

            const double halfWidth = fd.width / 2.0;
            const double edgeSlowMarginY = max(0.12, defenseParams.fieldMarginY * 0.7);
            if (robotPose.y > halfWidth - edgeSlowMarginY && fieldVy > 0.0) fieldVy = 0.0;
            if (robotPose.y < -halfWidth + edgeSlowMarginY && fieldVy < 0.0) fieldVy = 0.0;
            if (fabs(fieldVx) < 1e-4 && fabs(fieldVy) < 1e-4) {
                fieldVx = -min(0.18, defenseParams.boundaryArcSpeed);
            }

            const int arcSteps = 3;
            const double targetAngle = atan2(orbitTarget.y - ballPos.y, orbitTarget.x - ballPos.x);
            const double planStep = cap(toPInPI(targetAngle - currentAngle), 2.15, -2.15);
            const double planRadius = max(robotBallDist, min(targetRadius, robotBallDist + 0.35));
            vector<Pose2D> planPoints;
            planPoints.push_back(robotPose);
            Pose2D planTarget = robotPose;
            for (int i = 1; i <= arcSteps; ++i) {
                const double angle = currentAngle + planStep * (static_cast<double>(i) / arcSteps);
                Pose2D point;
                point.x = ballPos.x + planRadius * cos(angle);
                point.y = ballPos.y + planRadius * sin(angle);
                point.theta = atan2(ballPos.y - point.y, ballPos.x - point.x);
                planPoints.push_back(point);
                planTarget = point;
            }
            setFreekickPlan("goal_kick_orbit_away_from_goal", planPoints, planTarget, planMinDistToBall(planPoints));
            commandFieldVelocity(
                fieldVx,
                fieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                max(0.42, tangentSpeed));
            return true;
        };

        auto commandOpponentGoalKickSafeSideExit = [&]() {
            if (!opponentGoalKickDefense || !isInsideOpponentGoalKickPenalty(robotPose)) return false;
            if (isInsideOpponentGoalFrontCorridor(robotPose)) return false;

            const double sideSign = isInsideOpponentGoalKickPenalty(robotPose)
                ? opponentGoalKickOuterSideSign()
                : opponentGoalAwayOrbitSign();
            const double maxFieldX = safeFieldMaxX;
            const double minFieldX = safeFieldMinX;
            const double maxFieldY = safeFieldMaxY;
            const double laneAbsY = cap(
                max(
                    opponentPenaltyExitY + 0.05,
                    fabs(ballPos.y) + opponentGoalKickPathClearance + 0.10),
                maxFieldY,
                0.0);
            if (laneAbsY < opponentPenaltyExitY + 0.02) return false;

            const double signedLaneY = sideSign > 0.0 ? laneAbsY : -laneAbsY;
            const double safeSideRadius = max(
                opponentGoalKickNoTouchRadius,
                robotBallDist + (robotBallDist < opponentGoalKickNoTouchRadius ? 0.18 : 0.0));
            const bool needsSideLane =
                !isInsideOpponentGoalKickInnerLane(robotPose)
                || ((fabs(robotPose.y) < laneAbsY - 0.12 || robotBallDist < opponentGoalKickNoTouchRadius)
                    && (opponentGoalSideOrbitRisk
                        || robotPose.x > opponentPenaltyFrontX - 0.15
                        || pathMinDistToBall(forceOutsideOpponentGoalKickPenalty(robotPose)) < opponentGoalKickWidenReleaseClearance));
            if (!needsSideLane) return false;

            auto commandWidenBallClearance = [&]() {
                double radialX = robotPose.x - ballPos.x;
                double radialY = robotPose.y - ballPos.y;
                const double radialNorm = norm(radialX, radialY);
                if (radialNorm < 1e-4) return false;
                radialX /= radialNorm;
                radialY /= radialNorm;

                const double deficit = max(0.0, opponentGoalKickWidenReleaseClearance - robotBallDist);
                const double clearRatio = std::clamp(
                    (robotBallDist - contactClearance) / max(0.20, opponentGoalKickWidenReleaseClearance - contactClearance),
                    0.0,
                    1.0);
                double lateralDir = signedLaneY > robotPose.y ? 1.0 : -1.0;
                if (lateralDir * radialY < -0.05) {
                    lateralDir = radialY >= 0.0 ? 1.0 : -1.0;
                }

                double lateralSpeed = min(opponentGoalKickFastSpeed, 0.62 + clearRatio * 0.26);
                const double laneError = fabs(signedLaneY - robotPose.y);
                if (laneError < 0.55) {
                    lateralSpeed = min(lateralSpeed, max(0.30, laneError * 1.6));
                }

                double fieldVx = 0.0;
                double fieldVy = lateralDir * lateralSpeed;
                const double radialAssistSpeed = min(0.26, 0.05 + deficit * 0.35);
                if (lateralDir * radialY > 0.15) {
                    fieldVy += radialY * radialAssistSpeed;
                } else if (fabs(radialY) < 0.08) {
                    fieldVy += lateralDir * radialAssistSpeed;
                }
                if (radialX < -0.05) {
                    fieldVx += radialX * radialAssistSpeed;
                }

                if (robotPose.x < minFieldX + 0.10 && fieldVx < 0.0) fieldVx = 0.0;
                if (robotPose.x > maxFieldX - 0.05 && fieldVx > 0.0) fieldVx = 0.0;
                if (robotPose.y > maxFieldY - 0.10 && fieldVy > 0.0) fieldVy = 0.0;
                if (robotPose.y < -maxFieldY + 0.10 && fieldVy < 0.0) fieldVy = 0.0;

                const double predictedDist = norm(
                    robotPose.x + fieldVx * 0.45 - ballPos.x,
                    robotPose.y + fieldVy * 0.45 - ballPos.y);
                if (predictedDist < robotBallDist - 0.03 || predictedDist < contactClearance) {
                    fieldVx = 0.0;
                    fieldVy = (radialY >= 0.0 ? 1.0 : -1.0) * min(opponentGoalKickFastSpeed, 0.45 + deficit * 0.8);
                    if (robotPose.y > maxFieldY - 0.10 && fieldVy > 0.0) fieldVy = 0.0;
                    if (robotPose.y < -maxFieldY + 0.10 && fieldVy < 0.0) fieldVy = 0.0;

                    const double fallbackPredictedDist = norm(
                        robotPose.x + fieldVx * 0.45 - ballPos.x,
                        robotPose.y + fieldVy * 0.45 - ballPos.y);
                    if (fallbackPredictedDist < robotBallDist - 0.03
                        || fallbackPredictedDist < contactClearance) {
                        return false;
                    }
                }
                if (fabs(fieldVx) < 1e-4 && fabs(fieldVy) < 1e-4) return false;

                vector<Pose2D> planPoints;
                planPoints.push_back(robotPose);
                Pose2D planTarget;
                planTarget.x = robotPose.x + fieldVx * 0.85;
                planTarget.y = cap(robotPose.y + fieldVy * 0.85, maxFieldY, -maxFieldY);
                planTarget.theta = atan2(ballPos.y - planTarget.y, ballPos.x - planTarget.x);
                planPoints.push_back(planTarget);

                Pose2D sideLaneHint = planTarget;
                sideLaneHint.x = robotPose.x;
                sideLaneHint.y = cap(signedLaneY, maxFieldY, -maxFieldY);
                sideLaneHint.theta = atan2(ballPos.y - sideLaneHint.y, ballPos.x - sideLaneHint.x);
                planPoints.push_back(sideLaneHint);

                setFreekickPlan("goal_kick_widen_ball_clearance", planPoints, planTarget, planMinDistToBall(planPoints));
                commandFieldVelocity(
                    fieldVx,
                    fieldVy,
                    motionBallYawToRobot * defenseParams.fastExitTurnGain,
                    opponentGoalKickFastSpeed);
                return true;
            };

            const double lateralAngle = sideSign > 0.0 ? M_PI / 2.0 : -M_PI / 2.0;
            const double targetAngle = atan2(signedLaneY - ballPos.y, robotPose.x - ballPos.x);
            const double blendAngle = fabs(toPInPI(targetAngle - lateralAngle)) < 1.10 ? targetAngle : lateralAngle;

            Pose2D laneTarget;
            laneTarget.x = cap(
                ballPos.x + safeSideRadius * cos(blendAngle),
                maxFieldX,
                minFieldX);
            laneTarget.y = cap(
                ballPos.y + safeSideRadius * sin(blendAngle),
                maxFieldY,
                -maxFieldY);

            if (sideSign > 0.0) {
                laneTarget.y = max(laneTarget.y, min(signedLaneY, maxFieldY));
            } else {
                laneTarget.y = min(laneTarget.y, max(signedLaneY, -maxFieldY));
            }
            if (opponentGoalSideOrbitRisk) {
                laneTarget.x = min(laneTarget.x, min(robotPose.x, maxFieldX));
            }
            laneTarget.theta = atan2(ballPos.y - laneTarget.y, ballPos.x - laneTarget.x);

            const double lanePathMinBallDist = segmentMinDistToBall(robotPose, laneTarget);
            if (shouldRouteAroundOpponentGoalFront(robotPose, laneTarget)
                && commandOpponentGoalKickGoalFrontAvoidance(laneTarget)) {
                return true;
            }
            if (lanePathMinBallDist < opponentGoalKickWidenReleaseClearance) {
                return commandWidenBallClearance();
            }

            const bool onSafeLane = fabs(robotPose.y) > fd.penaltyAreaWidth / 2.0 + 0.25;
            if (onSafeLane) {
                Pose2D frontExitTarget = laneTarget;
                frontExitTarget.x = cap(opponentPenaltyExitX - 0.18, maxFieldX, minFieldX);
                frontExitTarget.theta = atan2(ballPos.y - frontExitTarget.y, ballPos.x - frontExitTarget.x);
                if (segmentMinDistToBall(robotPose, frontExitTarget) >= opponentGoalKickWidenReleaseClearance) {
                    laneTarget = frontExitTarget;
                }
            }

            laneTarget.theta = atan2(ballPos.y - laneTarget.y, ballPos.x - laneTarget.x);
            vector<Pose2D> planPoints = {robotPose, laneTarget};
            const double finalLanePathMinBallDist = planMinDistToBall(planPoints);
            if (finalLanePathMinBallDist < opponentGoalKickWidenReleaseClearance) return false;
            if (!directOpponentGoalKickPenaltyRouteAllowed(robotPose, laneTarget)) return false;
            if (shouldRouteAroundOpponentGoalFront(robotPose, laneTarget)
                && commandOpponentGoalKickGoalFrontAvoidance(laneTarget)) {
                return true;
            }

            double fieldVx = laneTarget.x - robotPose.x;
            double fieldVy = laneTarget.y - robotPose.y;
            const double moveNorm = norm(fieldVx, fieldVy);
            if (moveNorm < 1e-4) return false;
            fieldVx = fieldVx / moveNorm * opponentGoalKickFastSpeed;
            fieldVy = fieldVy / moveNorm * opponentGoalKickFastSpeed;

            const double ballRadialX = robotPose.x - ballPos.x;
            const double ballRadialY = robotPose.y - ballPos.y;
            const double ballRadialNorm = norm(ballRadialX, ballRadialY);
            if (ballRadialNorm > 1e-4) {
                const double unitX = ballRadialX / ballRadialNorm;
                const double unitY = ballRadialY / ballRadialNorm;
                const double towardBall = fieldVx * unitX + fieldVy * unitY;
                const double minAwaySpeed = robotBallDist < opponentGoalKickWidenReleaseClearance
                    ? min(opponentGoalKickFastSpeed * 0.70, 0.18 + (opponentGoalKickWidenReleaseClearance - robotBallDist) * 0.9)
                    : 0.0;
                if (towardBall < minAwaySpeed) {
                    const double correction = minAwaySpeed - towardBall;
                    fieldVx += correction * unitX;
                    fieldVy += correction * unitY;
                    const double correctedNorm = norm(fieldVx, fieldVy);
                    if (correctedNorm > opponentGoalKickFastSpeed && correctedNorm > 1e-4) {
                        fieldVx *= opponentGoalKickFastSpeed / correctedNorm;
                        fieldVy *= opponentGoalKickFastSpeed / correctedNorm;
                    }
                }
            }

            if (robotPose.x < minFieldX + 0.10 && fieldVx < 0.0) fieldVx = 0.0;
            if (robotPose.x > maxFieldX && fieldVx > 0.0) fieldVx = 0.0;
            if (robotPose.y > maxFieldY - 0.10 && fieldVy > 0.0) fieldVy = 0.0;
            if (robotPose.y < -maxFieldY + 0.10 && fieldVy < 0.0) fieldVy = 0.0;
            if (fabs(fieldVx) < 1e-4 && fabs(fieldVy) < 1e-4) return false;

            setFreekickPlan("goal_kick_safe_side_exit", planPoints, laneTarget, finalLanePathMinBallDist);
            commandFieldVelocity(
                fieldVx,
                fieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                opponentGoalKickFastSpeed);
            return true;
        };

        auto safeEnterFieldPose = [&]() -> pair<bool, Pose2D> {
            vector<Pose2D> candidates;
            const double minX = safeFieldMinX;
            const double maxX = safeFieldMaxX;
            const double minY = safeFieldMinY;
            const double maxY = safeFieldMaxY;

            auto addCandidate = [&](double x, double y) {
                Pose2D pose;
                pose.x = cap(x, maxX, minX);
                pose.y = cap(y, maxY, minY);
                pose.theta = atan2(ballPos.y - pose.y, ballPos.x - pose.x);
                candidates.push_back(pose);
            };

            Pose2D projected = clampToField(robotPose);
            addCandidate(projected.x, projected.y);
            addCandidate(targetPose.x, targetPose.y);

            const double awayAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
            Pose2D circleEntry = bestCirclePose(
                awayAngle,
                requiredBallDist + max(0.10, defenseParams.boundaryArcRadiusMargin),
                targetPose);
            addCandidate(circleEntry.x, circleEntry.y);

            const double entryCircleRadius = max(contactClearance + 0.08, requiredBallDist + max(0.10, defenseParams.boundaryArcRadiusMargin));
            vector<double> circleOffsets = {
                0.0, 0.25, -0.25, 0.50, -0.50, 0.80, -0.80,
                1.15, -1.15, 1.55, -1.55, 2.0, -2.0, 2.55, -2.55, M_PI
            };
            for (double offset : circleOffsets) {
                const double angle = toPInPI(awayAngle + offset);
                addCandidate(
                    ballPos.x + entryCircleRadius * cos(angle),
                    ballPos.y + entryCircleRadius * sin(angle));
            }

            vector<double> offsets = {0.0, 0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.4, -2.4, 3.0, -3.0};
            const bool outsideY = robotPose.y > fd.width / 2.0 - 0.02
                || robotPose.y < -fd.width / 2.0 + 0.02;
            const bool outsideX = robotPose.x > fd.length / 2.0 - 0.02
                || robotPose.x < -fd.length / 2.0 + 0.02;

            if (outsideY) {
                const double entryY = robotPose.y > 0.0 ? maxY : minY;
                const double robotBaseX = cap(robotPose.x, maxX, minX);
                const double ballBaseX = cap(ballPos.x, maxX, minX);
                for (double offset : offsets) {
                    addCandidate(robotBaseX + offset, entryY);
                    addCandidate(ballBaseX + offset, entryY);
                }
            }

            if (outsideX) {
                const double entryX = robotPose.x > 0.0 ? maxX : minX;
                const double robotBaseY = cap(robotPose.y, maxY, minY);
                const double ballBaseY = cap(ballPos.y, maxY, minY);
                for (double offset : offsets) {
                    addCandidate(entryX, robotBaseY + offset);
                    addCandidate(entryX, ballBaseY + offset);
                }
            }

            const double targetClearance = requiredBallDist + 0.03;
            const double pathClearance = min(max(contactClearance, defenseParams.fastExitPathBallClearance), max(contactClearance, robotBallDist - 0.03));

            Pose2D bestPose = projected;
            double bestCost = 1e9;
            bool found = false;
            Pose2D bestFallbackPose = projected;
            double bestFallbackScore = -1e9;
            auto consider = [&](const Pose2D &pose, double minTargetClearance, double minPathClearance) {
                const double targetDist = poseBallDist(pose);
                const double pathDist = pathMinDistToBall(pose);
                const double fallbackScore =
                    8.0 * min(targetDist, pathDist)
                    - 0.25 * norm(pose.x - robotPose.x, pose.y - robotPose.y)
                    - 0.10 * norm(pose.x - targetPose.x, pose.y - targetPose.y);
                if (fallbackScore > bestFallbackScore) {
                    bestFallbackPose = pose;
                    bestFallbackScore = fallbackScore;
                }
                if (targetDist < minTargetClearance || pathDist < minPathClearance) return;

                const double cost =
                    norm(pose.x - robotPose.x, pose.y - robotPose.y)
                    + 0.35 * norm(pose.x - targetPose.x, pose.y - targetPose.y)
                    + (pathDist < contactClearance ? (contactClearance - pathDist) * 3.0 : 0.0);
                if (cost < bestCost) {
                    bestPose = pose;
                    bestCost = cost;
                    found = true;
                }
            };

            for (const auto &candidate : candidates) {
                consider(candidate, targetClearance, pathClearance);
            }
            if (!found) {
                for (const auto &candidate : candidates) {
                    consider(candidate, contactClearance, min(pathClearance, max(contactClearance, robotBallDist - 0.08)));
                }
            }

            if (!found) {
                return {false, bestFallbackPose};
            }
            bestPose.theta = atan2(ballPos.y - bestPose.y, ballPos.x - bestPose.x);
            return {true, bestPose};
        };

        auto commandToPose = [&](const Pose2D &pose, bool fast, double overrideSpeedLimit, bool forceRetreatFromBall = true) {
            Pose2D targetRobot = brain->data->field2robot(pose);
            const Pose2D safetyBall = activeSafetyBallPose();
            const double safetyRobotBallDist = norm(robotPose.x - safetyBall.x, robotPose.y - safetyBall.y);
            const double ballYaw = activeSafetyBallYaw();
            double vx = targetRobot.x;
            double vy = targetRobot.y;
            double vtheta = ballYaw * (fast ? defenseParams.fastExitTurnGain : 2.0);

            if (!fast) {
                double translationScale = 1.0;
                if (fabs(ballYaw) > 1.0) {
                    translationScale = 0.25;
                } else if (fabs(ballYaw) > 0.7) {
                    translationScale = 0.5;
                } else if (fabs(ballYaw) > 0.4) {
                    translationScale = 0.75;
                }
                vx *= translationScale;
                vy *= translationScale;
            }

            const double configuredFastLimit = overrideSpeedLimit > 0.0
                ? overrideSpeedLimit
                : defenseParams.fastExitBackSpeed;
            double fastXLimit = min(brain->config->get_vx_limit(), max(vxLimit, configuredFastLimit));
            double fastYLimit = min(brain->config->get_vy_limit(), max(vyLimit, configuredFastLimit));
            double vxUpper = fastXLimit;
            double vxLower = -fastXLimit;
            double vyUpper = fastYLimit;
            double vyLower = -fastYLimit;
            if (fast) {
                // Bounds already initialized above.
            } else if (ballInOwnHalf) {
                vxUpper = min(vxLimit, 0.08);
                vxLower = -min(vxLimit, 0.35);
                vyUpper = vyLimit;
                vyLower = -vyLimit;
            } else {
                vxUpper = min(vxLimit, 0.12);
                vxLower = -min(vxLimit, 0.22);
                vyUpper = vyLimit;
                vyLower = -vyLimit;
            }

            auto capTranslation = [&]() {
                vx = cap(vx, vxUpper, vxLower);
                vy = cap(vy, vyUpper, vyLower);
            };

            auto scaleTranslationToBounds = [&]() {
                double scale = 1.0;
                if (vx > vxUpper && vx > 1e-6) scale = min(scale, vxUpper / vx);
                if (vx < vxLower && vx < -1e-6) scale = min(scale, vxLower / vx);
                if (vy > vyUpper && vy > 1e-6) scale = min(scale, vyUpper / vy);
                if (vy < vyLower && vy < -1e-6) scale = min(scale, vyLower / vy);
                if (scale < 1.0) {
                    vx *= max(0.0, scale);
                    vy *= max(0.0, scale);
                }
            };

            capTranslation();

            const double protectBallDist = forceRetreatFromBall
                ? requiredBallDist + max(0.05, defenseParams.fastExitReleaseMargin)
                : contactClearance;
            if (!goalLineException && safetyRobotBallDist < protectBallDist) {
                const double ballUnitX = cos(ballYaw);
                const double ballUnitY = sin(ballYaw);
                double towardBall = vx * ballUnitX + vy * ballUnitY;
                double maxTowardBall = 0.0;

                if (forceRetreatFromBall && safetyRobotBallDist < requiredBallDist) {
                    const double deficit = requiredBallDist - safetyRobotBallDist;
                    const double retreatRatio = std::clamp(deficit / max(0.35, requiredBallDist * 0.35), 0.0, 1.0);
                    const double limitForRetreat = fast ? fastXLimit : max(vxLimit, vyLimit);
                    const double minAwaySpeed = min(
                        limitForRetreat,
                        0.10 + retreatRatio * max(0.0, defenseParams.fastExitBackSpeed - 0.10));
                    maxTowardBall = -minAwaySpeed;
                }

                if (towardBall > maxTowardBall) {
                    const double correction = towardBall - maxTowardBall;
                    vx -= correction * ballUnitX;
                    vy -= correction * ballUnitY;
                }

                scaleTranslationToBounds();

                const double finalTowardBall = vx * ballUnitX + vy * ballUnitY;
                if (finalTowardBall > 0.0) {
                    vx -= finalTowardBall * ballUnitX;
                    vy -= finalTowardBall * ballUnitY;
                    scaleTranslationToBounds();
                }

                if (forceRetreatFromBall && maxTowardBall < 0.0) {
                    const double boundedTowardBall = vx * ballUnitX + vy * ballUnitY;
                    if (boundedTowardBall > maxTowardBall) {
                        const double correction = boundedTowardBall - maxTowardBall;
                        vx -= correction * ballUnitX;
                        vy -= correction * ballUnitY;
                        scaleTranslationToBounds();
                    }
                }
            }

            const double nextDt = 0.35;
            const double predictedX = robotPose.x + (vx * cos(robotPose.theta) - vy * sin(robotPose.theta)) * nextDt;
            const double predictedY = robotPose.y + (vx * sin(robotPose.theta) + vy * cos(robotPose.theta)) * nextDt;
            if (!goalLineException
                && safetyRobotBallDist < contactClearance + 0.08
                && norm(predictedX - safetyBall.x, predictedY - safetyBall.y) < contactClearance) {
                const double ballUnitX = cos(ballYaw);
                const double ballUnitY = sin(ballYaw);
                const double towardBall = vx * ballUnitX + vy * ballUnitY;
                const double maxTowardBall = -0.08;
                if (towardBall > maxTowardBall) {
                    const double correction = towardBall - maxTowardBall;
                    vx -= correction * ballUnitX;
                    vy -= correction * ballUnitY;
                    scaleTranslationToBounds();
                }
            }

            const double halfLength = fd.length / 2.0;
            const double halfWidth = fd.width / 2.0;
            const double edgeSlowMarginX = max(0.10, defenseParams.fieldMarginX * 0.6);
            const double edgeSlowMarginY = max(0.10, defenseParams.fieldMarginY * 0.6);
            double fieldVx = vx * cos(robotPose.theta) - vy * sin(robotPose.theta);
            double fieldVy = vx * sin(robotPose.theta) + vy * cos(robotPose.theta);
            double safeFieldVx = fieldVx;
            double safeFieldVy = fieldVy;
            if (opponentGoalKickDefense) {
                limitOpponentGoalKickFieldBounds(safeFieldVx, safeFieldVy, configuredFastLimit, true);
            } else {
                if (robotPose.x < -halfLength + edgeSlowMarginX && safeFieldVx < 0.0) safeFieldVx = 0.0;
                if (robotPose.x >  halfLength - edgeSlowMarginX && safeFieldVx > 0.0) safeFieldVx = 0.0;
                if (robotPose.y < -halfWidth + edgeSlowMarginY && safeFieldVy < 0.0) safeFieldVy = 0.0;
                if (robotPose.y >  halfWidth - edgeSlowMarginY && safeFieldVy > 0.0) safeFieldVy = 0.0;
            }
            if (safeFieldVx != fieldVx || safeFieldVy != fieldVy) {
                vx = safeFieldVx * cos(robotPose.theta) + safeFieldVy * sin(robotPose.theta);
                vy = -safeFieldVx * sin(robotPose.theta) + safeFieldVy * cos(robotPose.theta);
                scaleTranslationToBounds();
            }

            const double shortHorizon = 0.45;
            const double nextFieldX = robotPose.x + (vx * cos(robotPose.theta) - vy * sin(robotPose.theta)) * shortHorizon;
            const double nextFieldY = robotPose.y + (vx * sin(robotPose.theta) + vy * cos(robotPose.theta)) * shortHorizon;
            const Line shortPath = {robotPose.x, robotPose.y, nextFieldX, nextFieldY};
            const double shortPathBallDist = pointMinDistToLine(Point2D({safetyBall.x, safetyBall.y}), shortPath);
            if (!goalLineException && shortPathBallDist < contactClearance) {
                const double ballUnitX = cos(ballYaw);
                const double ballUnitY = sin(ballYaw);
                vx = -ballUnitX * min(0.35, max(0.16, contactClearance - safetyRobotBallDist + 0.12));
                vy = -ballUnitY * min(0.35, max(0.16, contactClearance - safetyRobotBallDist + 0.12));
                if (opponentGoalKickDefense) {
                    const double desiredFieldVx = vx * cos(robotPose.theta) - vy * sin(robotPose.theta);
                    const double desiredFieldVy = vx * sin(robotPose.theta) + vy * cos(robotPose.theta);
                    const double penaltyExitAssist = robotPose.x > opponentPenaltyExitX ? -0.22 : 0.0;
                    const double sideSign = opponentGoalAwayOrbitSign();
                    const double sideAssist = fabs(robotPose.y) < opponentPenaltyExitY + 0.12 ? sideSign * 0.18 : 0.0;
                    const double adjustedFieldVx = desiredFieldVx + penaltyExitAssist;
                    const double adjustedFieldVy = desiredFieldVy + sideAssist;
                    vx = adjustedFieldVx * cos(robotPose.theta) + adjustedFieldVy * sin(robotPose.theta);
                    vy = -adjustedFieldVx * sin(robotPose.theta) + adjustedFieldVy * cos(robotPose.theta);
                }
                scaleTranslationToBounds();
                if (opponentGoalKickDefense) {
                    double boundedFieldVx = vx * cos(robotPose.theta) - vy * sin(robotPose.theta);
                    double boundedFieldVy = vx * sin(robotPose.theta) + vy * cos(robotPose.theta);
                    limitOpponentGoalKickFieldBounds(boundedFieldVx, boundedFieldVy, configuredFastLimit, true);
                    vx = boundedFieldVx * cos(robotPose.theta) + boundedFieldVy * sin(robotPose.theta);
                    vy = -boundedFieldVx * sin(robotPose.theta) + boundedFieldVy * cos(robotPose.theta);
                    scaleTranslationToBounds();
                }
            }

            if (opponentGoalKickDefense) {
                double finalFieldVx = vx * cos(robotPose.theta) - vy * sin(robotPose.theta);
                double finalFieldVy = vx * sin(robotPose.theta) + vy * cos(robotPose.theta);
                limitOpponentGoalKickNoApproach(finalFieldVx, finalFieldVy, configuredFastLimit, true);
                limitOpponentGoalKickFieldBounds(finalFieldVx, finalFieldVy, configuredFastLimit, true);
                vx = finalFieldVx * cos(robotPose.theta) + finalFieldVy * sin(robotPose.theta);
                vy = -finalFieldVx * sin(robotPose.theta) + finalFieldVy * cos(robotPose.theta);
                scaleTranslationToBounds();
            }

            vtheta = cap(vtheta, 1.5, -1.5);
            brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        };

        auto commandOpponentGoalKickDefenseRetreat = [&]() {
            if (!opponentGoalKickDefense) return false;

            if (commandOpponentGoalKickImmediateBallExit()) {
                return true;
            }

            const double noTouchClearance = contactClearance;
            const double escapeClearance = noTouchClearance;
            const double routeClearance = noTouchClearance;
            const double directRouteClearance = noTouchClearance;
            const double minX = safeFieldMinX;
            const double maxX = safeFieldMaxX;
            const double minY = safeFieldMinY;
            const double maxY = safeFieldMaxY;
            const double goalMouthGuardX = max(0.62, defenseParams.fieldMarginX + 0.32);
            const double goalMouthGuardY = max(0.55, defenseParams.fieldMarginY * 0.75);

            auto inOpponentGoalMouth = [&](const Pose2D &pose) {
                return pose.x > fd.length / 2.0 - goalMouthGuardX
                    && fabs(pose.y) < fd.goalWidth / 2.0 + goalMouthGuardY;
            };

            auto segmentCrossesOpponentGoalMouth = [&](const Pose2D &from, const Pose2D &to) {
                if (inOpponentGoalMouth(from)) return false;
                return segmentIntersectsRect(
                    from,
                    to,
                    fd.length / 2.0 - goalMouthGuardX,
                    fd.length / 2.0 + 0.70,
                    -fd.goalWidth / 2.0 - goalMouthGuardY,
                    fd.goalWidth / 2.0 + goalMouthGuardY);
            };

            auto pointUsable = [&](const Pose2D &pose) {
                return pose.x >= minX
                    && pose.x <= maxX
                    && pose.y >= minY
                    && pose.y <= maxY
                    && poseBallDist(pose) >= routeClearance
                    && !inOpponentGoalMouth(pose);
            };

            auto directSegmentUsable = [&](const Pose2D &from, const Pose2D &to) {
                return segmentMinDistToBall(from, to) >= routeClearance
                    && directOpponentGoalKickPenaltyRouteAllowed(from, to)
                    && !segmentCrossesOpponentGoalFrontCorridor(from, to)
                    && !segmentCrossesOpponentGoalMouth(from, to);
            };
            auto directFastSegmentUsable = [&](const Pose2D &from, const Pose2D &to) {
                return segmentMinDistToBall(from, to) >= directRouteClearance
                    && directOpponentGoalKickPenaltyRouteAllowed(from, to)
                    && !segmentCrossesOpponentGoalFrontCorridor(from, to)
                    && !segmentCrossesOpponentGoalMouth(from, to);
            };

            Pose2D finalTarget = targetPose;
            finalTarget.x = cap(finalTarget.x, maxX, minX);
            finalTarget.y = cap(finalTarget.y, maxY, minY);
            if (isInsideOpponentGoalKickPenalty(finalTarget)) {
                finalTarget = forceOutsideOpponentGoalKickPenalty(finalTarget);
                finalTarget.x = cap(finalTarget.x, maxX, minX);
                finalTarget.y = cap(finalTarget.y, maxY, minY);
            }
            finalTarget.theta = atan2(ballPos.y - finalTarget.y, ballPos.x - finalTarget.x);
            const bool robotCanUseInnerExit =
                !isInsideOpponentGoalKickRoutePenalty(robotPose)
                || isInsideOpponentGoalKickInnerLane(robotPose);

            if (isInsideOpponentGoalFrontCorridor(robotPose)
                && commandOpponentGoalKickGoalFrontAvoidance(finalTarget)) {
                return true;
            }

            auto noTouchEscapeDir = [&]() {
                const Pose2D safetyBall = activeSafetyBallPose();
                double radialX = robotPose.x - safetyBall.x;
                double radialY = robotPose.y - safetyBall.y;
                double radialNorm = norm(radialX, radialY);
                if (radialNorm < 1e-4) {
                    radialX = -1.0;
                    radialY = 0.0;
                    radialNorm = 1.0;
                }
                radialX /= radialNorm;
                radialY /= radialNorm;

                double dirX = radialX;
                double dirY = radialY;
                if (dirX > -0.05) {
                    double tangentX = -radialY;
                    double tangentY = radialX;
                    if (tangentX > 0.0) {
                        tangentX = -tangentX;
                        tangentY = -tangentY;
                    }

                    double radialAssist = 0.0;
                    if (radialX < -1e-4) {
                        radialAssist = 0.35;
                    } else if (fabs(radialX) <= 1e-4) {
                        radialAssist = 0.25;
                    } else if (radialX > 1e-4) {
                        radialAssist = std::clamp((-0.05 - tangentX) / radialX, 0.20, 0.55);
                    }

                    dirX = tangentX + radialAssist * radialX;
                    dirY = tangentY + radialAssist * radialY;
                }

                double dirNorm = norm(dirX, dirY);
                if (dirNorm < 1e-6) {
                    dirX = 0.0;
                    dirY = radialY >= 0.0 ? 1.0 : -1.0;
                    dirNorm = 1.0;
                }
                return pair<double, double>{dirX / dirNorm, dirY / dirNorm};
            };

            const double directFinalPathMinBallDist = segmentMinDistToBall(robotPose, finalTarget);
            const bool finalDirectFastUsable =
                robotCanUseInnerExit
                && pointUsable(finalTarget)
                && directFastSegmentUsable(robotPose, finalTarget)
                && activeRobotBallDist >= contactClearance + 0.03;
            const bool finalDirectPreferred =
                robotCanUseInnerExit
                && pointUsable(finalTarget)
                && directFinalPathMinBallDist >= contactClearance + 0.10
                && directOpponentGoalKickPenaltyRouteAllowed(robotPose, finalTarget)
                && !segmentCrossesOpponentGoalFrontCorridor(robotPose, finalTarget)
                && !segmentCrossesOpponentGoalMouth(robotPose, finalTarget);

            if (finalDirectPreferred) {
                vector<Pose2D> planPoints = {robotPose, finalTarget};
                setFreekickPlan("goal_kick_retreat_direct_shortest",
                    planPoints,
                    finalTarget,
                    directFinalPathMinBallDist);
                commandToPose(finalTarget, true, opponentGoalKickRetreatSpeed(directFinalPathMinBallDist), true);
                return true;
            }

            if (robotCanUseInnerExit && !finalDirectFastUsable && activeRobotBallDist < escapeClearance + 0.02) {
                auto [escapeX, escapeY] = noTouchEscapeDir();
                const double clearRatio = std::clamp(
                    (activeRobotBallDist - 0.20) / max(0.25, escapeClearance - 0.20),
                    0.0,
                    1.0);
                const double escapeSpeed = min(
                    opponentGoalKickFastSpeed,
                    activeRobotBallDist < contactClearance + 0.10
                        ? 0.18 + 0.28 * clearRatio
                        : 0.40 + 0.35 * clearRatio);
                double escapeFieldVx = escapeX * escapeSpeed;
                double escapeFieldVy = escapeY * escapeSpeed;
                limitOpponentGoalKickFieldBounds(escapeFieldVx, escapeFieldVy, escapeSpeed, true);

                Pose2D nearDirectTarget = finalTarget;
                double nearDirectVx = nearDirectTarget.x - robotPose.x;
                double nearDirectVy = nearDirectTarget.y - robotPose.y;
                limitOpponentGoalKickNoApproach(nearDirectVx, nearDirectVy, escapeSpeed, false);
                const double nearDirectNorm = norm(nearDirectVx, nearDirectVy);
                const double skimStep = min(0.95, max(0.35, nearDirectNorm));
                if (nearDirectNorm > 1e-6) {
                    nearDirectTarget.x = cap(robotPose.x + nearDirectVx / nearDirectNorm * skimStep, maxX, minX);
                    nearDirectTarget.y = cap(robotPose.y + nearDirectVy / nearDirectNorm * skimStep, maxY, minY);
                    nearDirectTarget.theta = atan2(ballPos.y - nearDirectTarget.y, ballPos.x - nearDirectTarget.x);
                }
                const bool canSkimToTarget =
                    activeRobotBallDist >= contactClearance + 0.04
                    && nearDirectNorm > 0.08
                    && segmentMinDistToBall(robotPose, nearDirectTarget) >= contactClearance + 0.02
                    && !segmentCrossesOpponentGoalMouth(robotPose, nearDirectTarget);
                if (canSkimToTarget) {
                    vector<Pose2D> planPoints = {robotPose, nearDirectTarget, finalTarget};
                    setFreekickPlan("goal_kick_skim_direct", planPoints, nearDirectTarget, planMinDistToBall(planPoints));
                    commandToPose(finalTarget, true, opponentGoalKickFastSpeed, true);
                    return true;
                }

                const double escapeStep = activeRobotBallDist < contactClearance + 0.10 ? 0.36 : 0.58;
                const double escapeFieldNorm = norm(escapeFieldVx, escapeFieldVy);
                const double escapeDirX = escapeFieldNorm > 1e-6 ? escapeFieldVx / escapeFieldNorm : escapeX;
                const double escapeDirY = escapeFieldNorm > 1e-6 ? escapeFieldVy / escapeFieldNorm : escapeY;

                Pose2D escapeTarget;
                escapeTarget.x = cap(robotPose.x + escapeDirX * escapeStep, maxX, minX);
                escapeTarget.y = cap(robotPose.y + escapeDirY * escapeStep, maxY, minY);
                escapeTarget.theta = atan2(ballPos.y - escapeTarget.y, ballPos.x - escapeTarget.x);
                vector<Pose2D> planPoints = {robotPose, escapeTarget};
                setFreekickPlan("goal_kick_escape_ball_safe", planPoints, escapeTarget, planMinDistToBall(planPoints));
                commandFieldVelocity(
                    escapeFieldVx,
                    escapeFieldVy,
                    activeSafetyBallYaw() * defenseParams.fastExitTurnGain,
                    escapeSpeed);
                return true;
            }

            if (robotCanUseInnerExit && pointUsable(finalTarget) && directFastSegmentUsable(robotPose, finalTarget)) {
                vector<Pose2D> planPoints = {robotPose, finalTarget};
                const double directPathMinBallDist = planMinDistToBall(planPoints);
                setFreekickPlan(finalDirectFastUsable ? "goal_kick_retreat_direct_safe" : "goal_kick_retreat_line",
                    planPoints,
                    finalTarget,
                    directPathMinBallDist);
                commandToPose(finalTarget, true, opponentGoalKickRetreatSpeed(directPathMinBallDist), true);
                return true;
            }

            auto commandGoalKickCorridor = [&]() {
                const double sideSign = isInsideOpponentGoalKickRoutePenalty(robotPose)
                    ? opponentGoalKickOuterSideSign()
                    : opponentGoalAwayOrbitSign();
                const double laneAbsY = cap(
                    max(opponentPenaltyExitY + 0.05,
                        fabs(ballPos.y) + routeClearance + 0.10),
                    maxY,
                    0.0);
                if (laneAbsY < opponentPenaltyExitY + 0.02) return false;

                Pose2D corridor;
                corridor.x = cap(opponentPenaltyExitX - 0.22, maxX, minX);
                corridor.y = sideSign > 0.0 ? laneAbsY : -laneAbsY;
                corridor.y = cap(corridor.y, maxY, minY);
                corridor.theta = atan2(ballPos.y - corridor.y, ballPos.x - corridor.x);

                if (!pointUsable(corridor) || !directSegmentUsable(robotPose, corridor)) return false;

                vector<Pose2D> planPoints = {robotPose, corridor};
                if (directSegmentUsable(corridor, finalTarget)) {
                    planPoints.push_back(finalTarget);
                }
                setFreekickPlan("goal_kick_corridor_exit", planPoints, corridor, planMinDistToBall(planPoints));
                commandToPose(corridor, true, opponentGoalKickFastSpeed, true);
                return true;
            };

            if (commandGoalKickCorridor()) {
                return true;
            }

            const double currentAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
            const double targetAngle = atan2(finalTarget.y - ballPos.y, finalTarget.x - ballPos.x);
            const double angleDelta = toPInPI(targetAngle - currentAngle);
            const double preferredSign = fabs(angleDelta) > 0.08
                ? (angleDelta > 0.0 ? 1.0 : -1.0)
                : opponentGoalAwayOrbitSign();
            const double orbitRadius = max(
                routeClearance + 0.10,
                min(requiredBallDist + 0.35, max(activeRobotBallDist + 0.22, routeClearance + 0.10)));

            Pose2D bestWaypoint;
            double bestCost = 1e9;
            bool foundWaypoint = false;
            vector<double> signs = {preferredSign, -preferredSign};
            vector<double> steps = {0.38, 0.55, 0.75, 0.95, 1.20, 1.55, 1.90, 2.30};
            for (double sign : signs) {
                for (double step : steps) {
                    const double signedStep = sign * step;
                    Pose2D candidate;
                    candidate.x = ballPos.x + orbitRadius * cos(currentAngle + signedStep);
                    candidate.y = ballPos.y + orbitRadius * sin(currentAngle + signedStep);
                    candidate.x = cap(candidate.x, maxX, minX);
                    candidate.y = cap(candidate.y, maxY, minY);
                    candidate.theta = atan2(ballPos.y - candidate.y, ballPos.x - candidate.x);

                    if (!pointUsable(candidate)) continue;
                    if (!directSegmentUsable(robotPose, candidate)) continue;

                    const bool candidateToFinalClear = directSegmentUsable(candidate, finalTarget);
                    double cost =
                        norm(candidate.x - robotPose.x, candidate.y - robotPose.y)
                        + 0.35 * fabs(toPInPI(targetAngle - atan2(candidate.y - ballPos.y, candidate.x - ballPos.x)))
                        + 0.20 * norm(candidate.x - finalTarget.x, candidate.y - finalTarget.y);
                    if (sign != preferredSign) cost += 0.45;
                    if (candidate.x > robotPose.x + 0.03) cost += 2.0 + 3.0 * (candidate.x - robotPose.x);
                    if (!candidateToFinalClear) cost += 0.65;
                    if (cost < bestCost) {
                        bestWaypoint = candidate;
                        bestCost = cost;
                        foundWaypoint = true;
                    }
                }
            }

            if (foundWaypoint) {
                vector<Pose2D> planPoints = {robotPose, bestWaypoint};
                if (directSegmentUsable(bestWaypoint, finalTarget)) {
                    planPoints.push_back(finalTarget);
                }
                setFreekickPlan("goal_kick_orbit_ball_safe", planPoints, bestWaypoint, planMinDistToBall(planPoints));
                commandToPose(bestWaypoint, true, opponentGoalKickFastSpeed, true);
                return true;
            }

            auto [escapeX, escapeY] = noTouchEscapeDir();
            const double fallbackEscapeSpeed = min(opponentGoalKickRetreatSpeed(planMinDistToBall({robotPose, finalTarget})), 0.80);
            double fallbackFieldVx = escapeX * fallbackEscapeSpeed;
            double fallbackFieldVy = escapeY * fallbackEscapeSpeed;
            limitOpponentGoalKickFieldBounds(fallbackFieldVx, fallbackFieldVy, fallbackEscapeSpeed, true);
            const double fallbackFieldNorm = norm(fallbackFieldVx, fallbackFieldVy);
            const double fallbackDirX = fallbackFieldNorm > 1e-6 ? fallbackFieldVx / fallbackFieldNorm : escapeX;
            const double fallbackDirY = fallbackFieldNorm > 1e-6 ? fallbackFieldVy / fallbackFieldNorm : escapeY;

            Pose2D escapeTarget;
            escapeTarget.x = cap(robotPose.x + fallbackDirX * 0.75, maxX, minX);
            escapeTarget.y = cap(robotPose.y + fallbackDirY * 0.75, maxY, minY);
            escapeTarget.theta = atan2(ballPos.y - escapeTarget.y, ballPos.x - escapeTarget.x);
            vector<Pose2D> planPoints = {robotPose, escapeTarget};
            setFreekickPlan("goal_kick_no_touch_escape", planPoints, escapeTarget, planMinDistToBall(planPoints));
            commandFieldVelocity(
                fallbackFieldVx,
                fallbackFieldVy,
                motionBallYawToRobot * defenseParams.fastExitTurnGain,
                fallbackEscapeSpeed);
            return true;
        };

        if (opponentGoalKickDefense && commandOpponentGoalKickDefenseRetreat()) {
            return NodeStatus::RUNNING;
        }

        Pose2D moveTarget = targetPose;
        bool fastMove = false;
        double fastMoveLimit = 0.0;
        string planStage = "defense_target";
        const bool robotInOpponentGoalFront = isInsideOpponentGoalFrontCorridor(robotPose);

        if (opponentGoalKickDefense
            && robotInOpponentGoalFront
            && !goalLineException
            && robotBallDist < contactExitRadius) {
            commandLeaveBallContact();
            return NodeStatus::RUNNING;
        }

        if (opponentGoalKickDefense && isInsideOpponentGoalKickPenalty(robotPose)) {
            if (!robotInOpponentGoalFront && !goalLineException && robotBallDist < contactExitRadius) {
                if (commandOpponentGoalKickOrbitAwayFromGoal()) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }

            if (!robotInOpponentGoalFront && commandOpponentGoalKickSafeSideExit()) {
                return NodeStatus::RUNNING;
            }

            Pose2D exitPose = forceOutsideOpponentGoalKickPenalty(robotPose);
            if (robotInOpponentGoalFront) {
                exitPose = robotPose;
                exitPose.x = cap(opponentPenaltyExitX - 0.18, safeFieldMaxX, safeFieldMinX);
                exitPose.y = cap(exitPose.y, safeFieldMaxY, safeFieldMinY);
            }
            exitPose.theta = atan2(ballPos.y - exitPose.y, ballPos.x - exitPose.x);

            const double exitBallClearance = opponentGoalKickNoTouchRadius;
            const bool directExitUsable =
                poseLegal(exitPose, exitBallClearance)
                && directOpponentGoalKickPenaltyRouteAllowed(robotPose, exitPose)
                && pathKeepsBallClear(exitPose, opponentGoalKickPathClearance);
            if (!directExitUsable) {
                const double awayAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
                const double exitRadius = max(opponentGoalKickNoTouchRadius, robotBallDist + 0.25);
                exitPose = bestCirclePose(awayAngle, exitRadius, forceOutsideOpponentGoalKickPenalty(targetPose));
            }

            vector<Pose2D> planPoints = {robotPose, exitPose};
            const double exitPathMinBallDist = planMinDistToBall(planPoints);
            if (poseBallDist(exitPose) < opponentGoalKickNoTouchRadius
                || exitPathMinBallDist < opponentGoalKickPathClearance) {
                setFreekickPlan("goal_kick_exit_blocked_by_ball", planPoints, exitPose, exitPathMinBallDist);
                if (commandOpponentGoalKickFallbackRetreat("goal_kick_exit_fallback_retreat")) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }

            if (shouldRouteAroundOpponentGoalFront(robotPose, exitPose)) {
                if (commandOpponentGoalKickGoalFrontAvoidance(exitPose)) {
                    return NodeStatus::RUNNING;
                }
                setFreekickPlan("goal_front_blocked", planPoints, exitPose, exitPathMinBallDist);
                if (commandOpponentGoalKickFallbackRetreat("goal_front_fallback_retreat")) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }

            setFreekickPlan(robotInOpponentGoalFront ? "exit_opponent_goal_front" : "exit_opponent_penalty",
                planPoints,
                exitPose,
                exitPathMinBallDist);
            commandToPose(exitPose, true, opponentGoalKickFastSpeed, true);
            return NodeStatus::RUNNING;
        }

        if (opponentGoalKickDefense && robotInOpponentGoalFront) {
            vector<Pose2D> planPoints = {robotPose, targetPose};
            setFreekickPlan("exit_opponent_goal_front", planPoints, targetPose, planMinDistToBall(planPoints));
            commandToPose(targetPose, true, opponentGoalKickFastSpeed, true);
            return NodeStatus::RUNNING;
        }

        if (!goalLineException && robotBallDist < contactExitRadius) {
            if (commandOpponentGoalKickOrbitAwayFromGoal()) {
                return NodeStatus::RUNNING;
            }
            commandLeaveBallContact();
            return NodeStatus::RUNNING;
        }

        const double currentNearBallAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
        const double interiorAngle = atan2(fieldInwardY, fieldInwardX);
        const double nearBallInteriorAngleError = fabs(toPInPI(interiorAngle - currentNearBallAngle));
        const bool fastExitReady = robotInsideField || robotOnInfieldSideOfBall;
        if (!goalLineException
            && robotBallDist < requiredBallDist
            && !fastExitReady
            && nearBallInteriorAngleError > nearBallAngleTolerance
            && commandOrbitToFieldInteriorSide()) {
            return NodeStatus::RUNNING;
        }

        if (!robotInsideField && !(mustFastExit && fastExitReady)) {
            auto [hasSafeEntryPose, safeEntryPose] = safeEnterFieldPose();
            if (!hasSafeEntryPose) {
                vector<Pose2D> planPoints = {robotPose, safeEntryPose};
                setFreekickPlan("entry_blocked", planPoints, safeEntryPose, planMinDistToBall(planPoints));
                if (commandOpponentGoalKickFallbackRetreat("entry_fallback_retreat")) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }
            moveTarget = safeEntryPose;
            vector<Pose2D> planPoints = {robotPose, moveTarget};
            setFreekickPlan("enter_field", planPoints, moveTarget, planMinDistToBall(planPoints));
            commandToPose(moveTarget, true, defenseParams.boundaryArcSpeed, false);
            return NodeStatus::RUNNING;
        }

        if (!goalLineException
            && robotBallDist < requiredBallDist + max(0.25, defenseParams.boundaryArcRadiusMargin + defenseParams.fastExitReleaseMargin)
            && commandOpponentGoalKickOrbitAwayFromGoal()) {
            return NodeStatus::RUNNING;
        }

        if (shouldRouteAroundOpponentGoalFront(robotPose, moveTarget)) {
            Pose2D avoidPose = opponentGoalKickGoalFrontAvoidancePose(robotPose, moveTarget);
            const bool avoidUsable =
                !isInsideOpponentGoalFrontCorridor(avoidPose)
                && !isInsideOpponentGoalKickPenalty(avoidPose)
                && poseLegal(avoidPose, max(contactClearance, requiredBallDist + 0.03))
                && pathKeepsBallClear(avoidPose, max(contactClearance, defenseParams.fastExitPathBallClearance));
            if (avoidUsable) {
                moveTarget = avoidPose;
                fastMove = true;
                fastMoveLimit = max(defenseParams.boundaryArcSpeed, defenseParams.fastExitBackSpeed);
                planStage = "avoid_opponent_goal_front";
            } else {
                vector<Pose2D> planPoints = {robotPose, moveTarget};
                setFreekickPlan("goal_front_blocked", planPoints, moveTarget, planMinDistToBall(planPoints));
                if (commandOpponentGoalKickFallbackRetreat("goal_front_fallback_retreat")) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }
        }

        if (opponentGoalKickDefense && segmentCrossesOpponentGoalKickPenalty(robotPose, moveTarget)) {
            Pose2D cornerPose = opponentGoalKickPenaltyCornerPose(robotPose, moveTarget);
            const bool cornerUsable =
                poseLegal(cornerPose, max(contactClearance, requiredBallDist + 0.03))
                && pathKeepsBallClear(cornerPose, max(contactClearance, defenseParams.fastExitPathBallClearance));
            if (cornerUsable) {
                moveTarget = cornerPose;
                fastMove = true;
                fastMoveLimit = max(defenseParams.boundaryArcSpeed, defenseParams.fastExitBackSpeed);
                planStage = "avoid_opponent_penalty";
            } else if (commandOpponentGoalKickFallbackRetreat("opponent_penalty_fallback_retreat")) {
                return NodeStatus::RUNNING;
            }
        }

        const double currentAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
        const double targetAngle = atan2(targetPose.y - ballPos.y, targetPose.x - ballPos.x);
        const double angleDelta = toPInPI(targetAngle - currentAngle);
        const double targetBallDist = max(requiredBallDist + 0.10, poseBallDist(targetPose));
        const bool directPathSafe =
            poseLegal(targetPose, requiredBallDist + 0.03)
            && poseOnDefenseSide(targetPose)
            && obstacleClearTo(targetPose)
            && pathKeepsBallClear(targetPose, max(contactClearance, requiredBallDist - 0.05));

        if (mustFastExit && fastExitReady) {
            const double distToOptimal = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
            const bool nearlyOutsideLegalRadius = robotBallDist >= requiredBallDist - 0.20;
            const bool sameExitLane = fabs(angleDelta) < max(0.35, defenseParams.boundaryArcStep * 0.75);
            const bool optimalFastUsable =
                nearlyOutsideLegalRadius
                && (distToOptimal < 1.50 || sameExitLane)
                && poseLegal(targetPose, requiredBallDist + 0.03)
                && poseOnDefenseSide(targetPose)
                && obstacleClearTo(targetPose)
                && pathKeepsBallClear(targetPose, max(contactClearance, requiredBallDist - 0.12));

            if (optimalFastUsable) {
                moveTarget = targetPose;
                planStage = robotInsideField ? "fast_exit_direct" : "fast_exit_infield_side";
            } else {
                double awayAngle = atan2(robotPose.y - ballPos.y, robotPose.x - ballPos.x);
                moveTarget = bestCirclePose(awayAngle, requiredBallDist + 0.10, targetPose);
                planStage = robotInsideField ? "fast_exit_radius" : "fast_exit_infield_side";
            }
            fastMove = true;
            fastMoveLimit = max(defenseParams.fastExitBackSpeed, defenseParams.boundaryArcSpeed);
        } else {
            const bool shouldArcAlongBoundary =
                !goalLineException
                && robotBallDist >= requiredBallDist - 0.02
                && dist > defenseParams.arriveDistTolerance
                && (!robotOnDefenseSide
                    || fabs(angleDelta) > defenseParams.boundaryArcMinAngle
                    || !directPathSafe
                    || dist > defenseParams.boundaryArcDirectDist);

            if (shouldArcAlongBoundary) {
                const double arcStep = fabs(angleDelta) > defenseParams.boundaryArcMinAngle
                    ? cap(angleDelta, defenseParams.boundaryArcStep, -defenseParams.boundaryArcStep)
                    : 0.0;
                if (fabs(arcStep) < 1e-6 && directPathSafe) {
                    moveTarget = targetPose;
                } else {
                    const double arcRadius = fabs(arcStep) < 1e-6
                        ? targetBallDist
                        : max(
                            requiredBallDist + defenseParams.boundaryArcRadiusMargin,
                            min(robotBallDist, targetBallDist));
                    moveTarget = bestCirclePose(currentAngle + arcStep, arcRadius, targetPose);
                }
                if (!obstacleClearTo(moveTarget) || !pathKeepsBallClear(moveTarget, max(contactClearance, requiredBallDist - 0.03))) {
                    const double cautiousRadius = max(
                        requiredBallDist + defenseParams.boundaryArcRadiusMargin + 0.15,
                        robotBallDist + 0.10);
                    moveTarget = bestCirclePose(currentAngle + arcStep * 0.55, cautiousRadius, targetPose);
                }
                fastMove = true;
                fastMoveLimit = defenseParams.boundaryArcSpeed;
                planStage = "arc_around_ball";
            } else if (!goalLineException
                && directPathSafe
                && dist > defenseParams.arriveDistTolerance) {
                moveTarget = targetPose;
                fastMove = true;
                fastMoveLimit = defenseParams.boundaryArcSpeed;
                planStage = "fast_defense_target";
            }
        }

        if (shouldRouteAroundOpponentGoalFront(robotPose, moveTarget)) {
            Pose2D avoidPose = opponentGoalKickGoalFrontAvoidancePose(robotPose, moveTarget);
            const bool avoidUsable =
                !isInsideOpponentGoalFrontCorridor(avoidPose)
                && !isInsideOpponentGoalKickPenalty(avoidPose)
                && poseLegal(avoidPose, max(contactClearance, requiredBallDist + 0.03))
                && pathKeepsBallClear(avoidPose, max(contactClearance, defenseParams.fastExitPathBallClearance));
            if (avoidUsable) {
                moveTarget = avoidPose;
                fastMove = true;
                fastMoveLimit = max(defenseParams.boundaryArcSpeed, defenseParams.fastExitBackSpeed);
                planStage = "avoid_opponent_goal_front";
            } else {
                vector<Pose2D> planPoints = {robotPose, moveTarget};
                setFreekickPlan("goal_front_blocked", planPoints, moveTarget, planMinDistToBall(planPoints));
                if (commandOpponentGoalKickFallbackRetreat("goal_front_fallback_retreat")) {
                    return NodeStatus::RUNNING;
                }
                commandLeaveBallContact();
                return NodeStatus::RUNNING;
            }
        }

        if (opponentGoalKickDefense && segmentCrossesOpponentGoalKickPenalty(robotPose, moveTarget)) {
            Pose2D cornerPose = opponentGoalKickPenaltyCornerPose(robotPose, moveTarget);
            const bool cornerUsable =
                poseLegal(cornerPose, max(contactClearance, requiredBallDist + 0.03))
                && pathKeepsBallClear(cornerPose, max(contactClearance, defenseParams.fastExitPathBallClearance));
            if (cornerUsable) {
                moveTarget = cornerPose;
                fastMove = true;
                fastMoveLimit = max(defenseParams.boundaryArcSpeed, defenseParams.fastExitBackSpeed);
                planStage = "avoid_opponent_penalty";
            } else if (commandOpponentGoalKickFallbackRetreat("opponent_penalty_fallback_retreat")) {
                return NodeStatus::RUNNING;
            }
        }

        vector<Pose2D> planPoints = {robotPose, moveTarget};
        setFreekickPlan(planStage, planPoints, moveTarget, planMinDistToBall(planPoints));
        commandToPose(moveTarget, fastMove, fastMoveLimit);
        return NodeStatus::RUNNING;
    }

    if ( // 认为到达了目标位置
        dist < 0.3
        && fabs(deltaDir) < 0.15
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 获取避障开关参数
    static bool adjust_freekick_cached = false;
    static bool enable_freekick_avoid = false;
    if (!adjust_freekick_cached) {
        enable_freekick_avoid = brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool();
        adjust_freekick_cached = true;
    }

    if (!enable_freekick_avoid || dist < 1.5 || _isInFinalAdjust) {
        _isInFinalAdjust = true;
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 2.0;

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3));
        vx *= linearFactor;
        vy *= linearFactor;

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
        vx = cap(vx, vxLimit, -0.4);
        vy = cap(vy, vyLimit, -vyLimit);

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.4;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.3, 0.3, 0.2, avoidObstacle);

    return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
    brain->client->setVelocity(0, 0, 0);
    _defenseTargetLocked = false;
    _opponentGoalKickBallFrozen = false;
    brain->tree->setEntry<bool>("local_freekick_target_valid", false);
    brain->tree->setEntry<double>("local_freekick_target_error", 999.0);
    brain->tree->setEntry<bool>("local_freekick_plan_valid", false);
    brain->tree->setEntry<int>("local_freekick_plan_count", 0);
    brain->data->localFreekickTargetUpdateTime = brain->get_clock()->now();
}

NodeStatus GoToGoalBlockingPosition::tick() {
    
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
            : (ballPos.y > 0 ? fd.goalWidth / 4.0 : -fd.goalWidth / 4.0);
    } else {
        targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
        targetPose.y = curRole == "striker" ? (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth/ 2.0, -fd.penaltyAreaWidth / 2.0));
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    brain->tree->setEntry<bool>("local_freekick_target_valid", true);
    brain->tree->setEntry<double>("local_freekick_target_error", dist);
    brain->data->localFreekickTargetUpdateTime = brain->get_clock()->now();
    if ( // Considered to have reached the target position
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);    
    vy = cap(vy, vyLimit, -vyLimit);    
    

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    auto log = [=](string msg) {
        brain->log->debug("Assist", msg);
    };
    log("ticked");

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;
    string curRole = brain->tree->getEntry<string>("player_role");

    bool isSecondary = false; 
    bool has2Assists = false;
    int selfIdx = brain->config->get_player_id() - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue; 
        if (tmStatus.isLead) continue; 
        if (tmStatus.role != "striker") continue; 

        has2Assists = true;
        log("2 assists found");
        if (tmStatus.robotPoseToField.x > robotPose.x) {
            log("i am secondary");
            isSecondary = true; 
        }
    }
    log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


    Pose2D targetPose;
    targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
    targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); 
    targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); 
    if (has2Assists) { 
        targetPose.y += isSecondary ? - 0.5 : 0.5;
    }


    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( 
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

    bool avoidObstacle = brain->config->get_avoid_during_chase();
    double oaSafeDist = brain->config->get_chase_ao_safe_dist();

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
        vtheta = brain->data->ball.yawToRobot * 4.0; 
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -1.0);     
    vy = cap(vy, vyLimit, -vyLimit);     
    

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    auto log = [=](string msg) { 
        brain->log->debug("adjust5", msg); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double st = st_far; 
    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, 0); 
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    vtheta = ballYaw;
    vtheta *= vtheta_factor; 

    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.; 
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1;

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;

    if (brain->data->isFreekickKickingOff && !brain->data->isDirectShoot && !brain->isDefensing()) {
        brain->data->kickType = "cross";
        if (bPos.x > 0.0) {
            const double passTargetX = bPos.x;
            const double passTargetY = 0.0;
            brain->data->kickDir = atan2(
                passTargetY - bPos.y,
                passTargetX - bPos.x
            );
        } else {
            brain->data->kickDir = atan2(
                - bPos.y,
                fd.length / 2.0 - bPos.x
            );
        }
    }
    else if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { 
        brain->data->kickType = "shoot";
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    }

    brain->log->log(
        "field/kick_dir",
        format("Kick direction: %.2f rad at ball position (%.2f, %.2f)", 
               brain->data->kickDir, brain->data->ball.posToField.x, brain->data->ball.posToField.y)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->debug("striker_decide", msg);
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
    } else if (
                brain->config->get_enable_auto_visual_kick() &&
                brain->data->tmImLead && 
                brain->data->tmMyCostRank == 0 && 
                !brain->tree->getEntry<bool>("ball_out") && 
                brain->data->lose_ball == false &&
                brain->data->tmMyCost < 7.0 &&
                brain->data->ball.range < brain->config->get_auto_visual_kick_enable_dist_max() &&
                brain->data->ball.range > brain->config->get_auto_visual_kick_enable_dist_min() &&
                fabs(brain->data->ball.yawToRobot) < brain->config->get_auto_visual_kick_enable_angle() * 1.3 &&
                brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
                fabs(brain->data->ball.posToField.y) < 5 &&
                brain->data->robotPoseToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
                fabs(brain->data->robotPoseToField.y) < 5 
            ) {
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
    } else if (!brain->data->tmImLead) {
        newDecision = "assist";
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    } else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else newDecision = "kick";      
        if (newDecision == "kick") brain->data->isFreekickKickingOff = false;
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);
    
    // Publish player_decide message
    brain->visualizer->publishPlayerDecision(format("striker-%s", newDecision.c_str()));
    
    // Publish decision information through visualization_publisher
    auto decision_marker = brain->visualizer->createDecisionInfoMarker(
        "striker",
        newDecision,
        ballRange,
        ballYaw,
        kickDir,
        dir_rb_f,
        angleGoodForKick,
        brain->data->tmImLead,
        "map"
    );
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(decision_marker);
    brain->visualizer->publishMarkers(marker_array);
    
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
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
    }
    else
    {
        newDecision = "adjust";
    }

    setOutput("decision_out", newDecision);
    
    // Publish player_decide message
    brain->visualizer->publishPlayerDecision(format("goalie-%s", newDecision.c_str()));
    
    // Publish decision information through visualization_publisher
    auto decision_marker = brain->visualizer->createDecisionInfoMarker(
        "goalie",
        newDecision,
        ballRange,
        ballYaw,
        kickDir,
        dir_rb_f,
        angleIsGood,
        false,  // goalie does not need is_lead information
        "map"
    );
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(decision_marker);
    brain->visualizer->publishMarkers(marker_array);
    
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->get_vx_factor();   
    double yawOffset = brain->config->get_yaw_offset(); 


    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

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
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();


    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
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

    // Publish movement command
    double angle = brain->data->ball.yawToRobot;
    double speed = _speed;
    if (brain->config->get_soft_kickoff() &&
        (brain->data->isFreekickKickingOff || brain->data->isKickingOff)) {
        speed = std::min(speed, brain->config->get_soft_kickoff_speed());
    }
    brain->client->crabWalk(angle, speed);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    auto log = [=](string msg) {
        brain->log->debug("Kick", msg);
    };


    bool enableAbort = brain->config->get_abort_kick_when_ball_moved();
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }


    if (ballRange < _minRange) _minRange = ballRange;    

    
    bool avoidPushing = brain->config->get_avoid_during_kick();
    double kickAoSafeDist = brain->config->get_kick_ao_safe_dist();
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }


    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value();
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }


    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.1; 
        speed = min(speed, _speed);
        if (brain->config->get_soft_kickoff() &&
            (brain->data->isFreekickKickingOff || brain->data->isKickingOff)) {
            speed = std::min(speed, brain->config->get_soft_kickoff_speed());
        }
        brain->client->crabWalk(angle, speed);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}


// Static variable definition
rclcpp::Time RLVisionKick::_lastExitTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

NodeStatus RLVisionKick::onStart()
{
    _startTime = brain->get_clock()->now();
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
    
    // Start deceleration
    startDecelerate(500.0);
    stepDecelerate();
    
    return NodeStatus::RUNNING;
}

NodeStatus RLVisionKick::onRunning()
{
    // Check exit flag
    if (brain->data->shouldExitRLVisionKick) {
        brain->data->shouldExitRLVisionKick = false;
        brain->data->tmImInVisualKick = false;
        recordExitTime();
        return NodeStatus::SUCCESS;
    }
    
    // Handle deceleration phase
    if (_isDecelerating) {
        stepDecelerate();
        
        if (!_isDecelerating) {
            // Deceleration completed
            if (_pendingRobocupWalk) {
                brain->client->robocupWalk();
                _pendingRobocupWalk = false;
                return NodeStatus::SUCCESS;
            } else if (!_visionKickStarted) {
                // Start vision kick after deceleration
                brain->client->RLVisionKick();
                _headScanStartTime = brain->get_clock()->now();
                _visionKickStarted = true;
            }
        }
        return NodeStatus::RUNNING;
    }
    
    if (_visionKickStarted) {
        double headMsec = brain->msecsSince(_headScanStartTime);
        if (headMsec < 300.0) {
            brain->client->moveHead(0.4, 0.0);
        } else if (headMsec < 550.0) {
            brain->client->moveHead(0.7, 0.0);
        }
    }

    // Check exit conditions
    double elapsed = brain->msecsSince(_startTime);
    double minMsecKick = getInput<double>("min_msec_kick").value();
    
    // Check if ball is too far or cost is too high
    bool ballTooFar = brain->data->ballDetected && brain->data->ball.range > 5.0;
    bool shouldExit = (((ballTooFar || brain->data->tmMyCost > 8.0) && (elapsed > minMsecKick)) || brain->data->lose_ball || brain->tree->getEntry<bool>("ball_out"));
    
    if (shouldExit) {
        recordExitTime();
        startDecelerate(500.0);
        _pendingRobocupWalk = true;
        stepDecelerate();
        return NodeStatus::RUNNING;
    }

    return NodeStatus::RUNNING;
}

void RLVisionKick::onHalted()
{
    brain->data->tmImInVisualKick = false;
    brain->client->setVelocity(0.0, 0.0, 0.0);
    brain->client->robocupWalk();
    recordExitTime();
    
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
}

bool RLVisionKick::isMinIntervalSatisfied(double minIntervalMsec)
{
    // This is a static method, so we can't access brain instance
    // For now, always return true. If needed, pass brain pointer as parameter
    return true;
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
    brain->client->setVelocity(0.0, 0.0, 0.0);
    
    if (elapsed >= _decelDurationMs) {
        _isDecelerating = false;
        return true;
    }
    
    return false;
}

NodeStatus StandStill::onStart()
{

    _startTime = brain->get_clock()->now();


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

    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
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
        _angle = fabs(_angle) * (ballPixX < brain->config->cameraImageWidth / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
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
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick()
{
    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    string role = brain->tree->getEntry<string>("player_role");
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

    if (role == "striker") {
        if (brain->data->myStrikerIDRank == 0) {
            tx = isKickoff ? - fd.circleRadius - 0.5 : - fd.circleRadius * 2;
            ty = 0.0;
        } else if (brain->data->myStrikerIDRank == 1) {
            tx = isKickoff ? - fd.circleRadius - 0.5 : - fd.circleRadius * 2;
            ty = -1.5;
        } else if (brain->data->myStrikerIDRank == 2) {
            tx = - fd.length / 2.0 + fd.penaltyAreaLength;
            ty = fd.circleRadius / 2.0;
        } else if (brain->data->myStrikerIDRank == 3) {
            tx = - fd.length / 2.0 + fd.penaltyDist;
            ty = - fd.circleRadius / 2.0;
        }
    } else if (role == "goal_keeper") {
        tx = -fd.length / 2.0 + fd.goalAreaLength;
        ty = 0;
        ttheta = 0;
    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->debug("GoBackInField", msg);
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
    else { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus RobocupWalk::tick()
{
    // Intended to be wrapped by a RunOnce decorator at behavior-tree startup.
    brain->client->changeRobocupMode();
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
        brain->log->debug("recovery", "reset recovery");
        return NodeStatus::SUCCESS;
    }
    brain->log->debug("recovery", format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        brain->data->currentRobotModeIndex == 1 && 
        brain->data->recoveryPerformedRetryCount < brain->config->get_retry_max_count()) {
        brain->data->shouldExitRLVisionKick = true;
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->log->debug("recovery", format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 10) {
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->debug("recovery", format("Add retry count: %d", brain->data->recoveryPerformedRetryCount));
    }


    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        (brain->data->currentRobotModeIndex == 8 || brain->data->currentRobotModeIndex == 20)) { 
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->data->shouldExitRLVisionKick = false;
        brain->log->debug("recovery", "Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState)));
    }

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
