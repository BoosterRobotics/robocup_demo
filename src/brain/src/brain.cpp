#include <iostream>
#include <algorithm>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <string>
#include <fstream>  // 添加这一行
#include <iomanip>
#include <limits>
#include <sstream>
#include <cctype>
#include <yaml-cpp/yaml.h>  // 添加这一行

#include "brain.h"
#include "utils/kick_geometry.h"
#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

namespace
{
bool extractJsonIntField(const std::string &json, const std::string &field, int *value)
{
    if (value == nullptr)
        return false;

    const std::string key = "\"" + field + "\"";
    size_t pos = json.find(key);
    if (pos == std::string::npos)
        return false;

    pos = json.find(':', pos + key.size());
    if (pos == std::string::npos)
        return false;

    ++pos;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
        ++pos;
    if (pos < json.size() && json[pos] == '"')
        ++pos;

    const size_t start = pos;
    if (pos < json.size() && (json[pos] == '-' || json[pos] == '+'))
        ++pos;
    while (pos < json.size() && std::isdigit(static_cast<unsigned char>(json[pos])))
        ++pos;

    if (pos == start ||
        (pos == start + 1 && (json[start] == '-' || json[start] == '+')))
        return false;

    try {
        *value = std::stoi(json.substr(start, pos - start));
    } catch (const std::exception &) {
        return false;
    }
    return true;
}

double pitchToGroundObject(double robotHeight, double horizontalRange)
{
    // GameObject::range is the XY-plane distance, not the camera-to-object
    // hypotenuse. atan2 is therefore the correct geometry and stays defined
    // for a ground object directly below the robot (horizontalRange == 0).
    return std::atan2(robotHeight, horizontalRange);
}
} // namespace

Brain::Brain() : rclcpp::Node("brain_node")
{
    // 初始化tf广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    // 要注意参数必须先在这里声明，否则程序里也读不到
    // 配置在 yaml 文件中的参数，如果有层级结构，用点分号来获取

    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "left");
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 2);

    declare_parameter<double>("ball_predictor.step_interval", 100.0);
    declare_parameter<int>("ball_predictor.step_cnt", 50);
    declare_parameter<double>("ball_predictor.acceleration", -0.35);
    declare_parameter<double>("ball_predictor.speed_friction_factor", 0.45);

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.5);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.0);
    declare_parameter<double>("robot.vy_limit", 0.4);
    declare_parameter<double>("robot.vtheta_limit", 1.0);

    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);
    declare_parameter<double>("strategy.ball_confidence_decay_rate", 3.0);
    declare_parameter<bool>("strategy.enable_stable_kick", false);
    declare_parameter<double>("strategy.ball_memory_timeout", 3.0);
    declare_parameter<double>("strategy.ball_lost_msecs", 1000.0);
    declare_parameter<double>("strategy.tm_ball_dist_threshold", 3.0);
    declare_parameter<bool>("strategy.limit_near_ball_speed", true);
    declare_parameter<double>("strategy.near_ball_speed_limit", 0.3);
    declare_parameter<double>("strategy.near_ball_range", 4.0);
    declare_parameter<bool>("strategy.soft_kickoff", true);
    declare_parameter<double>("strategy.soft_kickoff_speed", 0.3);
    declare_parameter<double>("strategy.kick_range", 1.0);
    declare_parameter<double>("strategy.kick_theta_range", 0.2);
    declare_parameter<bool>("strategy.abort_kick_when_ball_moved", false);
    declare_parameter<double>("strategy.abort_kick_ball_move_threshold", 0.3);
    declare_parameter<bool>("strategy.abort_shoot_when_ball_moved", true);
    declare_parameter<bool>("strategy.limit_speed_near_border", false);
    declare_parameter<double>("strategy.near_border_speed_limit", 0.5);
    declare_parameter<double>("strategy.near_border_dist_threshold", -1.0);
    declare_parameter<bool>("strategy.enable_bypass", false);
    declare_parameter<bool>("strategy.enable_shoot", false);
    declare_parameter<bool>("strategy.enable_directional_kick", false);

    declare_parameter<double>("strategy.goalie_visual_kick_cooldown_ms", 600.0);
    declare_parameter<double>("strategy.goalie_visual_kick_max_ball_range", 5.0);
    declare_parameter<double>("strategy.goalie_visual_kick_max_direction_error", 1.2);
    declare_parameter<double>("strategy.goalie_simple_x_kp", 0.9);
    declare_parameter<double>("strategy.goalie_simple_y_kp", 2.2);
    declare_parameter<double>("strategy.goalie_simple_threat_vxf", -0.03);
    declare_parameter<double>("strategy.goalie_simple_predict_min_vxf", 0.06);
    declare_parameter<double>("strategy.goalie_simple_t_goal_max", 8.0);
    declare_parameter<double>("strategy.goalie_simple_t_plane_min", 0.02);
    declare_parameter<double>("strategy.goalie_simple_t_plane_max", 6.0);
    declare_parameter<double>("strategy.goalie_simple_vy_max", 0.8);
    declare_parameter<double>("strategy.goalie_simple_vx_max", 1.2);
    declare_parameter<double>("strategy.goalie_simple_vtheta_kp", 1.4);
    declare_parameter<double>("strategy.goalie_simple_vtheta_limit", 1.5);
    declare_parameter<bool>("strategy.enable_auto_visual_kick", false);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_min", 0.2);
    declare_parameter<double>("strategy.auto_visual_kick_enable_dist_max", 4.0);
    declare_parameter<double>("strategy.auto_visual_kick_enable_angle", 0.8);
    declare_parameter<double>("strategy.auto_visual_kick_max_direction_error", 0.35);
    declare_parameter<double>("strategy.auto_visual_kick.real_ball_hold_ms", 1000.0);
    declare_parameter<double>("strategy.auto_visual_kick.current_real_ball_max_age_ms", 650.0);
    declare_parameter<double>("strategy.auto_visual_kick.near_ball_memory_reject_range", 0.45);
    declare_parameter<bool>("strategy.auto_visual_kick.allow_predicted_kick_msg", true);
    declare_parameter<bool>("strategy.auto_visual_kick.allow_memory_kick_msg", true);
    declare_parameter<double>("strategy.auto_visual_kick.kick_msg_real_ball_max_age_ms", 650.0);
    declare_parameter<double>("strategy.auto_visual_kick.no_detection_kick_msg_reject_range", 0.45);
    declare_parameter<double>("strategy.auto_visual_kick.startup_decelerate_ms", 80.0);
    declare_parameter<double>("strategy.auto_visual_kick.exit_decelerate_ms", 140.0);
    declare_parameter<int>("strategy.auto_visual_kick.max_parallel_count", 1);
    declare_parameter<double>("strategy.auto_visual_kick.yield_my_cost_min", 4.0);
    declare_parameter<double>("strategy.auto_visual_kick.no_progress_timeout_ms", 3500.0);
    declare_parameter<double>("strategy.auto_visual_kick.progress_range_delta", 0.12);
    declare_parameter<double>("strategy.auto_visual_kick.progress_ball_forward_delta", 0.15);
    declare_parameter<double>("strategy.auto_visual_kick.max_total_ms", 14000.0);
    declare_parameter<int>("strategy.auto_visual_kick.max_extra_rounds", 1);
    declare_parameter<bool>("strategy.power_shoot.enable", false);
    declare_parameter<bool>("strategy.power_shoot.use_for_kickoff", false);
    declare_parameter<double>("strategy.power_shoot.xmin", 0.5);
    declare_parameter<double>("strategy.power_shoot.xmax", 1.0);
    declare_parameter<double>("strategy.power_shoot.ymin", -0.5);
    declare_parameter<double>("strategy.power_shoot.ymax", 0.5);
    declare_parameter<double>("strategy.shoot.threat_threshold", 0.0);
    declare_parameter<double>("strategy.shoot.xmin", 0.5);
    declare_parameter<double>("strategy.shoot.xmax", 1.0);
    declare_parameter<double>("strategy.shoot.ymin", -0.5);
    declare_parameter<double>("strategy.shoot.ymax", 0.5);
    declare_parameter<bool>("strategy.cooperation.enable_role_switch", true);
    declare_parameter<double>("strategy.cooperation.ball_control_cost_threshold", 10.0);
    declare_parameter<double>("strategy.cooperation.goalie_comm_fresh_msecs", 2200.0);
    declare_parameter<double>("strategy.cooperation.goalie_comm_stable_msecs", 1000.0);
    declare_parameter<double>("strategy.cooperation.goalie_distance_tie_margin", 0.15);
    declare_parameter<bool>("strategy.cooperation.goalie_fallback_use_gc_goalie", true);
    declare_parameter<bool>("strategy.cooperation.goalie_attack_handoff", true);
    declare_parameter<bool>("strategy.far_set_play_search.enable", true);
    declare_parameter<double>("strategy.far_set_play_search.trigger_x", 0.0);
    declare_parameter<double>("strategy.far_set_play_search.field_target_x_ratio", 0.25);
    declare_parameter<double>("strategy.far_set_play_search.lane_y_ratio", 0.28);
    declare_parameter<double>("strategy.far_set_play_search.max_move_secs", 30.0);
    declare_parameter<double>("strategy.far_set_play_search.vx_limit", 0.7);
    declare_parameter<double>("strategy.far_set_play_search.vy_limit", 0.4);
    declare_parameter<double>("strategy.set_play_stand.corridor_half_width", 0.6);
    declare_parameter<double>("strategy.set_play_stand.corridor_forward_range", 3.0);
    declare_parameter<double>("strategy.set_play_stand.side_margin", 0.2);

    declare_parameter<double>("RLVisionKick.goalie_clearance_power", 2.5);

    declare_parameter<int>("obstacle_avoidance.depth_sample_step", 16);
    declare_parameter<double>("obstacle_avoidance.obstacle_min_height", 0.15);
    declare_parameter<double>("obstacle_avoidance.grid_size", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_x", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_y", 0.2);
    declare_parameter<double>("obstacle_avoidance.exclusion_x", 0.25);
    declare_parameter<double>("obstacle_avoidance.exclusion_y", 0.4);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_radius", 0.3);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_height", 0.3);
    declare_parameter<int>("obstacle_avoidance.occupancy_threshold", 5);
    declare_parameter<double>("obstacle_avoidance.collision_threshold", 0.5);
    declare_parameter<double>("obstacle_avoidance.safe_distance", 2.0);
    declare_parameter<double>("obstacle_avoidance.avoid_secs", 3.0);
    declare_parameter<bool>("obstacle_avoidance.enable_freekick_avoid", false);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_safe_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_avoid_secs", 1.5);
    declare_parameter<double>("obstacle_avoidance.obstacle_memory_msecs", 500.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_chase", false);
    declare_parameter<double>("obstacle_avoidance.chase_ao_safe_dist", 2.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_kick", false);
    declare_parameter<double>("obstacle_avoidance.kick_ao_safe_dist", 1.0);
    declare_parameter<bool>("obstacle_avoidance.kick_ao_use_shoot", false);
    declare_parameter<bool>("obstacle_avoidance.always_turn_left", false);
    
    declare_parameter<int>("locator.min_marker_count", 5);
    declare_parameter<double>("locator.max_residual", 0.3);

    declare_parameter<bool>("enable_com", false);

    declare_parameter<bool>("rerunLog.enable_tcp", false);
    declare_parameter<string>("rerunLog.server_ip", "");
    declare_parameter<bool>("rerunLog.enable_file", false);
    declare_parameter<string>("rerunLog.log_dir", "");
    declare_parameter<double>("rerunLog.max_log_file_mins", 5.0);
    declare_parameter<int>("rerunLog.img_interval", 10);

    declare_parameter<bool>("debug.log_ball_pos", false);
    declare_parameter<bool>("debug.low_frequency_status.enable", false);
    declare_parameter<bool>("debug.low_frequency_status.mode_enable", true);
    declare_parameter<bool>("debug.low_frequency_status.field_ascii_enable", true);
    declare_parameter<double>("debug.low_frequency_status.log_interval_ms", 2000.0);
    declare_parameter<double>("debug.low_frequency_status.mode_query_interval_ms", 2000.0);
    declare_parameter<double>("debug.low_frequency_status.mode_query_timeout_ms", 1000.0);
    declare_parameter<double>("debug.low_frequency_status.field_ascii_interval_ms", 5000.0);
    declare_parameter<double>("debug.low_frequency_status.field_ascii_char_aspect", 2.0);
    declare_parameter<int>("debug.low_frequency_status.field_ascii_width", 45);

    declare_parameter<bool>("sound.enable", false);
    declare_parameter<string>("sound.sound_pack", "espeak");

    declare_parameter<string>("vision.image_topic", "/camera/camera/color/image_raw");
    declare_parameter<string>("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<double>("vision.cam_pixel_width", 1280);
    declare_parameter<double>("vision.cam_pixel_height", 720);
    declare_parameter<double>("vision.cam_fov_x", 90);
    declare_parameter<double>("vision.cam_fov_y", 65);

    declare_parameter<string>("game_control_ip", "0.0.0.0");

    declare_parameter<string>("tree_file_path", "");
    declare_parameter<string>("vision_config_path", "");
    declare_parameter<string>("vision_config_local_path", "");

    declare_parameter<int>("recovery.retry_max_count", 2);
    declare_parameter<string>("recovery.get_up_version", "kV1");

    declare_parameter<string>("RLVisionKick.visual_kick_version", "kV2");
}

Brain::~Brain()
{

}

void Brain::init()
{
    
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    ballStateEstimator = std::make_unique<ball_state_estimator::BallStateEstimator>();
    locator = std::make_shared<Locator>();
    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    communication = std::make_shared<BrainCommunication>(this);

    
    locator->init(config->fieldDimensions, config->pfMinMarkerCnt, config->pfMaxResidual);

   
    tree->init();

   
    client->init();

    log->prepare();
    

    
    // Finish all shared-state initialization before communication threads are
    // allowed to read and publish it.
    const auto now = get_clock()->now();
    data->lastSuccessfulLocalizeTime = now;
    data->timeLastDet = now;
    data->timeLastLineDet = now;
    data->timeLastGamecontrolMsg = now;
    data->ball.timePoint = now;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->tmStatus[i].isAlive = false;
        data->tmStatus[i].timeLastCom = now;
    }
    data->tmLastCmdChangeTime = now;

    communication->initCommunication();

   
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_vision/line_segments", SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));

    // 传感器只保留最新样本，并使用 best-effort，和 Booster 发布端的传感器 QoS 对齐。
    // 默认 reliable 订阅在发布端为 best-effort 时会完全匹配失败，表现为“里程计不更新”。
    const auto latestSensorQos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    const auto latestReliableQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", latestSensorQos, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", latestSensorQos, bind(&Brain::lowStateCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", latestSensorQos, bind(&Brain::headPoseCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state", latestReliableQos, bind(&Brain::recoveryStateCallback, this, _1));

    if (config->rerunLogEnableFile || config->rerunLogEnableTCP) {
        string imageTopic = get_parameter("vision.image_topic").as_string();
        imageSubscription = create_subscription<sensor_msgs::msg::Image>(imageTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::imageCallback, this, _1));
    }
    string depthTopic = get_parameter("vision.depth_image_topic").as_string();
    depthImageSubscription = create_subscription<sensor_msgs::msg::Image>(depthTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::depthImageCallback, this, _1));
    
    pubSoundPlay = create_publisher<std_msgs::msg::String>("/play_sound", 10);
    pubSpeak = create_publisher<std_msgs::msg::String>("/speak", 10);
    pubKickBall = create_publisher<brain::msg::Kick>("/kick_ball", 10);
    setupLowFrequencyDiagnostics();
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);
    get_parameter("game.treat_person_as_robot", config->treatPersonAsRobot);
    get_parameter("game.number_of_players", config->numOfPlayers);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.vx_limit", config->vxLimit);
    get_parameter("robot.vy_limit", config->vyLimit);
    get_parameter("robot.vtheta_limit", config->vthetaLimit);

    get_parameter("strategy.ball_confidence_threshold", config->ballConfidenceThreshold);
    get_parameter("strategy.ball_confidence_decay_rate", config->ballConfidenceDecayRate);
    get_parameter("strategy.enable_stable_kick", config->enableStableKick);
    get_parameter("strategy.tm_ball_dist_threshold", config->tmBallDistThreshold);
    get_parameter("strategy.limit_near_ball_speed", config->limitNearBallSpeed);
    get_parameter("strategy.near_ball_speed_limit", config->nearBallSpeedLimit);
    get_parameter("strategy.near_ball_range", config->nearBallRange);

    get_parameter("obstacle_avoidance.collision_threshold", config->collisionThreshold);
    get_parameter("obstacle_avoidance.safe_distance", config->safeDistance);
    get_parameter("obstacle_avoidance.avoid_secs", config->avoidSecs);

    get_parameter("locator.min_marker_count", config->pfMinMarkerCnt);
    get_parameter("locator.max_residual", config->pfMaxResidual);

    get_parameter("enable_com", config->enableCom);

    // get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.enable_tcp", config->rerunLogEnableTCP);
    get_parameter("rerunLog.server_ip", config->rerunLogServerIP);
    get_parameter("rerunLog.enable_file", config->rerunLogEnableFile);
    get_parameter("rerunLog.log_dir", config->rerunLogLogDir);
    get_parameter("rerunLog.max_log_file_mins", config->rerunLogMaxFileMins);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("sound.enable", config->soundEnable);
    get_parameter("sound.sound_pack", config->soundPack);

    get_parameter("recovery.get_up_version", config->recoveryGetUpVersion);
    get_parameter("RLVisionKick.visual_kick_version", config->RLVisionKickVisualKickVersion);

    get_parameter("tree_file_path", config->treeFilePath);

    get_parameter("vision.cam_pixel_width", config->camPixX);
    get_parameter("vision.cam_pixel_height", config->camPixY);
    double camDegX, camDegY;
    get_parameter("vision.cam_fov_x", camDegX);
    get_parameter("vision.cam_fov_y", camDegY);
    config->camAngleX = deg2rad(camDegX);
    config->camAngleY = deg2rad(camDegY);

    // 从视觉 config 中加载相关参数
    string visionConfigPath, visionConfigLocalPath;
    get_parameter("vision_config_path", visionConfigPath);
    get_parameter("vision_config_local_path", visionConfigLocalPath);
    if (!filesystem::exists(visionConfigPath)) {
        // 报错然后退出
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());
        exit(1);
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);
    if (filesystem::exists(visionConfigLocalPath)) {
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);
        MergeYAML(vConfig, vConfigLocal);
    }
    config->camfx = vConfig["camera"]["intrin"]["fx"].as<double>();
    config->camfy = vConfig["camera"]["intrin"]["fy"].as<double>();
    config->camcx = vConfig["camera"]["intrin"]["cx"].as<double>();
    config->camcy = vConfig["camera"]["intrin"]["cy"].as<double>();

    auto extrin = vConfig["camera"]["extrin"];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();
        }
    }
    prtDebug(format("camfx: %f, camfy: %f, camcx: %f, camcy: %f", config->camfx, config->camfy, config->camcx, config->camcy));
    string str_cam2head = "camToHead: \n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            str_cam2head += format("%.3f ", config->camToHead(i, j));
        }
        str_cam2head += "\n";
    }
    prtDebug(str_cam2head);


    config->handle();


    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}


void Brain::tick()
{
    // main.cpp 中 tick 与 ROS executor 分线程运行；串行化共享 BrainData，避免
    // odometerCallback/calibrateOdom 在同一帧互相覆盖半更新的 Pose2D。
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    // 输出 debug & log 相关信息
    logDebugInfo();
    // logObstacleDistance(); // 计算量大, 仅需要时使用
    logLags();
    statusReport();
    logStatusToConsole();
    playSoundForFun();
    updateLogFile();
    
    updateMemory();
    handleSpecialStates();
    handleCooperation();

    tree->tick();
    // 行为树先写入守门员拦截目标，本帧立即发布给运控。
    pubKickMsg();
    if (lowFrequencyStatusEnable_) {
        const auto now = get_clock()->now();
        sendModeQueryIfNeeded(now);
        logLowFrequencyDiagnostics(now);
    }
}

void Brain::pubKickMsg() {
    if (!pubKickBall) return;
    const bool goalieVisualKick =
        tree->getEntry<string>("player_role") == "goal_keeper" &&
        (data->goalieVisualKickSubState == "block_kick" ||
         data->goalieVisualKickSubState == "intercept_kick");
    const bool goalieInterceptTarget =
        goalieVisualKick &&
        data->goalieVisualKickSubState == "intercept_kick" &&
        data->goalieVisualKickTargetValid &&
        std::isfinite(data->goalieVisualKickTargetField.x) &&
        std::isfinite(data->goalieVisualKickTargetField.y);

    const double realBallAgeMs = data->ball.timePoint.nanoseconds() > 0
        ? msecsSince(data->ball.timePoint)
        : std::numeric_limits<double>::infinity();
    double realBallMaxAgeMs = 650.0;
    double predictedRejectRange = 0.45;
    bool allowPredicted = true;
    bool allowMemory = true;
    get_parameter("strategy.auto_visual_kick.kick_msg_real_ball_max_age_ms", realBallMaxAgeMs);
    get_parameter("strategy.auto_visual_kick.no_detection_kick_msg_reject_range", predictedRejectRange);
    get_parameter("strategy.auto_visual_kick.allow_predicted_kick_msg", allowPredicted);
    get_parameter("strategy.auto_visual_kick.allow_memory_kick_msg", allowMemory);
    realBallMaxAgeMs = std::clamp(realBallMaxAgeMs, 50.0, 1000.0);
    predictedRejectRange = std::clamp(predictedRejectRange, 0.3, 3.0);

    const bool realBallFresh = data->ballDetected &&
        (!data->tmImInVisualKick ||
         (realBallAgeMs >= 0.0 && realBallAgeMs <= realBallMaxAgeMs));
    const double predictedRange = std::hypot(
        data->ball.reliablePredictPosToRobot.x,
        data->ball.reliablePredictPosToRobot.y);
    const bool predictedBallUsable = allowPredicted && data->reliableBallPredict &&
        std::isfinite(predictedRange) &&
        (!data->tmImInVisualKick || predictedRange > predictedRejectRange);
    const bool memoryBallUsable = allowMemory &&
        tree->getEntry<bool>("ball_location_known") &&
        realBallAgeMs >= 0.0 && realBallAgeMs <= 1000.0 &&
        std::isfinite(data->ball.posToRobot.x) &&
        std::isfinite(data->ball.posToRobot.y) &&
        (!data->tmImInVisualKick || data->ball.range > predictedRejectRange);

    Point ballField;
    Pose2D ballRobot;
    if (goalieInterceptTarget) {
        ballField = data->goalieVisualKickTargetField;
        ballRobot = data->field2robot({ballField.x, ballField.y, 0.0});
    } else if (realBallFresh) {
        ballField = data->ball.posToField;
        ballRobot = {data->ball.posToRobot.x, data->ball.posToRobot.y, 0.0};
    } else if (predictedBallUsable) {
        ballField = data->ball.reliablePredictPosToField;
        ballRobot = {data->ball.reliablePredictPosToRobot.x,
                     data->ball.reliablePredictPosToRobot.y, 0.0};
    } else if (memoryBallUsable) {
        ballField = data->ball.posToField;
        ballRobot = {data->ball.posToRobot.x, data->ball.posToRobot.y, 0.0};
    } else {
        return;
    }

    brain::msg::Kick kickMsg;
    kickMsg.header.stamp = get_clock()->now();
    kickMsg.x = ballRobot.x;
    kickMsg.y = ballRobot.y;

    double goal_x = config->fieldDimensions.length / 2;
    double goal_y = 0.0;
    const double clearanceDir = kick_geometry::directionToOpponentGoal(
        ballField.x, ballField.y, goal_x);
    kickMsg.dir = toPInPI((goalieVisualKick ? clearanceDir : data->kickDir)
                         - data->robotPoseToField.theta);
    Pose2D goalPose;
    goalPose.x = goal_x;
    goalPose.y = goal_y;
    double ball_x = ballField.x;
    double ball_y = ballField.y;
    double dist = std::sqrt((goal_x - ball_x) * (goal_x - ball_x) + (goal_y - ball_y) * (goal_y - ball_y));
    dist = std::abs(dist);
    double power = 0.0;

    if (goalieVisualKick) {
        power = get_parameter("RLVisionKick.goalie_clearance_power").as_double();
    } else if (dist > 6.0) {
        power = 3.0;
    } else {
        power = 6.0;
    }
    kickMsg.power = power;

    auto goalPose_r = data->field2robot(goalPose);
    kickMsg.goal_x = goalPose_r.x;
    kickMsg.goal_y = goalPose_r.y;

    kickMsg.robot_theta_to_field = data->robotPoseToField.theta;

    pubKickBall->publish(kickMsg);
}

void Brain::handleSpecialStates() {

    const double KICKOFF_DURATION = 10.0; 
    string gameState = tree->getEntry<string>("gc_game_state");
    bool isKickoffSide = tree->getEntry<bool>("gc_is_kickoff_side");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    bool isFreekickKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    auto now = get_clock()->now();

    const bool normalKickoffState = gameSubStateType == "NONE";
    if (gameState == "SET" && normalKickoffState
        && !tree->getEntry<bool>("gc_play_stopped")) {
        if (isKickoffSide) {
            data->isKickingOff = true;
            data->kickoffStartTime = now;
            data->isOpponentKickingOff = false;
        } else {
            data->isOpponentKickingOff = true;
            data->opponentKickoffStartTime = now;
            data->isKickingOff = false;
        }
    } else if (msecsSince(data->kickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isKickingOff = false;
    }
    if (msecsSince(data->opponentKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isOpponentKickingOff = false;
    }

    if (gameState == "PLAY" && gameSubStateType == "FREE_KICK"
        && isFreekickKickoffSide && !tree->getEntry<bool>("gc_play_stopped")) {
        data->isFreekickKickingOff = true;
        data->freekickKickoffStartTime = now;
    } else if (msecsSince(data->freekickKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isFreekickKickingOff = false;
        data->isDirectShoot = false;
    }

    static int lastScore = 0;
    if (data->score > lastScore) {
        tree->setEntry<bool>("we_just_scored", true);
        lastScore = data->score;
    }
    if (gameState == "SET") {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

int Brain::selectGoaliePlayerId(bool *communicationHealthy,
                                bool *stableEnough,
                                bool *usedFallback)
{
    if (communicationHealthy) *communicationHealthy = false;
    if (stableEnough) *stableEnough = false;
    if (usedFallback) *usedFallback = true;

    double freshMs = get_parameter("strategy.cooperation.goalie_comm_fresh_msecs").as_double();
    double stableMs = get_parameter("strategy.cooperation.goalie_comm_stable_msecs").as_double();
    double tieMargin = get_parameter("strategy.cooperation.goalie_distance_tie_margin").as_double();
    const bool useGcFallback = get_parameter(
        "strategy.cooperation.goalie_fallback_use_gc_goalie").as_bool();
    freshMs = std::clamp(freshMs, 400.0, 10000.0);
    stableMs = std::clamp(stableMs, 0.0, 10000.0);
    tieMargin = std::clamp(tieMargin, 0.0, 0.8);

    const int playerCount = std::clamp(config->numOfPlayers, 1, HL_MAX_NUM_PLAYERS);
    const int selfIdx = config->playerId - 1;
    auto poseUsable = [](const Pose2D &pose) {
        return std::isfinite(pose.x) && std::isfinite(pose.y);
    };
    auto distanceToGoal = [&](const Pose2D &pose) {
        return std::hypot(pose.x + config->fieldDimensions.length / 2.0, pose.y);
    };

    int eligibleCount = 0;
    int freshPoseCount = 0;
    int selected = 0;
    double selectedDistance = std::numeric_limits<double>::infinity();
    int selectedNotKicking = 0;
    double selectedNotKickingDistance = std::numeric_limits<double>::infinity();
    auto consider = [&](int id, const Pose2D &pose, bool inVisualKick) {
        const double distance = distanceToGoal(pose);
        if (distance < selectedDistance - tieMargin ||
            (std::abs(distance - selectedDistance) <= tieMargin &&
             (selected == 0 || id < selected))) {
            selected = id;
            selectedDistance = distance;
        }
        if (!inVisualKick &&
            (distance < selectedNotKickingDistance - tieMargin ||
             (std::abs(distance - selectedNotKickingDistance) <= tieMargin &&
              (selectedNotKicking == 0 || id < selectedNotKicking)))) {
            selectedNotKicking = id;
            selectedNotKickingDistance = distance;
        }
    };

    for (int i = 0; i < playerCount; ++i) {
        if (data->penalty[i] != PENALTY_NONE) continue;
        ++eligibleCount;
        if (i == selfIdx) {
            if (poseUsable(data->robotPoseToField)) {
                consider(i + 1, data->robotPoseToField, data->tmImInVisualKick);
                ++freshPoseCount;
            }
            continue;
        }
        const auto &tm = data->tmStatus[i];
        const double ageMs = tm.timeLastCom.nanoseconds() > 0
            ? msecsSince(tm.timeLastCom)
            : std::numeric_limits<double>::infinity();
        if (!tm.isAlive || !std::isfinite(ageMs) || ageMs > freshMs ||
            !poseUsable(tm.robotPoseToField)) continue;
        consider(i + 1, tm.robotPoseToField, tm.isInVisualKick);
        ++freshPoseCount;
    }
    if (selectedNotKicking > 0) selected = selectedNotKicking;

    const bool healthy = eligibleCount > 0 && freshPoseCount == eligibleCount;
    if (communicationHealthy) *communicationHealthy = healthy;
    static bool healthySinceValid = false;
    static rclcpp::Time healthySince;
    if (healthy) {
        if (!healthySinceValid) {
            healthySince = get_clock()->now();
            healthySinceValid = true;
        }
        const bool stable = msecsSince(healthySince) >= stableMs;
        if (stableEnough) *stableEnough = stable;
        if (stable && selected > 0) {
            if (usedFallback) *usedFallback = false;
            return selected;
        }
    } else {
        healthySinceValid = false;
    }

    if (useGcFallback && data->goalkeeperPlayerId >= 1 &&
        data->goalkeeperPlayerId <= playerCount &&
        data->penalty[data->goalkeeperPlayerId - 1] == PENALTY_NONE) {
        return data->goalkeeperPlayerId;
    }
    for (int i = 0; i < playerCount; ++i) {
        if (data->penalty[i] == PENALTY_NONE) return i + 1;
    }
    return 0;
}

void Brain::handleCooperation() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/handleCooperation", rerun::TextLog(msg));
    };
    log_("handle cooperation");

    const int CMD_COOLDOWN = 2000; 
    const int COM_TIMEOUT = 5000; 

    int selfId = config->playerId;
    int selfIdx = selfId - 1;
    int numOfPlayers = std::clamp(config->numOfPlayers, 1, HL_MAX_NUM_PLAYERS);

    vector<int> aliveTmIdxs = {}; 


    data->tmImAlive = 
        (data->penalty[selfIdx] == PENALTY_NONE) 
        && tree->getEntry<bool>("odom_calibrated"); 
    updateCostToKick(); 
    log_(format("ImAlive: %d, myCost: %.1f", data->tmImAlive, data->tmMyCost));

    
    int gcAliveCount = 0; 
    for (int i = 0; i < numOfPlayers; i++)
    {
        if (data->penalty[i] == PENALTY_NONE) {
            gcAliveCount++;

            int tmId = i + 1;
            if (tmId == config->playerId) continue;

            auto tmStatus = data->tmStatus[i];
            log->setTimeNow();
            auto color = 0x00FFFFFF;
            if (!tmStatus.isAlive) color = 0x006666FF;
            else if (!tmStatus.isLead) color = 0x00CCCCFF;
            string label = format("ID: %d, Cost: %.1f", tmId, tmStatus.cost);
            log->logRobot(format("field/teammate-%d", tmId).c_str(), tmStatus.robotPoseToField, color, label);
            log->logBall(
            format("tm_ball-%d", tmId).c_str(),
            tmStatus.ballPosToField, 
            tmStatus.ballDetected ? 0x00FFFFFF : (tmStatus.isAlive ? 0x006666FF : 0x003333FF),
            tmStatus.ballConfidence,
            tmStatus.ballLocationKnown
            );
        }
    }
    log_(format("gcAliveCnt: %d", gcAliveCount));

    for (int i = 0; i < numOfPlayers; i++) {
        if (i == selfIdx) continue; 

        if (
            data->penalty[i] != PENALTY_NONE 
            || msecsSince(data->tmStatus[i].timeLastCom) > COM_TIMEOUT 
        ) {
            data->tmStatus[i].isAlive = false;
            data->tmStatus[i].isLead = false;
            data->tmStatus[i].isInVisualKick = false;
        }
        
        if (data->tmStatus[i].isAlive) {
            aliveTmIdxs.push_back(i);
            log->setTimeNow();
            log->log(format("debug/tm_cost_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].cost));
            log->log(format("debug/tm_lead_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].isLead));
        }
    }
    log_(format("alive TM Count: %d", aliveTmIdxs.size()));

    // log 当前 alive 队友的信息
    log_(format("Self: cost: %.1f, isLead: %d", data->tmMyCost, data->tmImLead));


    static rclcpp::Time lastTmBallPosTime = get_clock()->now();
    const double TM_BALL_TIMEOUT = 1000.; 
    const double RANGE_THRESHOLD = config->tmBallDistThreshold; 
    int trustedTMIdx = -1;
    double minRange = 1e6;
    log_(format("Find ball info among %d alive TMs", aliveTmIdxs.size()));
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        auto status = data->tmStatus[aliveTmIdxs[i]];
        log_(format("TM %d, ballDetected: %d, ballRange: %.1f", i + 1, status.ballDetected, status.ballRange));
        if (status.ballDetected && status.ballRange < minRange) {
            log_(format("tm ball range(%.1f) < minRange(%.1f)", status.ballRange, minRange));
            double dist = norm(status.ballPosToField.x - data->robotPoseToField.x, status.ballPosToField.y - data->robotPoseToField.y);
            if (dist > RANGE_THRESHOLD) {
                log_(format("tm ball dist to me(%.1f) > threshold(%.1f), TM %d can be trusted", dist, RANGE_THRESHOLD, i+ 1));
                minRange = status.ballRange;
                trustedTMIdx = aliveTmIdxs[i];
            }  else {
                log_(format("tm ball dist to me(%.1f) < threshold(%.1f), TM %d can NOT be trusted", dist, RANGE_THRESHOLD, i+ 1));
            }
        }
    }
    if (trustedTMIdx >= 0) {   
        log_(format("Reliable tm ball found. PlayerID = %d", trustedTMIdx + 1));
        data->tmBall.posToField = data->tmStatus[trustedTMIdx].ballPosToField;
        updateRelativePos(data->tmBall);

        tree->setEntry<bool>("tm_ball_pos_reliable", true);
        lastTmBallPosTime = get_clock()->now();
        if (!tree->getEntry<bool>("ball_location_known")) {
            log_("update ball.posToField");
            data->ball.posToField = data->tmBall.posToField;
            updateRelativePos(data->ball);
            data->robotBallAngleToField = atan2(
                data->ball.posToField.y - data->robotPoseToField.y,
                data->ball.posToField.x - data->robotPoseToField.x);
        }
    } else {
        log_("TM reported NO BALL or can not be trusted");
        if (msecsSince(lastTmBallPosTime) > TM_BALL_TIMEOUT) {
            log_("TM ball timeout reached");
            tree->setEntry<bool>("tm_ball_pos_reliable", false);
        }
    }

    bool switchRole;
    get_parameter("strategy.cooperation.enable_role_switch", switchRole);
    static int directedGoalieId = 0;
    static rclcpp::Time directedGoalieUntil;
    bool teamVisualKickActive = data->tmImInVisualKick;
    for (int tmIdx : aliveTmIdxs) {
        teamVisualKickActive = teamVisualKickActive || data->tmStatus[tmIdx].isInVisualKick;
    }
    if (directedGoalieId > 0 && teamVisualKickActive) {
        directedGoalieUntil = get_clock()->now() +
            rclcpp::Duration::from_nanoseconds(2000000000LL);
    }
    if (directedGoalieId > 0 && msecsSince(directedGoalieUntil) > 0.0) {
        if (data->tmMyCmd == 10 + directedGoalieId) {
            data->tmMyCmd = 0;
            ++data->tmCmdId;
            data->tmMyCmdId = data->tmCmdId;
        }
        directedGoalieId = 0;
    }
    bool goalieCommHealthy = false;
    bool goalieStable = false;
    bool goalieFallback = true;
    int selectedGoalieId = selectGoaliePlayerId(
        &goalieCommHealthy, &goalieStable, &goalieFallback);
    if (directedGoalieId > 0 && directedGoalieId <= numOfPlayers &&
        data->penalty[directedGoalieId - 1] == PENALTY_NONE) {
        selectedGoalieId = directedGoalieId;
    }
    data->activeGoaliePlayerId = selectedGoalieId;

    if (tree->getEntry<string>("gc_game_state") == "INITIAL" || !switchRole) {
        tree->setEntry<string>("player_role", config->playerRole);
    } else if (data->tmImAlive && selectedGoalieId > 0) {
        tree->setEntry<string>("player_role",
            selectedGoalieId == selfId ? "goal_keeper" : "striker");
    }
    log_(format("goalie election: selected=%d gc=%d healthy=%d stable=%d fallback=%d role=%s",
        selectedGoalieId, data->goalkeeperPlayerId,
        goalieCommHealthy ? 1 : 0, goalieStable ? 1 : 0, goalieFallback ? 1 : 0,
        tree->getEntry<string>("player_role").c_str()));


    constexpr double COST_TIE_EPS = 0.05;
    int myCostRank = 0;
    int myStrikerCostRank = 0;
    int myStrikerIDRank = 0;
    const bool selfIsStriker = tree->getEntry<string>("player_role") == "striker";
    for (int tmIdx : aliveTmIdxs) {
        const auto &tm = data->tmStatus[tmIdx];
        const bool teammateRanksAhead =
            tm.cost < data->tmMyCost - COST_TIE_EPS ||
            (std::fabs(tm.cost - data->tmMyCost) <= COST_TIE_EPS && tmIdx + 1 < selfId);
        if (teammateRanksAhead) ++myCostRank;
        if (selfIsStriker && tm.role == "striker" && teammateRanksAhead) {
            ++myStrikerCostRank;
        }
        if (tmIdx < selfIdx && tm.role == "striker") ++myStrikerIDRank;
    }
    data->tmMyCostRank = myCostRank;
    data->tmMyStrikerCostRank = selfIsStriker ? myStrikerCostRank : 0;
    data->myStrikerIDRank = myStrikerIDRank;

    const bool kickoffTacticalPhase =
        (data->isKickingOff || data->isOpponentKickingOff) &&
        tree->getEntry<string>("gc_game_state") == "PLAY";
    if (!data->isKickingOff && !data->isOpponentKickingOff) {
        data->kickoffStrikerGroupRank = -1;
        data->kickoffStrikerGroupLatched = false;
        data->kickoffBallHalf = 0;
    } else if (kickoffTacticalPhase && selfIsStriker
        && !data->kickoffStrikerGroupLatched) {
        // 开球阶段只在第一次进入 PLAY 时确定分组，避免 cost 变化导致机器人互换半区任务。
        data->kickoffStrikerGroupRank = myStrikerCostRank;
        data->kickoffStrikerGroupLatched = true;
    }

    struct BallOwnerCandidate {
        int id = 0;
        double cost = 1e9;
        bool visualKick = false;
    };
    vector<BallOwnerCandidate> ownerCandidates;
    const double goalieClaimBallX = -config->fieldDimensions.circleRadius - 2.0;
    const string selfRole = tree->getEntry<string>("player_role");
    const bool selfGoalieClaimsBall =
        selfRole == "goal_keeper" &&
        (data->tmImInVisualKick ||
         (data->ballDetected && !data->lose_ball && std::isfinite(data->ball.range) &&
          data->ball.range < 1.5 && data->ball.posToField.x < goalieClaimBallX));
    if (selfRole == "striker" || selfGoalieClaimsBall) {
        ownerCandidates.push_back({selfId, data->tmMyCost, data->tmImInVisualKick});
    }
    for (int tmIdx : aliveTmIdxs) {
        const auto &tm = data->tmStatus[tmIdx];
        const bool teammateGoalieClaimsBall =
            tm.role == "goal_keeper" &&
            (tm.isInVisualKick ||
             (tm.ballDetected && std::isfinite(tm.ballRange) && tm.ballRange < 1.5 &&
              tm.ballPosToField.x < goalieClaimBallX));
        if (tm.role == "striker" || teammateGoalieClaimsBall) {
            ownerCandidates.push_back({tmIdx + 1,
                std::isfinite(tm.cost) ? tm.cost : 1e9,
                tm.isInVisualKick});
        }
    }
    if (ownerCandidates.empty() && data->tmImAlive) {
        ownerCandidates.push_back({selfId, data->tmMyCost, data->tmImInVisualKick});
    }

    auto betterOwner = [](const BallOwnerCandidate &lhs, const BallOwnerCandidate &rhs) {
        return lhs.cost < rhs.cost - COST_TIE_EPS ||
            (std::fabs(lhs.cost - rhs.cost) <= COST_TIE_EPS && lhs.id < rhs.id);
    };
    BallOwnerCandidate bestOwner;
    BallOwnerCandidate activeVisualKickOwner;
    for (const auto &candidate : ownerCandidates) {
        if (bestOwner.id == 0 || betterOwner(candidate, bestOwner)) bestOwner = candidate;
        if (candidate.visualKick &&
            (activeVisualKickOwner.id == 0 || betterOwner(candidate, activeVisualKickOwner))) {
            activeVisualKickOwner = candidate;
        }
    }
    const int ownerId = activeVisualKickOwner.id > 0
        ? activeVisualKickOwner.id
        : bestOwner.id;
    data->tmImLead = data->tmImAlive && ownerId == selfId;
    tree->setEntry<bool>("is_lead", data->tmImLead);
    log_(format("ball owner: %d, myCost: %.1f, myCostRank: %d, myStrikerCostRank: %d, kickoffGroupRank: %d, myStrikerIDRank: %d",
        ownerId, data->tmMyCost, myCostRank, myStrikerCostRank,
        data->kickoffStrikerGroupRank, myStrikerIDRank));


    const bool goalieAttackHandoffEnabled = get_parameter(
        "strategy.cooperation.goalie_attack_handoff").as_bool();
    if (
        goalieAttackHandoffEnabled &&
        data->tmImAlive 
        && tree->getEntry<string>("player_role") == "goal_keeper"
        && data->ballDetected
        && !data->lose_ball
        && data->ball.range < 1.2
        && (data->tmImLead || data->tmImInVisualKick)
        && msecsSince(data->tmLastCmdChangeTime) > CMD_COOLDOWN 
    ) {
        auto distToGoal = [=](Pose2D pose) {
            return norm(pose.x + config->fieldDimensions.length / 2.0, pose.y);
        };
        double minDist = 1e6;
        int minIndex = -1; 
        double myDist = distToGoal(data->robotPoseToField);
        for (int i = 0; i < aliveTmIdxs.size(); i++) {
            int tmIdx = aliveTmIdxs[i];
            auto tmPose = data->tmStatus[tmIdx].robotPoseToField;
            if (!std::isfinite(tmPose.x) || !std::isfinite(tmPose.y)) continue;
            double dist = distToGoal(tmPose);
            if (dist < minDist) {
                minIndex = tmIdx;
                minDist = dist;
            }
        }
        if (minIndex >= 0) {
            data->tmLastCmdChangeTime = get_clock()->now();
            data->tmMyCmd = 10 + minIndex + 1; 
            data->tmCmdId += 1;
            data->tmMyCmdId = data->tmCmdId;
            directedGoalieId = minIndex + 1;
            directedGoalieUntil = get_clock()->now() + rclcpp::Duration::from_nanoseconds(6000000000LL);
            tree->setEntry<string>("player_role", "striker");
            log_(format("goalie: clearance handoff, ask player %d to guard goal", minIndex + 1));
        } else {
            log_(format("goalie: no teammate available for handoff, my dist: %.2f", myDist));
        }
    }


    auto cmd = data->tmReceivedCmd;
    if (cmd != 0) {
        log_(format("received cmd %d from teammate", cmd));
        if (cmd == 100) { // 队友要控球
            data->tmImLead = false;
            tree->setEntry<bool>("is_lead", false);
            log_("teammate wants to take lead, i'll assist");
        } else if (cmd > 10 && cmd < 20) { 
            log_("goalie wants to attack");
            int newGoalieId = cmd - 10;
            directedGoalieId = newGoalieId;
            directedGoalieUntil = get_clock()->now() + rclcpp::Duration::from_nanoseconds(6000000000LL);
            if (newGoalieId == selfId) { 
                log_("i become goalie");
                tree->setEntry<string>("player_role", "goal_keeper");
                speak("i become goalie", true);
            } else { 
                log_(format("teammate %d becomes goalie", newGoalieId));
            }
        } else {
            log_(format("unknown cmd %d from teammate", cmd));
        }

        data->tmReceivedCmd = 0; 
    }

    tree->setEntry<bool>("is_lead", data->tmImLead);

    return;
}

void Brain::updateMemory()
{
    updateBallMemory();
    updateRobotMemory();
    updateObstacleMemory();
    updateKickoffMemory();
}

void Brain::updateObstacleMemory() {
   
    auto obstacles = data->getObstacles();
    vector<GameObject> obs_new = {};

    const double OBS_EXPIRE_TIME = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
    for (int i = 0; i < obstacles.size(); i++) {
        auto obs = obstacles[i];
        if (obs.label == "Ball") continue; 
        if (msecsSince(obs.timePoint) > OBS_EXPIRE_TIME)  continue; 


        updateRelativePos(obs);
        obs_new.push_back(obs);
    }


    const bool ballLocationKnown =
        tree->getEntry<bool>("ball_location_known") ||
        tree->getEntry<bool>("tm_ball_pos_reliable");
    if (ballLocationKnown &&
        ((get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() && isFreekickStartPlacing())
         || tree->getEntry<string>("gc_game_state") == "READY")) {
        obs_new.push_back(data->ball);
    }

    data->setObstacles(obs_new);
    logObstacles();
}

void Brain::updateBallMemory()
{

    double secs = msecsSince(data->ball.timePoint) / 1000;
    
    double ballMemTimeout;
    get_parameter("strategy.ball_memory_timeout", ballMemTimeout);

    if (secs > ballMemTimeout)
    {
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_out", false);
    }

    const bool ballLocationKnown =
        tree->getEntry<bool>("ball_location_known") ||
        tree->getEntry<bool>("tm_ball_pos_reliable");
    if (ballLocationKnown) {
        updateRelativePos(data->ball);
        data->robotBallAngleToField = atan2(
            data->ball.posToField.y - data->robotPoseToField.y,
            data->ball.posToField.x - data->robotPoseToField.x);
    } else {
        data->robotBallAngleToField = 0.0;
    }
    if (tree->getEntry<bool>("tm_ball_pos_reliable")) {
        updateRelativePos(data->tmBall);
    }

    data->reliableBallPredict = false;
    if (!data->ballDetected && !data->predictedBallPos.empty() &&
        data->ballPosPredictTime.nanoseconds() > 0) {
        const double stepMs = std::max(1.0, get_parameter("ball_predictor.step_interval").as_double());
        const double ageMs = msecsSince(data->ballPosPredictTime);
        const double horizonMs = stepMs * static_cast<double>(data->predictedBallPos.size() - 1);
        if (std::isfinite(ageMs) && ageMs >= 0.0 && ageMs <= std::min(1000.0, horizonMs)) {
            const int idx = std::clamp(
                static_cast<int>(ageMs / stepMs),
                0,
                static_cast<int>(data->predictedBallPos.size()) - 1);
            const auto &pred = data->predictedBallPos[idx];
            if (std::isfinite(pred[0]) && std::isfinite(pred[1])) {
                data->ball.reliablePredictPosToField = {pred[0], pred[1], 0.0};
                const auto predRobot = data->field2robot({pred[0], pred[1], 0.0});
                data->ball.reliablePredictPosToRobot = {predRobot.x, predRobot.y, 0.0};
                data->reliableBallPredict = true;
            }
        }
    }
    tree->setEntry<double>("ball_range", ballLocationKnown ? data->ball.range : 0.0);



    log->setTimeNow();
    log->logBall(
        "field/ball", 
        data->ball.posToField, 
        data->ballDetected ? 0x00FF00FF : 0x006600FF,
        data->ballDetected,
        tree->getEntry<bool>("ball_location_known")
        );
    log->logBall(
        "field/tmBall", 
        data->tmBall.posToField, 
        0xFFFF00FF,
        tree->getEntry<bool>("tm_ball_pos_reliable"),
        tree->getEntry<bool>("tm_ball_pos_reliable")
        );
}

void Brain::updateBallPrediction()
{
    const auto now = data->ball.timePoint;
    const Point current = data->ball.posToField;

    const double stepSec = std::max(0.001,
        get_parameter("ball_predictor.step_interval").as_double() / 1000.0);
    const int stepCnt = std::max(1,
        static_cast<int>(get_parameter("ball_predictor.step_cnt").as_int()));
    const double acceleration = std::min(-1e-3,
        get_parameter("ball_predictor.acceleration").as_double());
    const double speedFrictionFactor = std::max(0.0,
        get_parameter("ball_predictor.speed_friction_factor").as_double());

    ballStateEstimator->setFriction(acceleration);
    ballStateEstimator->setMinSpeedThreshold(0.1);
    ballStateEstimator->update(current.x, current.y, now, 0.05);

    const auto estimatePosition = ballStateEstimator->getEstimatedPosition();
    auto estimateVelocity = ballStateEstimator->getEstimatedVelocity();
    ball_state_estimator::BallPhysics::clampVelocity(estimateVelocity.x, estimateVelocity.y);
    data->ballEstVelX = estimateVelocity.x;
    data->ballEstVelY = estimateVelocity.y;
    data->ballEstSpeed = estimateVelocity.norm();
    data->ballEstIsRolling = ballStateEstimator->isRolling() && data->ballEstSpeed >= 0.1;

    const auto endPosition = ball_state_estimator::BallPhysics::getEndPositionWithSpeedFriction(
        estimatePosition.x, estimatePosition.y,
        estimateVelocity.x, estimateVelocity.y,
        acceleration, speedFrictionFactor);
    data->ballEstEndPos = {
        std::clamp(endPosition.x, -config->fieldDimensions.length / 2.0,
                   config->fieldDimensions.length / 2.0),
        std::clamp(endPosition.y, -config->fieldDimensions.width / 2.0,
                   config->fieldDimensions.width / 2.0)};

    vector<array<double, 2>> predictions;
    predictions.reserve(stepCnt);
    double px = estimatePosition.x;
    double py = estimatePosition.y;
    double vx = estimateVelocity.x;
    double vy = estimateVelocity.y;
    const auto &fd = config->fieldDimensions;
    for (int i = 0; i < stepCnt; ++i) {
        predictions.push_back({
            std::clamp(px, -fd.length / 2.0, fd.length / 2.0),
            std::clamp(py, -fd.width / 2.0, fd.width / 2.0)});
        ball_state_estimator::BallPhysics::propagateWithSpeedFriction(
            px, py, vx, vy, stepSec, acceleration, speedFrictionFactor);
    }

    data->predictedBallPos = std::move(predictions);
    data->ballPosPredictTime = now;

    int closestIdx = 0;
    double closestDist = std::numeric_limits<double>::infinity();
    data->ballWillBreach = false;
    for (int i = 0; i < static_cast<int>(data->predictedBallPos.size()); ++i) {
        const auto &p = data->predictedBallPos[i];
        const auto pr = data->field2robot({p[0], p[1], 0.0});
        const double dist = std::hypot(pr.x, pr.y);
        if (dist < closestDist) {
            closestDist = dist;
            closestIdx = i;
        }
        if (i == 0 && pr.x < 0.0) {
            data->ballWillBreach = false;
            break;
        }
        if (i > 0 && pr.x < 0.0) {
            data->ballWillBreach = true;
            data->ballBreachPoint = {p[0], p[1]};
            data->ballBreachTime = now + rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(i * stepSec * 1e9));
            break;
        }
    }
    const auto &closest = data->predictedBallPos[closestIdx];
    data->ballInterceptPoint = {closest[0], closest[1]};
    data->ballInterceptTime = now + rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(closestIdx * stepSec * 1e9));
}

void Brain::updateRobotMemory() {
    auto robots = data->getRobots();
    vector<GameObject> newRobots = {};

    for (int i = 0; i < robots.size(); i++) {
        auto r = robots[i];


        if (msecsSince(r.timePoint) > 1000)  continue;


        updateRelativePos(r);
        newRobots.push_back(r);
    }

    data->setRobots(newRobots);

    logMemRobots();
}

void Brain::updateKickoffMemory() {
    
    static Point ballPos;
    static bool waitingForOpponentPrimaryKickoff = false;
    const double BALL_MOVE_THRESHOLD_FACTOR = 0.15; 
    const double BALL_MOVE_THRESHOLD_MIN = 0.3; 
    auto ballMoved = [=]() {
        if (!data->ballDetected) return false; 
        double range = data->ball.range;
        double threshold = max(range * BALL_MOVE_THRESHOLD_FACTOR, BALL_MOVE_THRESHOLD_MIN);
        double posChange = norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y);
        return posChange > threshold;
    };
    static rclcpp::Time kickOffTime;
    static bool opponentSetPlayLatched = false;
    const double TIMEOUT = 1000 * 10; 
    auto timeReached = [=]() {
        return msecsSince(kickOffTime) > TIMEOUT;
    };
    bool isWaitingForKickoffPreparation = (
        (tree->getEntry<string>("gc_game_state") == "SET"  || tree->getEntry<string>("gc_game_state") == "READY")
        && !tree->getEntry<bool>("gc_is_kickoff_side")
    );
    bool isWaitingForLegacyFreekickPreparation = (
        (tree->getEntry<string>("gc_game_sub_state") == "SET" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY")
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    );
    const bool opponentSetPlay =
        tree->getEntry<int>("gc_set_play") != 0
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    const bool isWaitingForV20SetPlayPreparation =
        opponentSetPlay && tree->getEntry<bool>("gc_play_stopped");
    const bool isNewOpponentSetPlay = opponentSetPlay && !opponentSetPlayLatched;
    const bool isV20OpponentSetPlayFinished =
        tree->getEntry<int>("gc_protocol_version") >= 20
        && !opponentSetPlay
        && opponentSetPlayLatched;

    if (isV20OpponentSetPlayFinished) {
        // 新版裁判机用 set_play 从非 0 变为 0 表示定位球结束，即 Ball Free。
        tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        waitingForOpponentPrimaryKickoff = false;
    } else if (isWaitingForLegacyFreekickPreparation
        || isWaitingForKickoffPreparation
        || isWaitingForV20SetPlayPreparation
        || isNewOpponentSetPlay) {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
        waitingForOpponentPrimaryKickoff = isWaitingForKickoffPreparation;
    } else if (tree->getEntry<bool>("wait_for_opponent_kickoff")) {
        // v20 的 secondaryTime 是距离 Ball Free 的剩余时间。
        // 对方普通开球时优先服从裁判机，避免视觉位置抖动导致提前进入追球逻辑。
        const bool isV20PrimaryKickoff =
            waitingForOpponentPrimaryKickoff
            && tree->getEntry<int>("gc_protocol_version") >= 20;
        const bool gameControllerSaysBallFree =
            tree->getEntry<string>("gc_game_state") == "PLAY"
            && tree->getEntry<int>("gc_secondary_time") <= 0;
        const bool legacyOrSetPlayReleased =
            !isV20PrimaryKickoff && (ballMoved() || timeReached());

        if ((isV20PrimaryKickoff && gameControllerSaysBallFree)
            || legacyOrSetPlayReleased) {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
            waitingForOpponentPrimaryKickoff = false;
        }
    }

    opponentSetPlayLatched = opponentSetPlay;
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY; 

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;


    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++)
    {
        auto post = goalposts[i];
        if (post.name == "OL")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.name == "OR")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

double Brain::calcKickDir(double goalPostMargin) {
    double dir_rb_f = data->robotBallAngleToField; 
    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1];
    
    if (isAngleGood(goalPostMargin)) return dir_rb_f;

    double delta_l = fabs(toPInPI(theta_l - dir_rb_f));
    double delta_r = fabs(toPInPI(theta_r - dir_rb_f));
    if (delta_l < delta_r) return theta_l;
    // else 
    return theta_r;
}

void Brain::updateCostToKick() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/updateCostToKick", rerun::TextLog(msg));
    };
    double cost = 0.;


    // Keep the time since this robot last saw the ball as an ownership cost.
    // This intentionally keeps growing while only a teammate supplies the ball.
    const double localBallAgeCost = std::max(
        0.0, msecsSince(data->ball.timePoint) / 1000.0);
    cost += localBallAgeCost;
    log_(format("local ball age cost: %.1f", localBallAgeCost));


    const bool localBallKnown = tree->getEntry<bool>("ball_location_known");
    const bool ballLocationKnown =
        localBallKnown || tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!localBallKnown) {
        cost += 5.0;
        log_(format("ball lost cost: %.1f", 5.0));
    }

    // Ball-dependent terms are meaningful only after a local or teammate ball
    // position has been validated. Unknown data must not influence strategy.
    if (ballLocationKnown) {
        cost += data->ball.range;
        log_(format("ball range cost: %.1f", data->ball.range));

        if (distToObstacle(data->ball.yawToRobot) < 1.5) {
            constexpr double OBSTACLE_COST = 0.5;
            cost += OBSTACLE_COST;
            log_(format("obstacle cost: %.1f", OBSTACLE_COST));
        }

        cost += fabs(data->ball.yawToRobot);
        log_(format("ball yaw cost: %.1f", fabs(data->ball.yawToRobot)));

        int selfIdx = config->playerId - 1;
        for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
            if (i == selfIdx) continue;

            auto status = data->tmStatus[i];
            if (!status.isAlive || !status.ballLocationKnown) continue;

            double theta_tm2ball = atan2(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
            double range_tm2ball = norm(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
            double theta_me2ball = data->robotBallAngleToField;
            double range_me2ball = data->ball.range;
            double deltaTheta = fabs(toPInPI(theta_tm2ball - theta_me2ball));

            const double BUMP_DIST = 1.0;
            if (range_tm2ball < range_me2ball && sin(deltaTheta) * range_tm2ball < BUMP_DIST) {
                cost += 2.0;
                log_(format("bump cost: %.1f", 2.0));
            }
        }

        const double alignmentError =
            fabs(toPInPI(data->kickDir - data->robotBallAngleToField));
        const double alignmentCost = alignmentError * 0.4 / 0.3;
        cost += alignmentCost;
        log_(format("alignment cost: %.1f (error: %.2f rad)",
                    alignmentCost, alignmentError));
    }
    

    if (data->recoveryState == RobotRecoveryState::HAS_FALLEN) {
        cost += 15.0;
        log_(format("fall cost: %.1f", 15.0));  
    }

    
    if (!tree->getEntry<bool>("odom_calibrated")) {
        cost += 100;
        log_(format("localization cost: %.1f", 100.0));  

    }
    
    double lastCost = data->tmMyCost;
    data->tmMyCost = lastCost * 0.8 + cost * 0.2;
    log_(format("lastCost: %.1f, newCost: %.1f, smoothCost: %.1f", lastCost, cost, data->tmMyCost));

    return;
}

bool Brain::isAngleGood(double goalPostMargin, string type) {
    double angle = 0;
    if (type == "kick") angle = data->robotBallAngleToField; // type=="kick" 机器人到球, field 坐标系中的方向
    if (type == "shoot") angle = data->robotPoseToField.theta; // type=="shoot" 机器人朝向
    

    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    
    if (kick_geometry::minorArcWidth(theta_l, theta_r) < M_PI / 3 * 2) {
        goalPostAngles = getGoalPostAngles(0.5);
        theta_l = goalPostAngles[0];
        theta_r = goalPostAngles[1];
    }

    return kick_geometry::isAngleWithinMinorArc(angle, theta_l, theta_r);
}

bool Brain::isPrimaryStriker() {
    string myRole = tree->getEntry<string>("player_role");
    if (myRole != "striker") return false; 

    if (!config->enableCom) return true; 

    // find first alive striker that is not me.
    auto firstAliveStrikerIdx = -1;
    auto myIdx = config->playerId - 1;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        auto status = data->tmStatus[i];
        if (data->penalty[i] == PENALTY_NONE && status.isAlive && status.role == "striker") {
            firstAliveStrikerIdx = i;
            break;
        }
    }

    if ( firstAliveStrikerIdx >= 0 && firstAliveStrikerIdx <  myIdx) return false; // 有 id 更小的活的前锋, 让他来当主力

    // else 没有 id 更小的活的前锋, 我是主力
    return true;
}

bool Brain::isBallOut(double locCompareDist, double lineCompareDist)
{
    auto ball = data->ball;
    auto fd = config->fieldDimensions;

    if (fabs(ball.posToField.x) > fd.length / 2 + locCompareDist)
        return true;
    if (fabs(ball.posToField.y) > fd.width / 2 + locCompareDist)
        return true;
    
    auto fieldLines = data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (
            (line.type == LineType::TouchLine || line.type == LineType::GoalLine)
            && line.confidence > 1.0
         ) {
            Point2D p = {ball.posToField.x, ball.posToField.y};
            // prtWarn(format("Ball: %.2f, %.2f PerpDist: %.2f", ball.posToField.x, ball.posToField.y, pointPerpDistToLine(p, line.posToField)));
            if (pointPerpDistToLine(p, line.posToField) > lineCompareDist) return true;
        }
    }
    return false;
}

void Brain::updateBallOut() {
    bool lastBallOut = tree->getEntry<bool>("ball_out");
    double range = lastBallOut ? 4.0 : 2.5;
    double threshold = config->ballOutThreshold;
    threshold += (data->isFreekickKickingOff ? 1.0 : 0.0); // 如果正在踢任意球, 则放宽出界判断
    threshold *= (lastBallOut ? 1.0 : 1.5); // 防止震荡. 如果上次判断为出界, 则放宽出界判断
    tree->setEntry<bool>("ball_out", isBallOut(threshold, 10.0) && data->ball.range < range); // 严格通过定位判断是否出界
}

double Brain::distToBorder() {
    vector<Line> borders;
    auto fd = config->fieldDimensions;
    borders.push_back({fd.length / 2, fd.width / 2, -fd.length / 2, fd.width / 2});
    borders.push_back({fd.length / 2, -fd.width / 2, -fd.length / 2, -fd.width / 2});
    borders.push_back({fd.length / 2, fd.width / 2, fd.length / 2, -fd.width / 2});
    borders.push_back({-fd.length / 2, fd.width / 2, -fd.length / 2, -fd.width / 2});
    double maxDist = -100;
    Point2D robot = {data->robotPoseToField.x, data->robotPoseToField.y};
    for (int i = 0; i < borders.size(); i++) {
        auto line = borders[i];
        double dist = pointPerpDistToLine(robot, line);
        if (dist > maxDist) maxDist = dist;
    }
    return maxDist;
}

bool Brain::isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio, double yRatio) {
    double x = (boundingBox.xmin + boundingBox.xmax) / 2.0;
    double y = (boundingBox.ymin + boundingBox.ymax) / 2.0;

    return (x  > config->camPixX * (1 - xRatio) / 2)
        && (x < config->camPixX * (1 + xRatio) / 2)
        && (y > config->camPixY * (1 - yRatio) / 2)
        && (y < config->camPixY * (1 + yRatio) / 2);
}

bool Brain::isDefensing() {
    bool isFreeKick = tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    bool isKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    
    return isFreeKick && (!isKickoffSide);
}

bool Brain::isTrueFallRecoveryState(RobotRecoveryState state) const
{
    return state == RobotRecoveryState::IS_FALLING ||
           state == RobotRecoveryState::HAS_FALLEN ||
           state == RobotRecoveryState::IS_GETTING_UP;
}

bool Brain::isFallLocalizationHoldActive() const
{
    return fallLocalizationHoldActive_;
}

void Brain::updateFallLocalizationHoldState(bool holdActive)
{
    if (holdActive && !fallLocalizationHoldActive_) {
        fallLocalizationHoldActive_ = true;
        fallLocalizationHoldPose_ = data->robotPoseToField;
        fallLocalizationHoldOdomToField_ = data->odomToField;
        RCLCPP_WARN(
            get_logger(),
            "Freezing field localization during fall recovery at (%.3f, %.3f, %.3f)",
            fallLocalizationHoldPose_.x,
            fallLocalizationHoldPose_.y,
            fallLocalizationHoldPose_.theta);
    } else if (!holdActive && fallLocalizationHoldActive_) {
        fallLocalizationHoldActive_ = false;
        RCLCPP_INFO(
            get_logger(),
            "Released fall localization hold at (%.3f, %.3f, %.3f)",
            fallLocalizationHoldPose_.x,
            fallLocalizationHoldPose_.y,
            fallLocalizationHoldPose_.theta);
    }
}

void Brain::reanchorFrozenPoseToCurrentOdom(const Pose2D &targetPose)
{
    const double xOr =
        -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x -
        sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    const double yOr =
        sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x -
        cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    const double thetaOr = -data->robotPoseToOdom.theta;
    transCoord(
        xOr, yOr, thetaOr,
        targetPose.x, targetPose.y, targetPose.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta);
    data->robotPoseToField = targetPose;
}

void Brain::storeRobotPoseToOdomSample(const rclcpp::Time &stamp, const Pose2D &pose)
{
    if (stamp.nanoseconds() <= 0)
        return;

    constexpr int64_t kOdomPoseBufferNs = 5'000'000'000LL;
    std::lock_guard<std::mutex> lock(odomPoseBufferMutex_);
    odomPoseBuffer_.push_back(OdomPoseSample{stamp, pose});
    const int64_t newestNs = stamp.nanoseconds();
    while (!odomPoseBuffer_.empty() &&
           odomPoseBuffer_.front().stamp.get_clock_type() == stamp.get_clock_type() &&
           newestNs - odomPoseBuffer_.front().stamp.nanoseconds() > kOdomPoseBufferNs) {
        odomPoseBuffer_.pop_front();
    }
    while (odomPoseBuffer_.size() > 1000)
        odomPoseBuffer_.pop_front();
}

bool Brain::lookupRobotPoseToOdomAt(
    const rclcpp::Time &stamp, Pose2D &pose, double *ageMs) const
{
    if (ageMs != nullptr)
        *ageMs = -1.0;
    if (stamp.nanoseconds() <= 0)
        return false;

    constexpr int64_t kMaxLookupAgeNs = 500'000'000LL;
    std::lock_guard<std::mutex> lock(odomPoseBufferMutex_);
    if (odomPoseBuffer_.empty())
        return false;

    const int64_t targetNs = stamp.nanoseconds();
    auto upperIt = std::lower_bound(
        odomPoseBuffer_.begin(), odomPoseBuffer_.end(), targetNs,
        [](const OdomPoseSample &sample, int64_t target) {
            return sample.stamp.nanoseconds() < target;
        });

    if (upperIt != odomPoseBuffer_.end() && upperIt != odomPoseBuffer_.begin()) {
        const auto lowerIt = std::prev(upperIt);
        if (lowerIt->stamp.get_clock_type() == stamp.get_clock_type() &&
            upperIt->stamp.get_clock_type() == stamp.get_clock_type()) {
            const int64_t lowerNs = lowerIt->stamp.nanoseconds();
            const int64_t upperNs = upperIt->stamp.nanoseconds();
            const int64_t nearestAbsDiffNs = std::min(targetNs - lowerNs, upperNs - targetNs);
            if (nearestAbsDiffNs >= 0 && nearestAbsDiffNs <= kMaxLookupAgeNs) {
                const double denom = static_cast<double>(upperNs - lowerNs);
                const double alpha = denom > 0.0
                    ? static_cast<double>(targetNs - lowerNs) / denom : 0.0;
                pose.x = lowerIt->pose.x + alpha * (upperIt->pose.x - lowerIt->pose.x);
                pose.y = lowerIt->pose.y + alpha * (upperIt->pose.y - lowerIt->pose.y);
                pose.theta = toPInPI(
                    lowerIt->pose.theta +
                    alpha * toPInPI(upperIt->pose.theta - lowerIt->pose.theta));
                if (ageMs != nullptr)
                    *ageMs = static_cast<double>(nearestAbsDiffNs) / 1e6;
                return true;
            }
        }
    }

    auto bestIt = odomPoseBuffer_.end();
    int64_t bestAbsDiffNs = std::numeric_limits<int64_t>::max();
    for (auto it = odomPoseBuffer_.begin(); it != odomPoseBuffer_.end(); ++it) {
        if (it->stamp.get_clock_type() != stamp.get_clock_type())
            continue;
        const int64_t absDiffNs = std::llabs(it->stamp.nanoseconds() - targetNs);
        if (absDiffNs < bestAbsDiffNs) {
            bestAbsDiffNs = absDiffNs;
            bestIt = it;
        }
    }
    if (bestIt == odomPoseBuffer_.end() || bestAbsDiffNs > kMaxLookupAgeNs)
        return false;
    pose = bestIt->pose;
    if (ageMs != nullptr)
        *ageMs = static_cast<double>(bestAbsDiffNs) / 1e6;
    return true;
}

void Brain::calibrateOdom(double x, double y, double theta, bool alignToVisionTime)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    updateFallLocalizationHoldState(isTrueFallRecoveryState(data->recoveryState));
    if (isFallLocalizationHoldActive()) {
        reanchorFrozenPoseToCurrentOdom(fallLocalizationHoldPose_);
        fallLocalizationHoldOdomToField_ = data->odomToField;
        return;
    }

    auto markings = data->getMarkings();
    Pose2D calibrationOdom = data->robotPoseToOdom;
    bool odomTimeMatched = false;
    double odomTimeAgeMs = -1.0;

    // 定位器使用的是标志物被拍摄时的机器人坐标。使用同一时刻的 odom
    // 建立 odom->field，再由当前 odom 推进到当前场地位姿。
    rclcpp::Time localizationFrameTime;
    for (const auto &marking : markings) {
        if (marking.timePoint.nanoseconds() <= 0)
            continue;
        if (localizationFrameTime.nanoseconds() <= 0 ||
            marking.timePoint.nanoseconds() > localizationFrameTime.nanoseconds()) {
            localizationFrameTime = marking.timePoint;
        }
    }

    if (alignToVisionTime && localizationFrameTime.nanoseconds() > 0 &&
        data->timeLastOdom.nanoseconds() > 0 &&
        data->timeLastOdom.get_clock_type() == localizationFrameTime.get_clock_type())
        odomTimeMatched = lookupRobotPoseToOdomAt(
            localizationFrameTime, calibrationOdom, &odomTimeAgeMs);

    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(calibrationOdom.theta) * calibrationOdom.x - sin(calibrationOdom.theta) * calibrationOdom.y;
    y_or = sin(calibrationOdom.theta) * calibrationOdom.x - cos(calibrationOdom.theta) * calibrationOdom.y;
    theta_or = -calibrationOdom.theta;

    
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);


    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);


    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder 
    );

    // robots
    auto robots = data->getRobots();
    for (int i = 0; i < robots.size(); i++) {
        updateFieldPos(robots[i]);
    }
    data->setRobots(robots);

    // goalposts
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        updateFieldPos(goalposts[i]);
    }
    
    // markers
    for (int i = 0; i < markings.size(); i++) {
        updateFieldPos(markings[i]);
    }

    log->setTimeNow();
    log->log(
        "debug/odom_calibration",
        rerun::TextLog(format(
            "vision_odom_matched=%d age_ms=%.1f source=(%.3f, %.3f, %.3f) current=(%.3f, %.3f, %.3f) field=(%.3f, %.3f, %.3f)",
            odomTimeMatched ? 1 : 0,
            odomTimeAgeMs,
            calibrationOdom.x,
            calibrationOdom.y,
            calibrationOdom.theta,
            data->robotPoseToOdom.x,
            data->robotPoseToOdom.y,
            data->robotPoseToOdom.theta,
            data->robotPoseToField.x,
            data->robotPoseToField.y,
            data->robotPoseToField.theta)));

    // relog
    log->setTimeNow();
    // logVisionBox(get_clock()->now());
    vector<GameObject> gameObjects = {};
    if(data->ballDetected) gameObjects.push_back(data->ball);
    for (int i = 0; i < markings.size(); i++) gameObjects.push_back(markings[i]);
    for (int i = 0; i < robots.size(); i++) gameObjects.push_back(robots[i]);
    for (int i = 0; i < goalposts.size(); i++) gameObjects.push_back(goalposts[i]);
    logDetection(gameObjects);
}

void Brain::playSound(string soundName, double blockMsecs, bool allowRepeat)
{
    if (!pubSoundPlay) return;

    static string _lastSound;
    static rclcpp::Time _lastTime;
    static double _lastBlockMsecs = 0;

    auto now = get_clock()->now();
    if (msecsSince(_lastTime) < _lastBlockMsecs) return;

    if (_lastSound == soundName && (!allowRepeat)) return;

    // else
    
    std_msgs::msg::String msg;
    msg.data = soundName;
    pubSoundPlay->publish(msg);

    _lastBlockMsecs = blockMsecs;
    _lastTime = now;
    _lastSound = soundName;
}

void Brain::speak(string text, bool allowRepeat)
{
    auto log_ = [=](string msg) {
        // log->setTimeNow();
        // log->log("debug/speak", rerun::TextLog(msg));
    };

    const double COOLDOWN_MSECS = 2000.;
    if (!pubSpeak) {
        log_("publisher not found");
        return;
    }
    if (!config->soundEnable || config->soundPack != "espeak") {
        log_("config not compatible");
        return;
    }

    static string _lastText;
    static rclcpp::Time _lastTime;

    if (msecsSince(_lastTime) < COOLDOWN_MSECS) {
        log_("cooldown in process");
        return;
    }
    
    if (_lastText == text && (!allowRepeat)) {
        log_("repeat not allowed");
        return;
    }
    
    // else
    _lastTime = get_clock()->now();
    std_msgs::msg::String msg;
    msg.data = text;
    pubSpeak->publish(msg);

    _lastText = text;
}

double Brain::msecsSince(rclcpp::Time time)
{
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    return (now - time).nanoseconds() / 1e6;
}

rclcpp::Time Brain::timePointFromHeader(std_msgs::msg::Header header) {
    auto stamp = header.stamp;
    // nanosec == 0 是合法时间戳；旧代码把它改成 1ns，导致视觉帧无法
    // 与同一时刻的 odom 样本对齐。零时间戳保留为无效值，由调用方回退。
    if (stamp.sec < 0 || (stamp.sec == 0 && stamp.nanosec == 0))
        return rclcpp::Time(0, 0, RCL_ROS_TIME);
    return rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
}


void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/joystick", rerun::TextLog(msg));
    };
    // prtDebug("joy!!", RED_CODE);
    string soundPack = config->soundPack;

    // 通过手柄控制机器人, 不阻塞按键
    if (
        fabs(joy.lx) > 0.1
        || fabs(joy.ly) > 0.1
        || fabs(joy.rx) > 0.1
        || fabs(joy.ry) > 0.1
    ) {
        tree->setEntry<bool>("go_manual", true);
        // prtWarn("GO Manual");
    } else {
        tree->setEntry<bool>("go_manual", false);
        // prtWarn("Axe manual take over end");
    }

    // 按键响应顺序: LT 组合键, RT 组合键, 单键
    if (joy.lt && !joy.rt) { // LT 组合键
        // 用于在线调试参数
        if (joy.hat_u || joy.hat_d)
        {
            config->vxFactor += 0.01 * (joy.hat_u ? 1.0 : -1.0);
            speak(format("vx factor: %.2f", config->vxFactor));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        if (joy.hat_l || joy.hat_r)
        {
            config->yawOffset += 0.01 * (joy.hat_r ? 1.0 : -1.0);
            speak(format("yaw offset: %.2f", config->yawOffset));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        // 用于控制切换不同的状态
        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
            // playSound("sad");
        }
        if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
            // playSound("search");
        }
        if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
            // playSound("exited");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
            log_("SWITCH ROLE");
            // playSound("talk");
        }
    }

    if (joy.rt) { // RT 组合键
        // Nothing for now
    }

    // else, 单键位
    if (!joy.lt && !joy.rt) {
        if (joy.lb) {
            tree->setEntry<bool>("assist_chase", true);
            prtDebug("Assit Chase");
            playSound(soundPack + "-chase", 5000);
        } else {
            tree->setEntry<bool>("assist_chase", false);
            // prtWarn("Exit Assit Chase");
        }
        if (joy.rb) {
            tree->setEntry<bool>("assist_kick", true);
            prtDebug("Assit Kick");
            playSound(soundPack + "-kick", 5000);
        } else {
            tree->setEntry<bool>("assist_kick", false);
            // prtWarn("Exit Assit Kick");
        }

        if (joy.hat_u) {
            playSound(soundPack + "-celebrate", 2000, true);
        } else if (joy.hat_d) {
            playSound(soundPack + "-regret", 2000, true);
        } else if (joy.hat_r) {
            playSound(soundPack + "-ready", 2000, true);
        } else if (joy.hat_l) {
            playSound(soundPack + "-taunt", 2000, true);
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    data->timeLastGamecontrolMsg = get_clock()->now();
    communication->setGameControllerProtocolVersion(msg.version);
    tree->setEntry<bool>("gc_play_stopped", msg.stopped);
    tree->setEntry<int>("gc_set_play", static_cast<int>(msg.set_play));
    tree->setEntry<int>("gc_protocol_version", static_cast<int>(msg.version));
    tree->setEntry<int>("gc_secondary_time", static_cast<int>(msg.secondary_time));

    // 处理比赛的一级状态
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // 比赛的一级状态
    vector<string> gameStateMap = {
        "INITIAL", // 初始化状态, 球员在场外准备
        "READY",   // 准备状态, 球员进场, 并走到自己的始发位置
        "SET",     // 停止动作, 等待裁判机发出开始比赛的指令
        "PLAY",    // 正常比赛
        "END"      // 比赛结束
    };
    if (msg.state >= gameStateMap.size()) {
        prtErr(format("received invalid game controller state %d", msg.state));
        return;
    }
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId); // 我方是否是开球方
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    // 处理比赛的二级状态
    string gameSubStateType;
    switch (static_cast<int>(msg.secondary_state)) {
        case 0:
            gameSubStateType = "NONE";
            data->realGameSubState = "NONE";
            break;
        case 3:
            gameSubStateType = "TIMEOUT"; // 包含两队 timeout 和 裁判 timeout
            data->realGameSubState = "TIMEOUT";
            break;
        // 暂时不处理其它状态, 除 TIMEOUT 外, 都按 FREE_KICK 处理
        case 4:
            // gameSubStateType = "DIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "DIRECT_FREEKICK";
            data->isDirectShoot = true;
            break;
        case 5:
            // gameSubStateType = "INDIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "INDIRECT_FREEKICK";
            break;
        case 6:
            // gameSubStateType = "PENALTY_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "PENALTY_KICK";
            data->isDirectShoot = true;
            break;
        case 7:
            // gameSubStateType = "CORNER_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "CORNER_KICK";
            break;
        case 8:
            // gameSubStateType = "GOAL_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "GOAL_KICK";
            data->isDirectShoot = true;
            break;
        case 9:
            // gameSubStateType = "THROW_IN";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "THROW_IN";
            break;
        default:
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "UNKNOWN_SET_PLAY";
            break;
    }
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};                               // STOP: 停下来; -> GET_READY: 移动到进攻或防守位置; -> SET: 站住不动
    string gameSubState;
    if (msg.version >= 20 && (msg.game_phase == 1 || msg.game_phase == 2)) {
        gameSubStateType = "NONE";
        data->realGameSubState = msg.game_phase == 1 ? "PENALTYSHOOT" : "OVERTIME";
    }
    const bool isSubStateKickOffSide =
        msg.version >= 20
        ? static_cast<int>(msg.kick_off_team) == config->teamId
        : static_cast<int>(msg.secondary_state_info[0]) == config->teamId;

    if (msg.version >= 20) {
        if (gameSubStateType == "FREE_KICK") {
            if (msg.stopped) {
                gameSubState = "STOP";
            } else if (isSubStateKickOffSide) {
                // 我方定位球恢复后沿用原来的普通进攻树，由我方完成开球。
                gameSubStateType = "NONE";
                gameSubState = "PLAY";
                data->isFreekickKickingOff = true;
                data->freekickKickoffStartTime = get_clock()->now();
            } else {
                // 对方定位球恢复后只运行找球/防守站位树；Ball Free 前禁止进入 Kick。
                gameSubState = "GET_READY";
            }
        } else {
            gameSubState = msg.stopped ? "STOP" : "PLAY";
        }
    } else {
        const auto legacySubState = static_cast<unsigned char>(msg.secondary_state_info[1]);
        gameSubState = legacySubState < gameSubStateMap.size()
            ? gameSubStateMap[legacySubState]
            : "STOP";
    }
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);
    const bool penaltyKickActive = gameState == "PLAY" &&
        (data->realGameSubState == "PENALTY_KICK" ||
         data->realGameSubState == "PENALTYSHOOT");
    tree->setEntry<bool>("penalty_kick_active", penaltyKickActive);
    tree->setEntry<bool>("penalty_kick_defending",
        penaltyKickActive && !isSubStateKickOffSide);

    int setPlaySearchDirection = 0;
    const bool farSetPlaySearchEnabled =
        get_parameter("strategy.far_set_play_search.enable").as_bool();
    const bool isActiveSetPlay =
        msg.version >= 20
        && msg.set_play != 0;
    if (farSetPlaySearchEnabled && isActiveSetPlay) {
        if (data->realGameSubState == "GOAL_KICK") {
            // 门球位于获得门球一方所防守的半场：
            // 我方门球去我方半场，对方门球去对方半场。
            setPlaySearchDirection =
                isSubStateKickOffSide ? -1 : 1;
        } else if (data->realGameSubState == "CORNER_KICK") {
            // 角球位于获得角球一方所进攻的半场：
            // 我方角球去对方半场，对方角球去我方半场。
            setPlaySearchDirection =
                isSubStateKickOffSide ? 1 : -1;
        }
    }
    tree->setEntry<int>(
        "gc_opponent_set_play_search_direction",
        setPlaySearchDirection);

    // cout << "game state: " << gameState << " game sub state type: " << gameSubStateType << endl;
    // 找到队的信息
    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    }
    else
    {
        // 数据包中没有包含我们的队，不应该再处理了
        prtErr(format("received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number, msg.teams[1].team_number, config->teamId));
        return;
    }

    data->goalkeeperPlayerId = static_cast<int>(myTeamInfo.goalkeeper);

    int liveCount = 0;
    int oppoLiveCount = 0;
    // 处理判罚状态. penalty[playerId - 1] 代表我方的球员是否处于判罚状态, 处理判罚状态意味着不能移动
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);
        
        if (static_cast<int>(myTeamInfo.players[i].red_card_count) > 0) {
            data->penalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);

        if (static_cast<int>(oppoTeamInfo.players[i].red_card_count) > 0) {
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

    // cout << "penalty: " << data->penalty[0] << " " << data->penalty[1] << " " << data->penalty[2] << " " << data->penalty[3] << endl;
    // cout << "oppo penalty: " << data->oppoPenalty[0] << " " << data->oppoPenalty[1] << " " << data->oppoPenalty[2] << " " << data->oppoPenalty[3] << endl;
    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[config->playerId - 1] != PENALTY_NONE); // 当前 robot 是否被判罚中
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); // 被判罚了, 则需要重新进场, 因此需要重新定位

    // log game state   
    log->setTimeNow();
    log->logToScreen(
        "tick/gamecontrol",
        format("Player: %d  Role: %s PrimaryStriker: %s GameState: %s  SubStateType: %s  SubState: %s UnderPenalty: %d isKickoff: %d isSubStateKickoff: %d SetPlaySearchDir: %d", 
            config->playerId, tree->getEntry<string>("player_role").c_str(), isPrimaryStriker() ? "Yes" : "No", gameState.c_str(), gameSubStateType.c_str(), gameSubState.c_str(), isUnderPenalty, isKickOffSide, isSubStateKickOffSide, setPlaySearchDirection
            ),
        0xFFFFFFFF,
        30.0
    );

    // FOR FUN 处理进球后的庆祝挥手的逻辑
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);
    
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    data->camConnected = true;
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastDet = timePoint; // 用于在调试中输出延迟信息

    auto gameObjects = getGameObjects(msg);

    // 对检测到的对象进行分组
    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalposts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // 为了调试方便, 可以在 config 中设置 treat_person_as_robot, 使得 Person 被当作 Robot 处理
            if (config->treatPersonAsRobot)
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // 分别处理分组后的对象
    detectProcessBalls(balls);
    detectProcessGoalposts(goalposts);
    detectProcessMarkings(markings);
    detectProcessRobots(robots);

    // 处理并记录视野信息
    detectProcessVisionBox(msg);

    // logVisionBox(timePoint);
    logDetection(gameObjects);
}

void Brain::updateLinePosToField(FieldLine& line) {
    double __; // __ is a placeholder for transformations
    transCoord(
        line.posToRobot.x0, line.posToRobot.y0, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x0, line.posToField.y0, __
    );
    transCoord(
        line.posToRobot.x1, line.posToRobot.y1, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x1, line.posToField.y1, __
    );
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    // auto timestamp = msg.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastLineDet = timePoint; // 用于在调试中输出延迟信息

    vector<FieldLine> lines = {};
    FieldLine line;

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations 
    for (int i = 0; i < msg.coordinates.size() / 4; i++) {
        int index = i * 4;
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index];
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1];
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2];
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3];
        updateLinePosToField(line);
        line.timePoint = timePoint;
        // TODO infer line dir and id

        lines.push_back(line);
    }
    lines = processFieldLines(lines);
    data->setFieldLines(lines);

    { // log processed lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};
        vector<string> logLabels = {};
        vector<unsigned int> logColors = {};
        
        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0}, 
                {line.posToRobot.x1, -line.posToRobot.y1}, 
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0}, 
                {line.posToField.x1, -line.posToField.y1}, 
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
            string label;
            unsigned int color = 0xFFFF00FF;
            if (line.type == LineType::GoalLine) {
                label = "GoalLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::TouchLine) {
                label = "TouchLine";
                color = 0xFF0000FF;
            } 
            else if (line.type == LineType::MiddleLine) label = "MiddleLine";
            else if (line.type == LineType::PenaltyArea) label = "PenaltyArea";
            else if (line.type == LineType::GoalArea) label = "GoalArea";
            else if (line.type == LineType::NA) label = "NA";
            else label = "Other"; // should not see this label logged

            label += format(" c = %.1f", line.confidence);

            // if (line.dir == LineDir::Horizontal) label += " Horizontal";
            // else if (line.dir == LineDir::Vertical) label += " Vertical";
            // else label += " NA";

            logLabels.push_back(label);
            logColors.push_back(color);
        };
        log->log(
            "field/det_lines",
            rerun::LineStrips2D(logLinesOnField)
                .with_colors(logColors)
                .with_radii(0.04)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
        // log->log(
        //     "robotframe/det_lines",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(logColors)
        //         .with_radii(0.04)
        //         .with_draw_order(20)
        //         .with_labels(logLabels)
        // );
        log->log(
            "image/det_lines",
            rerun::LineStrips2D(logLinesOnCam)
                .with_colors(logColors)
                .with_radii(1.0)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
    }

    { // log original lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0}, 
                {line.posToRobot.x1, -line.posToRobot.y1}, 
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0}, 
                {line.posToField.x1, -line.posToField.y1}, 
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
        };
        // log->log(
        //     "field/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnField)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "robotframe/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "image/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnCam)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(3.0)
        //         .with_draw_order(10)
        // );
    }
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    const rclcpp::Time callbackTime = get_clock()->now();
    data->timeLastOdom = callbackTime;
    ++data->odomUpdateCount;
    data->odomBuffer.add(msg, callbackTime);

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;
    storeRobotPoseToOdomSample(callbackTime, data->robotPoseToOdom);

    updateFallLocalizationHoldState(isTrueFallRecoveryState(data->recoveryState));
    if (isFallLocalizationHoldActive()) {
        // 起身/倒地时底层 odom 可能跳变；保持最后可信的场地位姿，同时
        // 每帧重锚定 odom->field，使恢复后不会把跳变一次性带入场地坐标。
        reanchorFrozenPoseToCurrentOdom(fallLocalizationHoldPose_);
        fallLocalizationHoldOdomToField_ = data->odomToField;
    } else {
        // 根据 Odom 信息, 更新机器人在 Field 坐标系中的位置
        transCoord(
            data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
            data->odomToField.x, data->odomToField.y, data->odomToField.theta,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    }

    // 发布tf变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = callbackTime;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    // 设置平移部分
    transform.transform.translation.x = data->robotPoseToOdom.x;
    transform.transform.translation.y = data->robotPoseToOdom.y;
    transform.transform.translation.z = 0.0;
    
    // 设置旋转部分（从欧拉角转换为四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, data->robotPoseToOdom.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    log->setTimeNow();
    log->log("debug/odom_callback", rerun::TextLog(format("x: %.3f, y: %.3f, theta: %.3f update_count: %llu", data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta, static_cast<unsigned long long>(data->odomUpdateCount))));
    
    // 广播tf变换
    tf_broadcaster_->sendTransform(transform);

    // Log Odom 信息

    log->setTimeNow();
    auto color = 0x00FF00FF;
    if (!data->tmImAlive) color = 0x006600FF;
    else if (!data->tmImLead) color = 0x00CC00FF;
    string label = format("Cost: %.1f", data->tmMyCost);
    log->logRobot("field/robot", data->robotPoseToField, color, label, true);
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;
    log->log("debug/head_angles", rerun::TextLog(format("pitch: %.1f, yaw: %.1f", data->headYaw, data->headPitch)));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {
        // 未防止摄像头连接不好时, 自动降低分辨率, 影响 CamTrackBall 的计算, 更新分辨率配置
        config->camPixX = msg.width;
        config->camPixY = msg.height;
        log->log("debug/imageCallback", rerun::TextLog(format("img width: %.d, img height: %.d", msg.width, msg.height)));

        cv::Mat image;
        // 根据图像编码格式进行处理
        if (msg.encoding == "nv12" || msg.encoding == "NV12") {
            // NV12: Y plane (H x W) + interleaved UV (H/2 x W)
            size_t expected = (size_t)(msg.width * msg.height * 3 / 2);
            if (msg.data.size() < expected) {
                prtErr(format("NV12 buffer too small. got %zu expect >= %zu", msg.data.size(), expected));
                return;
            }
            cv::Mat yuv(msg.height + msg.height / 2, msg.width, CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
            cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_NV12);
        } else if (msg.encoding == "bgra8") {
            // 创建 OpenCV Mat 对象，处理 BGRA 格式图像
            image = cv::Mat(msg.height, msg.width, CV_8UC4, const_cast<uint8_t *>(msg.data.data()));
            cv::Mat imageBGR;
            // 将 BGRA 转换为 BGR，忽略 Alpha 通道
            cv::cvtColor(image, imageBGR, cv::COLOR_BGRA2BGR);
            image = imageBGR;
        } else if (msg.encoding == "bgr8") {
            // 原有 BGR8 处理逻辑
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        } else if (msg.encoding == "rgb8") {
            // 原有 RGB8 处理逻辑
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        } else {
            // 处理其他编码格式，或者记录错误日志
            prtErr(format("Unsupported image encoding: %s", msg.encoding.c_str()));
            return;
        }

        // 压缩图像
        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10}; // 10 表示压缩质量，可以根据需要调整
        cv::imencode(".jpg", image, compressed_image, compression_params);

        // 将压缩后的图像数据传递给 rerun
        // double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        // log->setTimeSeconds(time);
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose& msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    // 计算 head_to_base 矩阵
    Eigen::Matrix4d headToBase = Eigen::Matrix4d::Identity();
    
    // 从四元数获取旋转矩阵
    Eigen::Quaterniond q(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );
    headToBase.block<3,3>(0,0) = q.toRotationMatrix();
    
    // 设置平移向量
    headToBase.block<3,1>(0,3) = Eigen::Vector3d(
        msg.position.x,
        msg.position.y,
        msg.position.z
    );

    // // 定义并计算 cam_to_head 矩阵
    // Eigen::Matrix4d camToHead;
    // camToHead << 0,  0,  1,  0,
    //             -1,  0,  0,  0,
    //              0, -1,  0,  0,
    //              0,  0,  0,  1;

    // 计算 cam_to_base 矩阵并存储
    data->camToRobot = headToBase * config->camToHead;
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    std::lock_guard<std::recursive_mutex> stateLock(brainStateMutex_);

    // uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,  
    // uint8_t is_recovery_available; // 1 for available, 0 for not available
    // 使用 RobotRecoveryState 结构，将msg里面的msg转换为RobotRecoveryState
    try
    {
        const std::vector<unsigned char>& buffer = msg.msg;
        if (buffer.size() < sizeof(RobotRecoveryStateData)) {
            RCLCPP_WARN(
                get_logger(),
                "Invalid recovery state message size: %zu < %zu",
                buffer.size(), sizeof(RobotRecoveryStateData));
            return;
        }
        RobotRecoveryStateData recoveryState;
        memcpy(&recoveryState, buffer.data(), sizeof(RobotRecoveryStateData));

        vector<RobotRecoveryState> recoveryStateMap = {
            RobotRecoveryState::IS_READY,
            RobotRecoveryState::IS_FALLING,
            RobotRecoveryState::HAS_FALLEN,
            RobotRecoveryState::IS_GETTING_UP
        };
        if (recoveryState.state >= recoveryStateMap.size()) {
            RCLCPP_WARN(get_logger(), "Invalid recovery state value: %u", static_cast<unsigned int>(recoveryState.state));
            return;
        }
        const auto previousState = this->data->recoveryState;
        this->data->recoveryState = recoveryStateMap[static_cast<int>(recoveryState.state)];
        this->data->isRecoveryAvailable = static_cast<bool>(recoveryState.is_recovery_available);
        this->data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);

        const bool wasRecovering = isTrueFallRecoveryState(previousState);
        const bool isRecovering = isTrueFallRecoveryState(this->data->recoveryState);
        if (wasRecovering && !isRecovering && fallLocalizationHoldActive_)
            reanchorFrozenPoseToCurrentOdom(fallLocalizationHoldPose_);
        updateFallLocalizationHoldState(isRecovering);
        if (isRecovering && !wasRecovering) {
            RCLCPP_WARN(get_logger(), "Recovery state entered: %u", static_cast<unsigned int>(recoveryState.state));
        }

        // cout << "recoveryState: " << static_cast<int>(recoveryState.state) << endl;
        // cout << "recovery is available: " << static_cast<int>(recoveryState.is_recovery_available) << endl;
        // cout << "current planner idx: " << static_cast<int>(recoveryState.current_planner_index) << endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


int Brain::markCntOnFieldLine(const string markType, const FieldLine line, const double margin) {
    int cnt = 0;
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto marking = markings[i];
        if (marking.label == markType) {
            Point2D point = {marking.posToField.x, marking.posToField.y};
            if (fabs(pointPerpDistToLine(point, line.posToField)) < margin) {
                cnt += 1;
            }
        }
    }
    return cnt;
}

int Brain::goalpostCntOnFieldLine(const FieldLine line, const double margin) {
    int cnt = 0;
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        auto post = goalposts[i];
        Point2D point = {post.posToField.x, post.posToField.y};
        if (pointMinDistToLine(point, line.posToField) < margin) {
            cnt += 1;
        }
    }
    return cnt;
}

bool Brain::isBallOnFieldLine(const FieldLine line, const double margin) {
    if (!tree->getEntry<bool>("ball_location_known") &&
        !tree->getEntry<bool>("tm_ball_pos_reliable")) {
        return false;
    }
    auto ballPos = data->ball.posToField;
    Point2D point = {ballPos.x, ballPos.y};
    return fabs(pointPerpDistToLine(point, line.posToField)) < margin;
}

void Brain::identifyFieldLine(FieldLine& line) {
    auto mapLines = config->mapLines;
    FieldLine mapLine;
    double confidence;
    line.type = LineType::NA;

    double bestConfidence = 0;
    double secondBestConfidence = 0;
    int bestIndex = -1;
    for (int i = 0; i < mapLines.size(); i++) {
        mapLine = mapLines[i];
        confidence = line.dir == mapLine.dir ? 
            probPartOfLine(line.posToField, mapLine.posToField)
            : 0.0;

        // Boost confidence with other features
        if (mapLine.type == LineType::GoalLine) { 
            confidence += 0.3 * markCntOnFieldLine("TCross", line, 0.2);
            confidence += 0.5 * goalpostCntOnFieldLine(line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "CORNER_KICK")
            ) confidence += 0.3; // 角球时, 球在底线上
        }
        if (mapLine.type == LineType::MiddleLine) {
            confidence += 0.3 * markCntOnFieldLine("XCross", line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK")
            ) confidence += 0.3; // 门球时, 球在中线上
        }
        if (mapLine.type == LineType::TouchLine) {
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK" || data->realGameSubState == "CORNER_KICK" || data->realGameSubState == "THROW_IN")
            ) confidence += 0.3; // 发角球, 门球和边线球时, 球在边线上
        }
        
        // 防止将 goalarealine 误认为 goalline
        auto fd = config->fieldDimensions;
        if (
            mapLine.type == LineType::GoalLine
            && fabs(line.posToField.y0) < fd.goalAreaWidth / 2 + 0.5
            && fabs(line.posToField.y1) < fd.goalAreaWidth / 2 + 0.5
        ) confidence -= 0.3;

        // 防止将 penalty area 误认为 touchline
        if (
            mapLine.type == LineType::TouchLine
            && min(fabs(line.posToField.x0), fabs(line.posToField.x1)) > fd.length / 2.0 -  fd.penaltyAreaLength - 0.5
            && line.posToField.x0 * line.posToField.x1 > 0
        ) confidence -= 0.3;

        double length = norm(line.posToField.x0 - line.posToField.x1, line.posToField.y0 - line.posToField.y1);
        if (length < 0.5) confidence -= 0.5;
        else if (length < 1.0) confidence -= 0.1;
        
        if (confidence > bestConfidence) {
            secondBestConfidence = bestConfidence;
            bestConfidence = confidence;
            bestIndex = i;
        }
    }

    if (bestConfidence - secondBestConfidence < 0.5) bestConfidence -= 0.5;



    if (bestIndex >= 0 && bestIndex < mapLines.size()) {
        line.type = mapLines[bestIndex].type;
        line.half = mapLines[bestIndex].half;
        line.side = mapLines[bestIndex].side;
        line.confidence = bestConfidence;
        return;
    }

    // else 
    line.type = LineType::NA;
    line.half = LineHalf::NA;
    line.side = LineSide::NA;
    line.confidence = 0.0;
    return;
}

void Brain::identifyMarking(GameObject& marking) {
    double minDist = 100;
    double secMinDist = 100;
    int mmIndex = -1;
    for (int i = 0; i < config->mapMarkings.size(); i++) {
       auto mm = config->mapMarkings[i];
       
       if (mm.type != marking.label) continue;

       double dist = norm(marking.posToField.x - mm.x, marking.posToField.y - mm.y);

       if (dist < minDist) {
           secMinDist = minDist;
           minDist = dist;
           mmIndex = i;
       } else if (dist < secMinDist) {
           secMinDist = dist; 
       }
    }

    auto fd = config->fieldDimensions;
    if (
        mmIndex >=0 && mmIndex < config->mapMarkings.size()
        && minDist < 1.5 * 14 / fd.length // 1.0 for adultsize
        && secMinDist - minDist > 1.5 * 14 / fd.length // 2.0 for adultsize
        // && marking.confidence > 70 
    ) {
        marking.id = mmIndex;
        marking.name = config->mapMarkings[mmIndex].name;
        marking.idConfidence = 1.0;
    } else {
        marking.id = -1;
        marking.name = "NA";
        marking.idConfidence = 0.0;
    }
}


void Brain::identifyGoalpost(GameObject& goalpost) {
    string side = "NA";
    string half = "NA";
    if (goalpost.posToField.x > 0) half = "O";
    else half = "S";

    if (goalpost.posToField.y > 0) side = "L";
    else side = "R";
    
    goalpost.id = 0;
    goalpost.name = half + side;
    goalpost.idConfidence = 1.0;
    // TODO 参考 markings, 做更为精细的 goalpostid
}

vector<FieldLine> Brain::processFieldLines(vector<FieldLine>& fieldLines) {
    vector<FieldLine> original = fieldLines;
    vector<FieldLine> res;
    

    int sizeBefore = original.size();
    // merge lines that are actually the same line
    for (int i = 0; i < original.size(); i++) {
        for (int j = i + 1; j < original.size(); j++) {
            auto line1 = original[i].posToField;
            auto line2 = original[j].posToField;
            if (isSameLine(line1, line2, 0.1, 1.0)) {
                FieldLine mergedLine;
                mergedLine.posToField = mergeLines(line1, line2);
                mergedLine.posToRobot = mergeLines(original[i].posToRobot, original[j].posToRobot);
                mergedLine.posOnCam = mergeLines(original[i].posOnCam, original[j].posOnCam);
                mergedLine.timePoint = original[i].timePoint;

                // replace first line in original with merged line and remove second line
                original[i] = mergedLine;
                original.erase(original.begin() + j);
                j--;
            }
        }
    }
    int sizeAfter = original.size();
    // prtWarn(format("Merged %d lines into %d lines", sizeBefore, sizeAfter));
    // return original;
    // filter out lines that are too short and infer direction while ditch lines whose dir cannot be inferred
    double valve = 0.2;
    for (int i = 0; i < original.size(); i++) {
        auto line = original[i];
        auto lineDir = atan2(line.posToField.y1 - line.posToField.y0, line.posToField.x1 - line.posToField.x0);

        if (fabs(toPInPI(lineDir - M_PI)) < 0.1 || fabs(lineDir) < 0.1) line.dir = LineDir::Vertical;
        else if (fabs(toPInPI(lineDir - M_PI/2)) < 0.1 || fabs(toPInPI(lineDir + M_PI/2)) < 0.1) line.dir = LineDir::Horizontal;
        else continue;

        // if line is direction can be verified, check if it is long enough
        if (lineLength(line.posToField) > valve) {
            res.push_back(line);
        }
    }

    // identify each line 
    for (int i = 0; i < res.size(); i++) {
        identifyFieldLine(res[i]);
    }
    return res;
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    // auto timestamp = detections.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(detections.header);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;
        gObj.color = obj.color;

        if (obj.target_uv.size() == 2)
        { // 地面标志点的精确像素位置信息
            gObj.precisePixelPoint.x = static_cast<double>(obj.target_uv[0]);
            gObj.precisePixelPoint.y = static_cast<double>(obj.target_uv[1]);
        }

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        // 深度优先
        // if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        // { // 深度测距成功， 以深度测距为准
        //     gObj.posToRobot.x = obj.position[0];
        //     gObj.posToRobot.y = obj.position[1];
        // }
        // else
        // { // 深度测距失败，以投影距离为准
        //     gObj.posToRobot.x = obj.position_projection[0];
        //     gObj.posToRobot.y = obj.position_projection[1];
        // } // 注意，z 值没有用到

        // 不用深度测距, 直接用投影距离
        gObj.posToRobot.x = obj.position_projection[0];
        gObj.posToRobot.y = obj.position_projection[1];

        // 计算角度
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = pitchToGroundObject(config->robotHeight, gObj.range); // 注意这是一个近似值

        // 计算对象在 field 坐标系中的位置
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // 注意, z 没有在其它地方使用, 这里仅为参数占位使用
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    static rclcpp::Time lastSeenRealBallTime;
    double bestConfidence = 0;
    int indexRealBall = -1;  // 认为哪一个球是真的, -1 表示没有检测到球

    // 找出最可能的真球
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];
        auto oldBall = data->ball;

        // 防止把天上的灯识别为球
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 15.0)
            continue;

        // 排除在场外太远的球, 注意这个功能会影响 cahse 等功能. 先不采用.
        // if (isBallOut(2.0, 2.0))
        //     continue;

        // 如果检测出的球与上次看到的球, 距离和时间都很近, 则对其 confidence 进行适当加分
        // double c = ballObj.confidence;
        // double oldC = oldBall.confidence;
        // double msecs = msecsSince(oldBall.timePoint);
        // double dist = norm(ballObj.posToField.x - oldBall.posToField.x, ballObj.posToField.y - oldBall.posToField.y);
        // ballObj.confidence += 0.5 * max(oldC, 0.0) * max(1 - msecs/1000, 0.0) * max(1 - dist / 2, 0.0);
        // ballObj.confidence = min(100.0, ballObj.confidence);
        // log->setTimeNow();
        // log->log("/debug/oldc_newc", rerun::TextLog(format(
        //     "oldc: %.2f  newc: %.2f",
        //     oldC,
        //     ballObj.confidence
        // )));

        // 判断: 如果置信度太低, 则认为是误检
        if (ballObj.confidence < config->ballConfidenceThreshold)
            continue;

        // TODO 加入更多排除参数, 例如在身体上, 明显在球场外, 位置突然大幅度变化等
        // 被遮挡的条件要加入. 如果突然消失, 没有遮挡的话, 则只相信一小会儿, 如果有遮挡, 可以相信比较长的时间.

        // 找出剩下的球中, 置信度最高的
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    auto now = this->get_clock()->now();

    if (indexRealBall >= 0)
    { // 检测到球了
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];
        data->ball.confidence = bestConfidence;

        tree->setEntry<bool>("ball_location_known", true);
        updateBallOut();
        updateBallPrediction();

        lastSeenRealBallTime = now;
        data->lose_ball = false;        
    }
    else
    { // 没有检测到球
        log->setTimeNow();
        // log->log("image/detection_boxes_realball", rerun::Clear::FLAT);
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;

        if (lastSeenRealBallTime.seconds() > 0.0)
        {
            double msecs = (now - lastSeenRealBallTime).nanoseconds() / 1e6;
            data->lose_ball = (msecs > get_parameter("strategy.ball_lost_msecs").as_double());
        }
        else
        {
            data->lose_ball = false;
        }

        // data->ball.confidence = 0; // DO NOT set confidence to 0, confidence decay depends on this.
    }

    // 计算机器人到球的向量, 在 field 坐标系中的方向。
    // 尚未获得有效球位置时使用明确的中性值。
    if (tree->getEntry<bool>("ball_location_known") ||
        tree->getEntry<bool>("tm_ball_pos_reliable")) {
        data->robotBallAngleToField = atan2(
            data->ball.posToField.y - data->robotPoseToField.y,
            data->ball.posToField.x - data->robotPoseToField.x);
    } else {
        data->robotBallAngleToField = 0.0;
    }
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    // // for testing 测距稳定性 ---------
    // for (int i = 0; i < markingObjs.size(); i++) {
    //    auto m = markingObjs[i];
    //    if (m.label != "PenaltyPoint" || m.posToField.x < 0.0) continue;

    //    double range = norm(m.posToField.x - data->robotPoseToField.x, m.posToField.y - data->robotPoseToField.y);

    //    log->setTimeNow();
    //    log->log("debug/penalty_point/range", rerun::Scalar(range));
    //    log->log("debug/penalty_point/x", rerun::Scalar(m.posToField.x));
    //    log->log("debug/penalty_point/y", rerun::Scalar(m.posToField.y));
    // }
    // // end testing

    const double confidenceValve = 50; // confidence 低于这个阈值, 排除
    vector<GameObject> markings = {};
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (marking.confidence < confidenceValve)
            continue;

        // 排除天的上误识别标记
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 15.0)
            continue;

        // 如果通过了重重考验, 则记入 brain
        identifyMarking(marking);
        markings.push_back(marking);
    }
    data->setMarkings(markings);

    // log identified markings
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];
        if (markings[i].id >= 0) {
            circles.push_back(log->circle(m.posToField.x, -m.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", m.name.c_str(), m.idConfidence));
        }
    }
    
    log->log("field/identified_markings",
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
       .with_radii(0.01)
       .with_labels(labels)
       .with_colors(0xFFFFFFFF));
}

void Brain::detectProcessGoalposts(const vector<GameObject> &goalpostObjs)
{
    const double confidenceValve = 50; // confidence 低于这个阈值, 排除
    vector<GameObject> goalposts = {};

    for (int i = 0; i < goalpostObjs.size(); i++) {
        auto goalpost = goalpostObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (goalpost.confidence < confidenceValve)
            continue;

        identifyGoalpost(goalpost);
        goalposts.push_back(goalpost);
    }
    data->setGoalposts(goalposts);

    // log identified goalposts
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < goalposts.size(); i++) {
        auto p = goalposts[i];
        if (goalposts[i].id >= 0) {
            circles.push_back(log->circle(p.posToField.x, -p.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", p.name.c_str(), p.idConfidence));
        }
    }
    
    // log->log("field/identified_goalposts",
    //     rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
    //     .with_radii(0.01)
    //     .with_labels(labels)
    //     .with_colors(0xFFFFFFFF));
}


void Brain::detectProcessRobots(const vector<GameObject> &robotObjs) {
    // auto robots = data->getRobots();

    // for (int i = 0; i < robotObjs.size(); i++) {
    //     auto rbt = robotObjs[i];
    //     if (rbt.confidence < 50) continue;

    //     // find nearest robot in memory
    //     double minDist = 1e6; 
    //     int minIndex = -1;
    //     for (int j = 0; j < robots.size(); j++) {
    //         auto rm = robots[j];
    //         double dist = norm(rm.posToField.x - rbt.posToField.x, rm.posToField.y - rbt.posToField.y);
    //         if (dist < minDist) {
    //             minDist = dist; 
    //             minIndex = j;
    //         }
    //     }
    //     // prtDebug(format("minDist = %.2f", minDist));
    //     if (minDist < 0.5) { // 认为是同一个机器人
    //         robots[minIndex] = rbt;
    //     } else { // 认为是不同的机器人
    //         robots.push_back(rbt);
    //     }
    // }
    // // 注意这里不清理已经看不见的机器人, 而是在 updateMemory 中进行处理.

    vector<GameObject> robots = {};
    for (int i = 0; i < robotObjs.size(); i++) {
        auto rbt = robotObjs[i];
        if (rbt.confidence < 50) continue;
        
        // else
        robots.push_back(rbt);
    }

    data->setRobots(robots);
}


void Brain::detectProcessVisionBox(const vision_interface::msg::Detections &msg) {    
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    // 处理并记录视野信息
    VisionBox vbox;
    vbox.timePoint = timePoint;
    for (int i = 0; i < msg.corner_pos.size(); i++) vbox.posToRobot.push_back(msg.corner_pos[i]);

    // 处理左上与右上两点 x 小于 0 , 实际为无限远的场景
    const double VISION_LIMIT = 20.0;
    vector<vector<double>> v = {};
    for (int i = 0; i < 4; i++) {
        int start = i; int end = (i + 1) % 4;
        v.push_back({vbox.posToRobot[end * 2] - vbox.posToRobot[start * 2], vbox.posToRobot[end * 2 + 1] - vbox.posToRobot[start * 2 + 1]});
        v.push_back({-vbox.posToRobot[end * 2] + vbox.posToRobot[start * 2], -vbox.posToRobot[end * 2 + 1] + vbox.posToRobot[start * 2 + 1]});
    }

    for (int i = 0; i < 2; i++) {
        double ox = vbox.posToRobot[2* i]; double oy = vbox.posToRobot[2 * i + 1];
        if (
            (i == 0 && crossProduct(v[5], v[6]) < 0)
            || (i == 1 && crossProduct(v[3], v[4]) < 0)
        ){
            vbox.posToRobot[2 * i] = -ox / fabs(ox) * VISION_LIMIT;
            vbox.posToRobot[2 * i + 1] = -oy / fabs(oy) * VISION_LIMIT;
        }
    }

    // 转换到 field 坐标系中
    for (int i = 0; i < 5; i++) {
        double xr, yr, xf, yf, __;
        xr = vbox.posToRobot[2 * i];
        yr = vbox.posToRobot[2 * i + 1];
        transCoord(
            xr, yr, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            xf, yf, __
        );
        vbox.posToField.push_back(xf);
        vbox.posToField.push_back(yf);
    }
    
    // 一次性将结果赋值到 data 中
    data->visionBox = vbox;
}

void Brain::logVisionBox(const rclcpp::Time &timePoint) {
    if (data->visionBox.posToField.size() >= 10) {
        auto v = data->visionBox.posToField;

        rerun::LineStrip2D logLines({{v[0], -v[1]}, {v[2], -v[3]}, {v[4], -v[5]}, {v[6], -v[7]}, {v[0], -v[1]}});
        log->setTimeSeconds(timePoint.seconds());
        log->log(
            "field/visionBox",
            rerun::LineStrips2D(logLines)
            .with_colors(0x0000FFCC)
            .with_radii(0.01)
        );
    }
}

void Brain::logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox) {
    if (gameObjects.size() == 0) {
        if (logBoundingBox) log->log("image/detection_boxes", rerun::Clear::FLAT);
        log->log("field/detection_points", rerun::Clear::FLAT);
        // log->log("robotframe/detection_points", rerun::Clear::FLAT);
        return;
    }
    
    // else 
    rclcpp::Time timePoint = gameObjects[0].timePoint;
    log->setTimeSeconds(timePoint.seconds());

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
        {"PenaltyPoint", rerun::Color(0xFF9900FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    vector<double> radiis;

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(
            format("%s x:%.2f y:%.2f c:%.1f", 
                label == "Opponent" || label == "Person" ? (label + "[" + obj.color + "]").c_str() : label.c_str(), 
                obj.posToRobot.x, 
                obj.posToRobot.y, 
                obj.confidence)
            )
        );
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y}); // y 取反是因为 rerun Viewer 的坐标系是左手系。转一下看起来更方便。
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        // if (obj.label == "Opponent") radiis.push_back(0.5);
        radiis.push_back(0.1);

        auto color = rerun::Color(0xFFFFFFFF);

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            color = detectColorMap[label];
        }
        else
        {
            // do nothing, use default
            // colors.push_back(rerun::Color(0xFFFFFFFF));
        }
        if (label == "Ball" && isBallOut(0.2, 10.0))
            color = rerun::Color(0x000000FF);
        if (label == "Ball" && obj.confidence < config->ballConfidenceThreshold)
            color = rerun::Color(0xAAAAAAFF);
        colors.push_back(color);
    }

    
    if (logBoundingBox) log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
                 .with_radii(radiis)
             // .with_labels(labels)
    );
    // log->log("robotframe/detection_points",
    //          rerun::Points2D(points_r)
    //              .with_colors(colors)
    //              .with_radii(radiis)
    //          // .with_labels(labels)
    // );
}


void Brain::logMemRobots() {
    auto rbts = data->getRobots();
    // prtDebug(format("logMemRobots called, robotsize = %d", rbts.size()), RED_CODE);

    if (rbts.size() == 0) {
        log->log("field/mem_robots", rerun::Clear::FLAT);
        // log->log("robotframe/mem_robots", rerun::Clear::FLAT);
        return;
    }
    
    // else 
    log->setTimeNow();
    // vector<rerun::Vec2D> points;
    vector<rerun::LineStrip2D> circles;
    vector<rerun::Vec2D> points_r; // robot frame
    for (int i = 0; i < rbts.size(); i++)
    {
        auto rbt = rbts[i];
        log->logRobot("field/robots", Pose2D({rbt.posToField.x, rbt.posToField.y, -M_PI}), 0xFF0000FF);
        // circles.push_back(log->circle(rbt.posToField.x, -rbt.posToField.y, 0.5)); // y 取反是因为 rerun Viewer 的坐标系是左手系。转一下看起来更方便。
        // points_r.push_back(rerun::Vec2D{rbt.posToRobot.x, -rbt.posToRobot.y});
    }

    // log->log("field/mem_robots",
    //          rerun::LineStrips2D(circles)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.01)
    //          // .with_labels(labels)
    // );
    // log->log("robotframe/mem_robots",
    //          rerun::Points2D(points_r)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.5)
    //          // .with_labels(labels)
    // );
}

void Brain::logObstacles() {
    // log->setTimeNow();
    // time is set on the outside
    
    // 记录障碍物(即有被占用的网格)
    auto obs = data->getObstacles();
    vector<rerun::Vec2D> points;
    vector<rerun::Color> colors;
    vector<rerun::Text> labels;
    const int occThreshold = get_parameter("obstacle_avoidance.occupancy_threshold").as_int();
    for (int i = 0; i < obs.size(); i++) {
        auto o = obs[i];

        if (o.confidence < occThreshold) continue; // 这个逻辑覆盖了后面的逻辑, 注掉可以以不同颜色 log 不同的置信度.

        points.push_back(rerun::Vec2D{o.posToField.x, -o.posToField.y});
        double mem_msecs = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
        auto age = msecsSince(o.timePoint);
        uint8_t alpha = static_cast<uint8_t>(0xFF - 0xFF * age / mem_msecs);
        uint32_t color = (o.confidence > occThreshold) ? (0xFF000000 | alpha) : (0xFFFF0000 | alpha);
        colors.push_back(rerun::Color(color));

        labels.push_back(rerun::Text(format("count: %.0f age: %.0fms", o.confidence, age)));
    }
    log->log(
        "field/obstacles", 
        rerun::Points2D(points)
        .with_colors(colors)
        .with_labels(labels)
        .with_radii(0.1)
    );
}

void Brain::logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid_occupied, vector<rerun::Vec3D> &points_robot) {
    // time is set on the outside
    const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 网格大小
    const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
    const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
    const double y_max = -y_min;

    // 记录原始点云和网格
    vector<rerun::Position3D> vertices;
    vector<rerun::Color> vertex_colors;
    vector<array<uint32_t, 3>> triangle_indices;
    const int OCCUPANCY_THRESHOLD = get_parameter("obstacle_avoidance.occupancy_threshold").as_int(); // 设置一个显示用的阈值

    for (int i = 0; i < grid_x_count; i++) {
        for (int j = 0; j < grid_y_count; j++) {
            if (grid_occupied[i][j] > 0) {
                // 计算有障碍网格的四个顶点坐标
                double x0 = x_min + i * grid_size;
                double y0 = y_min + j * grid_size;
                double x1 = x0 + grid_size;
                double y1 = y0 + grid_size;

                // 添加四个顶点
                uint32_t base_index = vertices.size();
                vertices.push_back({x0, y0, 0.0});
                vertices.push_back({x1, y0, 0.0});
                vertices.push_back({x1, y1, 0.0});
                vertices.push_back({x0, y1, 0.0});

                // 设置颜色：根据占用情况设置不同的红色
                rerun::Color color;
                if (grid_occupied[i][j] > OCCUPANCY_THRESHOLD) {
                    color = rerun::Color(255, 0, 0, 255);  // RGBA, 红色
                } else {
                    color = rerun::Color(255, 255, 0, 255);  // RGBA, 黄色
                }
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);

                // 添加两个三角形面
                triangle_indices.push_back({base_index, base_index + 1, base_index + 2});
                triangle_indices.push_back({base_index, base_index + 2, base_index + 3});
            }
        }
    }

    vector<uint32_t> point_colors;
    for (auto &point : points_robot) {
        float z_val = std::clamp(point.z(), 0.0f, 1.0f);
        const double obstacleMinHeight = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
        if (z_val < obstacleMinHeight) {
            point_colors.push_back(0x0000FFFF);
        } else {
            uint8_t r = static_cast<uint8_t>(z_val * 255);
            uint8_t g = static_cast<uint8_t>((1 - z_val) * 255);
            point_colors.push_back((r << 24) | (g << 16) | 0xFF);
        }
    }
    
    log->log("depth/depth_points",
            rerun::Points3D(points_robot)
                .with_radii(0.01)
                .with_colors(point_colors)
    );
    
    log->log("depth/grid_mesh",
            rerun::Mesh3D(vertices)
                .with_vertex_colors(vertex_colors)
                .with_triangle_indices(triangle_indices)
    );

    // 记录 ball exclusion box
    double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
    double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
    log->log(
        "depth/ball_exclusion_box",
        rerun::Boxes3D::from_centers_and_half_sizes(
            {{ data->ball.posToRobot.x, data->ball.posToRobot.y, h/2}},
            {{ r, r, h/2}})
        .with_colors(0x00FF0044)     // 半透明绿色
    );
}

void Brain::logDebugInfo() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/brain_tick", rerun::TextLog(msg));
    };
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string isLead = data->tmImLead ? "ON" : "OFF";
    string ballOut = tree->getEntry<bool>("ball_out") ? "YES" : "NO";
    string ballDetected = data->ballDetected ? "YES" : "NO";
    string decision = tree->getEntry<string>("decision");
    string freeKickKickingOff = data->isFreekickKickingOff ? "YES" : "NO";
    string directShoot = data->isDirectShoot ? "YES" : "NO";
    string primaryStriker = isPrimaryStriker() ? "YES" : "NO";
    log_(format("Game State: %s, SubState: %s, SubStateType: %s, Lead: %s, Decision: %s, FreeKickKickingOff: %s, DirectShoot: %s, PrimaryStriker: %s",
        gameState.c_str(), gameSubState.c_str(), gameSubStateType.c_str(), isLead.c_str(), decision.c_str(), freeKickKickingOff.c_str(), directShoot.c_str(), primaryStriker.c_str()));

    log->setTimeNow();
    log->log("debug/my_cost_scalar", rerun::Scalar(data->tmMyCost));
    log->log("debug/my_lead_scalar", rerun::Scalar(data->tmImLead));
}

void Brain::updateRelativePos(GameObject &obj) {
    Pose2D pf;
    pf.x = obj.posToField.x;
    pf.y = obj.posToField.y;
    pf.theta = 0;
    Pose2D pr = data->field2robot(pf);
    obj.posToRobot.x = pr.x;
    obj.posToRobot.y = pr.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = pitchToGroundObject(config->robotHeight, obj.range);
}

void Brain::updateFieldPos(GameObject &obj) {
    Pose2D pr;
    pr.x = obj.posToRobot.x;
    pr.y = obj.posToRobot.y;
    pr.theta = 0;
    Pose2D pf = data->robot2field(pr);
    obj.posToField.x = pf.x;
    obj.posToField.y = pf.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = pitchToGroundObject(config->robotHeight, obj.range);
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image &msg)
{
    try {
        // 检查图像数据是否有效
        if (msg.data.empty() || msg.height == 0 || msg.width == 0) {
            RCLCPP_WARN(get_logger(), "Received empty depth image");
            return;
        }

        // 创建深度图像和转换
        cv::Mat depthFloat;
        // 根据图像编码格式进行处理
        if (msg.encoding == "16UC1" || msg.encoding == "mono16") {
            size_t expected = (size_t)msg.width * msg.height * sizeof(uint16_t);
            if (msg.data.size() < expected) {
                RCLCPP_ERROR(get_logger(), "Depth mono16 size mismatch");
                return;
            }
            cv::Mat depthRaw(msg.height, msg.width, CV_16UC1, const_cast<uint8_t*>(msg.data.data()));
            depthRaw.convertTo(depthFloat, CV_32FC1, 1.0 / 1000.0); // 若是实际深度单位 mm
        } else if (msg.encoding == "32FC1") {
            // 检查数据大小是否正确
            size_t expected_size = msg.height * msg.width * sizeof(float);
            if (msg.data.size() != expected_size) {
                RCLCPP_ERROR(get_logger(), "Depth image size mismatch: expected %zu, got %zu", 
                    expected_size, msg.data.size());
                return;
            }

            // 直接创建 32 位浮点数格式的深度图像
            depthFloat = cv::Mat(msg.height, msg.width, CV_32FC1, 
                const_cast<float*>(reinterpret_cast<const float*>(msg.data.data()))).clone();
            
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported depth image encoding: %s", msg.encoding.c_str());
            return;
        }

        vector<rerun::Vec3D> points_robot;  // for log

        const double fx = config->camfx;
        const double fy = config->camfy;
        const double cx = config->camcx;
        const double cy = config->camcy;
        // cout << "fx = " << fx << " fy = " << fy << " cx = " << cx << " cy = " << cy << endl;
        
        // 定义网格参数
        const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 网格大小
        const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
        const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
        const double y_max = -y_min;
        const int grid_x_count = static_cast<int>((x_max - x_min) / grid_size);
        const int grid_y_count = static_cast<int>((y_max - y_min) / grid_size);
        
        // 创建网格占用数组
        vector<vector<int>> grid_occupied(grid_x_count, vector<int>(grid_y_count, 0));
        
        const bool ballLocationKnown =
            tree->getEntry<bool>("ball_location_known") ||
            tree->getEntry<bool>("tm_ball_pos_reliable");

        // 处理深度图像点
        const int sampleStep = get_parameter("obstacle_avoidance.depth_sample_step").as_int();
        for (int y = 0; y < msg.height; y += sampleStep)
        {
            for (int x = 0; x < msg.width; x += sampleStep)
            {
                float depth = depthFloat.at<float>(y, x);
                if (depth > 0)
                {
                    // 转换到相机坐标系
                    double x_cam = (x - cx) * depth / fx;
                    double y_cam = (y - cy) * depth / fy;
                    double z_cam = depth;

                    // 转换到机器人坐标系
                    Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);
                    Eigen::Vector4d point_robot = data->camToRobot * point_cam;
                    
                    // 记录点用于可视化
                    points_robot.push_back(rerun::Vec3D{point_robot(0), point_robot(1), point_robot(2)});
                    
                    // 更新网格占用情况
                    const double Z_THRESHOLD = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
                    const double EXCLUDE_MAX_X = get_parameter("obstacle_avoidance.exclusion_x").as_double(); // 排除机器人自己的身体
                    const double EXCLUDE_MIN_X = -EXCLUDE_MAX_X;
                    const double EXCLUDE_MAX_Y = get_parameter("obstacle_avoidance.exclusion_y").as_double(); // 排除机器人自己的身体
                    const double EXCLUDE_MIN_Y = -EXCLUDE_MAX_Y;

                    auto isInRange = [&]() {
                        return point_robot(0) >= x_min && point_robot(0) < x_max
                            && point_robot(1) >= y_min && point_robot(1) < y_max;
                    };
                    auto isSelfBody = [&]() {
                        return point_robot(0) >= EXCLUDE_MIN_X && point_robot(0) <= EXCLUDE_MAX_X
                            && point_robot(1) >= EXCLUDE_MIN_Y && point_robot(1) <= EXCLUDE_MAX_Y;
                    };
                    auto isBall = [&]() {
                        if (!ballLocationKnown) return false;
                        double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
                        double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
                        return fabs(point_robot(0) - data->ball.posToRobot.x) < r 
                            && fabs(point_robot(1) - data->ball.posToRobot.y) < r
                            && point_robot(2) < h;
                    };

                    if (
                        point_robot(2) > Z_THRESHOLD 
                        && isInRange()
                        &&!isSelfBody() 
                        &&!isBall()
                    )
                    {
                        int grid_x = static_cast<int>((point_robot(0) - x_min) / grid_size);
                        int grid_y = static_cast<int>((point_robot(1) - y_min) / grid_size);
                        grid_occupied[grid_x][grid_y] += 1;
                    }
                }
            }
        }

        auto obs_old = data->getObstacles();
        vector<GameObject> obs_new = {};

        // 本次看到的记入 obstables
        for (int i = 0; i < grid_x_count; i++) {
            for (int j = 0; j < grid_y_count; j++) {
                if (grid_occupied[i][j] > 0) {
                    GameObject obj;
                    obj.label = "Obstacle";
                    obj.timePoint = get_clock()->now();
                    obj.posToRobot.x = x_min + (i + 0.5) * grid_size;
                    obj.posToRobot.y = y_min + (j + 0.5) * grid_size;
                    obj.confidence = grid_occupied[i][j];
                    updateFieldPos(obj);
                    obs_new.push_back(obj);
                }
            }
        }

        // 清理旧 obstacle
        for (int i = 0; i < obs_old.size(); i++) {
           // 先把当前视野范围内的旧 obstacle 清空, 注意角度只是粗略计算, 并通过 offset 适当扩大了一些范围.
            double visionLeft = data->headYaw + config->camAngleX / 2;
            double visionRight = data->headYaw - config->camAngleX / 2;
            auto obs = obs_old[i];
            const double offset = 0.20;
            double obsYawLeft = atan2(obs.posToRobot.y - offset, obs.posToRobot.x + offset);
            double obsYawRight = atan2(obs.posToRobot.y + offset, obs.posToRobot.x + offset);
            if (obsYawLeft < visionLeft && obsYawRight > visionRight) continue; 

            // 如果旧的 obs 与新的 obs 太过接近, 则认为旧的 obs 已经不存在, 防止边界情况下 obs 堆积
            bool found = false;
            for (int j = 0; j < obs_new.size(); j++) {
                auto obs_n = obs_new[j];
                double dist = norm(obs.posToRobot.x - obs_n.posToRobot.x, obs.posToRobot.y - obs_n.posToRobot.y);
                if (dist < 0.5 * grid_size) {
                    found = true;
                    break;
                }
            }
            if (found) continue;

            // else
            obs_new.push_back(obs);
        }

        
        data->setObstacles(obs_new); // note: 此处不清空超时的旧 obstacles, 而在 tick 中清理
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        logDepth(grid_x_count, grid_y_count, grid_occupied, points_robot);
        logObstacles();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image callback: %s", e.what());
    }
}

double Brain::distToObstacle(double angle) {
    auto obs = data->getObstacles();
    double minDist = 1e9;
    // double obstacleThreshold = static_cast<double>(config->obstacleThreshold);
    double obstacleThreshold = static_cast<double>(get_parameter("obstacle_avoidance.occupancy_threshold").as_int());

    double collisionThreshold = config->collisionThreshold;

    for (int i = 0; i < obs.size(); i++) {
        if (obs[i].confidence < obstacleThreshold) continue;

        auto o = obs[i];
        Line line = {
            0, 0,
            cos(angle) * 100, sin(angle) * 100
        };
        double perpDist = fabs(pointPerpDistToLine(Point2D{o.posToRobot.x, o.posToRobot.y}, line));
        if (perpDist < collisionThreshold) {
            double dist = innerProduct(vector<double>{o.posToRobot.x, o.posToRobot.y}, vector<double>{cos(angle), sin(angle)});
            if (dist > 0 && dist < minDist) {
                minDist = dist;
            }
        }
    }
    return minDist;
}

vector<double> Brain::findSafeDirections(double startAngle, double safeDist, double step) {
    double safeAngleLeft = startAngle;
    double safeAngleRight = startAngle;
    double leftFound = 0;
    double rightFound = 0;
    for (double angle = startAngle; angle < startAngle + M_PI; angle += step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleLeft = angle;
            leftFound = 1;
            break;
        }
    }
    for (double angle = startAngle; angle > startAngle - M_PI; angle -= step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleRight = angle;
            rightFound = 1;
            break;
        }
    }

    return vector<double>{leftFound, toPInPI(safeAngleLeft), rightFound, toPInPI(safeAngleRight)};
}

double Brain::calcAvoidDir(double startAngle, double safeDist) {
    auto res = findSafeDirections(startAngle, safeDist);
    bool leftFound = res[0] > 0.5;
    bool rightFound = res[2] > 0.5;
    double angleLeft = res[1];
    double angleRight = res[3]; 
    double determinedAngle = 0;
    if (leftFound && rightFound) {
        determinedAngle = fabs(angleLeft) < fabs(angleRight) ? angleLeft : angleRight;
    } else if (leftFound) {
        determinedAngle = angleLeft;
    } else if (rightFound) {
        determinedAngle = angleRight;
    } else {
        return 0;
    }
    return toPInPI(determinedAngle);
}

void Brain::updateLogFile() {
    if (config->rerunLogEnableFile && msecsSince(data->timeLastLogSave) > config->rerunLogMaxFileMins * 60000)
        log->updateLogFilePath();
}

void Brain::setupLowFrequencyDiagnostics()
{
    get_parameter("debug.low_frequency_status.enable", lowFrequencyStatusEnable_);
    get_parameter("debug.low_frequency_status.mode_enable", lowFrequencyStatusModeEnable_);
    get_parameter("debug.low_frequency_status.field_ascii_enable", lowFrequencyStatusFieldAsciiEnable_);
    get_parameter("debug.low_frequency_status.log_interval_ms", lowFrequencyStatusLogIntervalMs_);
    get_parameter("debug.low_frequency_status.mode_query_interval_ms", lowFrequencyStatusModeQueryIntervalMs_);
    get_parameter("debug.low_frequency_status.mode_query_timeout_ms", lowFrequencyStatusModeQueryTimeoutMs_);
    get_parameter("debug.low_frequency_status.field_ascii_interval_ms", lowFrequencyStatusFieldAsciiIntervalMs_);
    get_parameter("debug.low_frequency_status.field_ascii_char_aspect", lowFrequencyStatusFieldAsciiCharAspect_);
    get_parameter("debug.low_frequency_status.field_ascii_width", lowFrequencyStatusFieldAsciiWidth_);

    lowFrequencyStatusLogIntervalMs_ = std::max(500.0, lowFrequencyStatusLogIntervalMs_);
    lowFrequencyStatusModeQueryIntervalMs_ = std::max(500.0, lowFrequencyStatusModeQueryIntervalMs_);
    lowFrequencyStatusModeQueryTimeoutMs_ = std::max(100.0, lowFrequencyStatusModeQueryTimeoutMs_);
    lowFrequencyStatusFieldAsciiIntervalMs_ = std::max(1000.0, lowFrequencyStatusFieldAsciiIntervalMs_);
    lowFrequencyStatusFieldAsciiCharAspect_ = std::clamp(lowFrequencyStatusFieldAsciiCharAspect_, 1.0, 3.0);
    lowFrequencyStatusFieldAsciiWidth_ = std::clamp(lowFrequencyStatusFieldAsciiWidth_, 20, 90);

    if (!lowFrequencyStatusEnable_)
        return;

    if (lowFrequencyStatusModeEnable_) {
        auto locoApiQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        modeQueryPublisher_ = create_publisher<booster_msgs::msg::RpcReqMsg>(
            "LocoApiTopicReq", locoApiQos);
        locoApiResponseSubscription_ = create_subscription<booster_msgs::msg::RpcRespMsg>(
            "LocoApiTopicResp",
            locoApiQos,
            bind(&Brain::locoApiResponseCallback, this, _1));
    } else {
        std::lock_guard<std::mutex> lock(lowFrequencyStatusMutex_);
        sdkRobotModeError_ = "mode_query_disabled";
    }

    RCLCPP_INFO(
        get_logger(),
        "[low_frequency_status] enabled mode=%d mode_interval=%.0fms field_ascii=%d field_interval=%.0fms",
        lowFrequencyStatusModeEnable_ ? 1 : 0,
        lowFrequencyStatusModeQueryIntervalMs_,
        lowFrequencyStatusFieldAsciiEnable_ ? 1 : 0,
        lowFrequencyStatusFieldAsciiIntervalMs_);
}

void Brain::sendModeQueryIfNeeded(const rclcpp::Time &now)
{
    if (!lowFrequencyStatusModeEnable_ || modeQueryPublisher_ == nullptr)
        return;

    std::string uuid;
    {
        std::lock_guard<std::mutex> lock(lowFrequencyStatusMutex_);

        if (!pendingModeQueryUuid_.empty()) {
            const bool timeout =
                pendingModeQueryTime_.nanoseconds() > 0 &&
                pendingModeQueryTime_.get_clock_type() == now.get_clock_type() &&
                (now - pendingModeQueryTime_).nanoseconds() / 1e6 > lowFrequencyStatusModeQueryTimeoutMs_;
            if (!timeout)
                return;

            sdkRobotModeError_ = "query_timeout";
            pendingModeQueryUuid_.clear();
        }

        if (lastModeQueryTime_.nanoseconds() > 0 &&
            lastModeQueryTime_.get_clock_type() == now.get_clock_type() &&
            (now - lastModeQueryTime_).nanoseconds() / 1e6 < lowFrequencyStatusModeQueryIntervalMs_) {
            return;
        }

        uuid = gen_uuid();
        pendingModeQueryUuid_ = uuid;
        pendingModeQueryTime_ = now;
        lastModeQueryTime_ = now;
    }

    booster_msgs::msg::RpcReqMsg msg;
    msg.uuid = uuid;
    msg.header = "{\"api_id\":2017,\"expect_response\":true}";
    msg.body = "";
    modeQueryPublisher_->publish(msg);
}

void Brain::locoApiResponseCallback(const booster_msgs::msg::RpcRespMsg &msg)
{
    if (!lowFrequencyStatusEnable_ || !lowFrequencyStatusModeEnable_)
        return;

    std::lock_guard<std::mutex> lock(lowFrequencyStatusMutex_);
    if (pendingModeQueryUuid_.empty() || msg.uuid != pendingModeQueryUuid_)
        return;

    pendingModeQueryUuid_.clear();
    const auto now = get_clock()->now();

    int status = 0;
    if (!msg.header.empty() && !extractJsonIntField(msg.header, "status", &status)) {
        sdkRobotModeStatus_ = -1;
        sdkRobotModeError_ = "bad_response_header";
        return;
    }

    sdkRobotModeStatus_ = status;
    if (status != 0) {
        sdkRobotModeError_ = format("response_status_%d", status);
        return;
    }

    int mode = -1;
    if (!extractJsonIntField(msg.body, "mode", &mode)) {
        sdkRobotModeError_ = "bad_response_body";
        return;
    }

    sdkRobotMode_ = mode;
    sdkRobotModeTime_ = now;
    sdkRobotModeError_.clear();
}

void Brain::logLowFrequencyDiagnostics(const rclcpp::Time &now)
{
    const bool shouldLogStatus =
        lastLowFrequencyStatusLogTime_.nanoseconds() == 0 ||
        lastLowFrequencyStatusLogTime_.get_clock_type() != now.get_clock_type() ||
        (now - lastLowFrequencyStatusLogTime_).nanoseconds() / 1e6 >= lowFrequencyStatusLogIntervalMs_;
    const bool shouldLogField =
        lowFrequencyStatusFieldAsciiEnable_ &&
        (lastFieldAsciiLogTime_.nanoseconds() == 0 ||
         lastFieldAsciiLogTime_.get_clock_type() != now.get_clock_type() ||
         (now - lastFieldAsciiLogTime_).nanoseconds() / 1e6 >= lowFrequencyStatusFieldAsciiIntervalMs_);

    if (!shouldLogStatus && !shouldLogField)
        return;

    int mode = -1;
    int modeStatus = -1;
    double modeAgeMs = -1.0;
    bool modePending = false;
    std::string modeError;
    {
        std::lock_guard<std::mutex> lock(lowFrequencyStatusMutex_);
        mode = sdkRobotMode_;
        modeStatus = sdkRobotModeStatus_;
        modePending = !pendingModeQueryUuid_.empty();
        modeError = sdkRobotModeError_;
        if (sdkRobotModeTime_.nanoseconds() > 0 &&
            sdkRobotModeTime_.get_clock_type() == now.get_clock_type()) {
            modeAgeMs = (now - sdkRobotModeTime_).nanoseconds() / 1e6;
        }
    }

    const Pose2D robotPose = data->robotPoseToField;
    const Pose2D odomPose = data->robotPoseToOdom;
    const uint64_t odomUpdateCount = data->odomUpdateCount;
    double odomAgeMs = -1.0;
    if (data->timeLastOdom.nanoseconds() > 0 &&
        data->timeLastOdom.get_clock_type() == now.get_clock_type()) {
        odomAgeMs = (now - data->timeLastOdom).nanoseconds() / 1e6;
    }
    Point ballField{0.0, 0.0, 0.0};
    bool ballDetected = false;
    bool ballKnown = false;
    double ballAgeMs = -1.0;
    double ballRange = -1.0;
    double ballConfidence = 0.0;
    if (data->ballDetected)
        lowFrequencyStatusBallSeenOnce_ = true;
    if (lowFrequencyStatusBallSeenOnce_) {
        ballDetected = data->ballDetected;
        ballField = data->ball.posToField;
        ballRange = data->ball.range;
        ballConfidence = data->ball.confidence;
        ballKnown = std::isfinite(ballField.x) &&
            std::isfinite(ballField.y) &&
            data->ball.timePoint.nanoseconds() > 0;
        if (ballKnown && data->ball.timePoint.get_clock_type() == now.get_clock_type())
            ballAgeMs = (now - data->ball.timePoint).nanoseconds() / 1e6;
    }

    string gameState = "-----";
    string gameSubState = "-----";
    string decision = "-----";
    int controlState = -1;
    try {
        gameState = tree->getEntry<string>("gc_game_state");
        gameSubState = tree->getEntry<string>("gc_game_sub_state");
        decision = tree->getEntry<string>("decision");
        controlState = tree->getEntry<int>("control_state");
        if (gameState.empty()) gameState = "-----";
        if (gameSubState.empty()) gameSubState = "-----";
        if (decision.empty()) decision = "-----";
    } catch (const std::exception &e) {
        decision = string("blackboard_error:") + e.what();
    }

    if (shouldLogStatus) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "MODE: " << modeNameFromIndex(mode) << "(" << mode << ")"
           << " sdk_status=" << modeStatus
           << " cache_age_ms=" << std::setprecision(0) << modeAgeMs
           << " next_query_pending=" << (modePending ? 1 : 0)
           << " error=" << (modeError.empty() ? "none" : modeError) << "\n";
        ss << std::setprecision(2)
           << "robot_field_pose x=" << robotPose.x
           << " y=" << robotPose.y
           << " theta_deg=" << robotPose.theta * 180.0 / M_PI
           << " heading=" << headingChar(robotPose.theta) << "\n";
        ss << "robot_odom_pose x=" << odomPose.x
           << " y=" << odomPose.y
           << " theta=" << odomPose.theta
           << " age_ms=" << std::setprecision(0) << odomAgeMs
           << " update_count=" << odomUpdateCount
           << " recovery_hold=" << (fallLocalizationHoldActive_ ? 1 : 0) << "\n";
        ss << "ball_field_pos x=" << ballField.x
           << " y=" << ballField.y
           << " detected=" << (ballDetected ? 1 : 0)
           << " known=" << (ballKnown ? 1 : 0)
           << " age_ms=" << std::setprecision(0) << ballAgeMs
           << " range=" << std::setprecision(2) << ballRange
           << " confidence=" << ballConfidence << "\n";
        ss << "game_state=" << gameState
           << " sub_state=" << gameSubState
           << " control_state=" << controlState
           << " decision=" << decision;
        const string statusText = ss.str();

        RCLCPP_INFO(get_logger(), "[low_frequency_status]\n%s", statusText.c_str());
        log->setTimeNow();
        log->log("debug/low_frequency_status", rerun::TextLog(statusText));
        lastLowFrequencyStatusLogTime_ = now;
    }

    if (shouldLogField) {
        const string fieldText = makeLowFrequencyFieldAscii(robotPose, ballField, ballKnown);
        RCLCPP_INFO(get_logger(), "[field_ascii]\n%s", fieldText.c_str());
        log->setTimeNow();
        log->log("debug/field_ascii", rerun::TextLog(fieldText));
        lastFieldAsciiLogTime_ = now;
    }
}

std::string Brain::makeLowFrequencyFieldAscii(
    const Pose2D &robotPose,
    const Point &ballField,
    bool ballKnown) const
{
    const auto &fd = config->fieldDimensions;
    if (fd.length <= 0.0 || fd.width <= 0.0)
        return "field dimensions invalid";

    const int innerW = std::clamp(lowFrequencyStatusFieldAsciiWidth_, 20, 90);
    const double charAspect = std::clamp(lowFrequencyStatusFieldAsciiCharAspect_, 1.0, 3.0);
    const int innerH = std::clamp(
        static_cast<int>(std::round(innerW * fd.width / fd.length / charAspect)), 8, 60);
    const double halfL = fd.length * 0.5;
    const double halfW = fd.width * 0.5;
    std::vector<std::string> grid(innerH + 2, std::string(innerW + 2, ' '));

    for (int c = 0; c < innerW + 2; ++c) {
        grid.front()[c] = '-';
        grid.back()[c] = '-';
    }
    for (int r = 0; r < innerH + 2; ++r) {
        grid[r].front() = '|';
        grid[r].back() = '|';
    }
    grid.front().front() = '+';
    grid.front().back() = '+';
    grid.back().front() = '+';
    grid.back().back() = '+';

    auto colForX = [&](double x) {
        const double ratio = (x + halfL) / fd.length;
        return 1 + std::clamp(
            static_cast<int>(std::lround(ratio * (innerW - 1))), 0, innerW - 1);
    };
    auto rowForY = [&](double y) {
        const double ratio = (halfW - y) / fd.width;
        return 1 + std::clamp(
            static_cast<int>(std::lround(ratio * (innerH - 1))), 0, innerH - 1);
    };
    auto put = [&](int row, int col, char ch, bool overwrite = true) {
        if (row < 1 || row > innerH || col < 1 || col > innerW)
            return;
        if (overwrite || grid[row][col] == ' ')
            grid[row][col] = ch;
    };
    auto putField = [&](double x, double y, char ch, bool overwrite = true) {
        if (!std::isfinite(x) || !std::isfinite(y))
            return;
        put(rowForY(y), colForX(x), ch, overwrite);
    };
    auto drawVertical = [&](double x, double y0, double y1, char ch) {
        const int c = colForX(x);
        int r0 = rowForY(y0);
        int r1 = rowForY(y1);
        if (r0 > r1) std::swap(r0, r1);
        for (int r = r0; r <= r1; ++r)
            put(r, c, ch, false);
    };
    auto drawHorizontal = [&](double y, double x0, double x1, char ch) {
        const int r = rowForY(y);
        int c0 = colForX(x0);
        int c1 = colForX(x1);
        if (c0 > c1) std::swap(c0, c1);
        for (int c = c0; c <= c1; ++c)
            put(r, c, ch, false);
    };
    auto drawBox = [&](double x0, double x1, double y0, double y1, char ch) {
        drawVertical(x0, y0, y1, ch);
        drawVertical(x1, y0, y1, ch);
        drawHorizontal(y0, x0, x1, ch);
        drawHorizontal(y1, x0, x1, ch);
    };

    drawVertical(0.0, -halfW, halfW, ':');
    drawBox(halfL - fd.penaltyAreaLength, halfL,
            -fd.penaltyAreaWidth * 0.5, fd.penaltyAreaWidth * 0.5, '.');
    drawBox(-halfL, -halfL + fd.penaltyAreaLength,
            -fd.penaltyAreaWidth * 0.5, fd.penaltyAreaWidth * 0.5, '.');
    drawBox(halfL - fd.goalAreaLength, halfL,
            -fd.goalWidth * 0.5, fd.goalWidth * 0.5, ',');
    drawBox(-halfL, -halfL + fd.goalAreaLength,
            -fd.goalWidth * 0.5, fd.goalWidth * 0.5, ',');

    putField(0.0, 0.0, 'o');
    putField(fd.length * 0.5 - fd.penaltyDist, 0.0, '+');
    putField(-fd.length * 0.5 + fd.penaltyDist, 0.0, '+');
    if (ballKnown)
        putField(ballField.x, ballField.y, 'B');

    if (std::isfinite(robotPose.x) && std::isfinite(robotPose.y) &&
        std::isfinite(robotPose.theta)) {
        const int robotRow = rowForY(robotPose.y);
        const int robotCol = colForX(robotPose.x);
        const double hx = std::cos(robotPose.theta);
        const double hy = std::sin(robotPose.theta);
        int dr = 0;
        int dc = 0;
        if (std::abs(hx) >= std::abs(hy))
            dc = hx >= 0.0 ? 1 : -1;
        else
            dr = hy >= 0.0 ? -1 : 1;

        const int pdr = -dc;
        const int pdc = dr;
        put(robotRow, robotCol, 'T');
        put(robotRow + dr, robotCol + dc, 'T');
        put(robotRow + 2 * dr, robotCol + 2 * dc, 'T');
        put(robotRow + pdr, robotCol + pdc, 'T');
        put(robotRow - pdr, robotCol - pdc, 'T');
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Field " << fd.length << "m x " << fd.width << "m, grid "
       << innerW << "x" << innerH << " char_aspect=" << charAspect
       << " (x+ right/opponent goal, y+ up/left touchline)\n";
    ss << "Legend: T robot body, T stem points heading, B ball, : midfield, . penalty area, o center mark\n";
    for (const auto &line : grid)
        ss << line << "\n";
    return ss.str();
}

std::string Brain::modeNameFromIndex(int mode) const
{
    switch (mode) {
    case -1: return "kUnknown";
    case 0: return "kDamping";
    case 1: return "kPrepare";
    case 2: return "kWalking";
    case 3: return "kCustom";
    case 4: return "kSoccer";
    default: return "kInvalid";
    }
}

char Brain::headingChar(double theta) const
{
    const double x = std::cos(theta);
    const double y = std::sin(theta);
    if (std::abs(x) >= std::abs(y))
        return x >= 0.0 ? '>' : '<';
    return y >= 0.0 ? '^' : 'v';
}

// ------------------------------------------------------ 调试 log 相关 ------------------------------------------------------
void Brain::logObstacleDistance() {
    log->setTimeNow();

    // log obstacle distance for test
    vector<rerun::LineStrip2D> lines = {};   
    for (int i = 0; i < 180; i++) {
        double angle = i * M_PI / 90;
        double dist = min(5.0, distToObstacle(angle));
        double angle_f = toPInPI(data->robotPoseToField.theta + angle);
        lines.push_back(
            rerun::LineStrip2D({
                {data->robotPoseToField.x, -data->robotPoseToField.y}, 
                {data->robotPoseToField.x + cos(-angle_f) * dist, -data->robotPoseToField.y + sin(-angle_f) * dist}
            })
        );
    }
    log->log(
        "field/obstacle_distance",
        rerun::LineStrips2D(lines)
            .with_colors(0x666666FF)
            .with_radii(0.01)
            .with_draw_order(-10)
    );
}

void Brain::logLags() {
    log->setTimeNow();
    auto color = 0x00FF00FF;
    
    double detLag = msecsSince(data->timeLastDet);
    if (detLag > 500) color = 0xFF0000FF;
    else if (detLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;
    double MAX_LAG_LENGTH = config->camPixX;
    log->log(
        "image/detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -150.}, {10. + min(detLag, MAX_LAG_LENGTH), -150.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Det Lag %.0fms", detLag)})
    );
    log->log(
        "performance/detection_lag_timeseries",
        rerun::Scalar(detLag)
    );

    // log fieldline detection delay
    double lineLag = msecsSince(data->timeLastLineDet);
    if (lineLag > 500) color = 0xFF0000FF;
    else if (lineLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;

    log->log(
        "image/fieldline_detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -100.}, {10. + min(lineLag, MAX_LAG_LENGTH), -100.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Line Det Lag %.0fms", lineLag)})
    );
    log->log(
        "performance/fieldline_detection_lag_timeseries",
        rerun::Scalar(lineLag)
    );

    // log game control delay
    double gcLag = msecsSince(data->timeLastGamecontrolMsg);
    if (gcLag > 5000) color = 0xFF0000FF;
    else if (gcLag > 1000) color = 0xFFFF00FF;
    else color = 0x00FF00FF;

    log->log(
        "image/gamecontrol_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -50.}, {10. + min(gcLag, MAX_LAG_LENGTH), -50.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("GC Lag %.0fms", gcLag)})
    );
    log->log(
        "performance/gamecontrol_lag_timeseries",
        rerun::Scalar(gcLag)
    );
}

void Brain::statusReport() {
    if (!config->soundEnable || config->soundPack != "espeak") return;

    log->setTimeNow();
    static int reportInterval = 100;
    static string lastReport = "";
    string report;
    bool camOK = msecsSince(data->timeLastDet) < 1000;
    bool gcOK = msecsSince(data->timeLastGamecontrolMsg) < 1000;

    if (camOK && gcOK) {
        report = "Team" + to_string(config->teamId) + " Player " + to_string(config->playerId) + " " + tree->getEntry<string>("player_role") + " " + " OK";
    } else {
        report = "";
        if (!camOK) report += "camera lost";
        if (!gcOK) report += "gamecontrol lost";
    }
    if (lastReport != report) {
        speak(report);
        lastReport = report;
    }
}

void Brain::logStatusToConsole() {
    static int cnt = 0;
    const int LOG_INTERVAL = 30;
    cnt++;
    if (cnt % LOG_INTERVAL == 0) {
        string msg = "";
        string gameState = tree->getEntry<string>("gc_game_state");
        gameState = gameState == "" ? "-----" : gameState;
        string gameSubType = tree->getEntry<string>("gc_game_sub_state_type");
        gameSubType = gameSubType == "" ? "-----" : gameSubType;
        string gameSubState = tree->getEntry<string>("gc_game_sub_state");
        gameSubState = gameSubState == "" ? "-----" : gameSubState;

        msg += format(
            "ROBOT:\n\tTeamID: %d\tPlayerID: %d\tNumberOfPlayers: %d\tRole: %s\tStartRole: %s\n\n",
            config->teamId,
            config->playerId,
            config->numOfPlayers,
            tree->getEntry<string>("player_role").c_str(),
            config->playerRole.c_str()
        );
        msg += format(
            "GAME:\n\tState: %s\tKickOffSide: %s\tisKickingOff: %s(%s)\n\tSubType: %s\tSubState: %s\tSubKickOffSide: %s\tisKickingOff: %s(%s)\n\tScore: %s\tJustScored: %s\n\tLiveCount: %d\tOppoLiveCount: %d\tPrimary: %s\n\n", 
            gameState.c_str(), 
            tree->getEntry<bool>("gc_is_kickoff_side") ? "YES" : "NO",
            data->isKickingOff ? "YES" : "NO",
            msecsSince(data->kickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->kickoffStartTime)/1000).c_str(),
            gameSubType.c_str(),
            gameSubState.c_str(),
            tree->getEntry<bool>("gc_is_sub_state_kickoff_side") ? "YES" : "NO",
            data->isFreekickKickingOff ? "YES" : "NO",
            msecsSince(data->freekickKickoffStartTime)/1000 > 100 ? "--" : to_string(msecsSince(data->freekickKickoffStartTime)/1000).c_str(),
            format("%d:%d", data->score, data->oppoScore).c_str(),
            tree->getEntry<bool>("we_just_scored") ? "YES" : "NO",
            data->liveCount,
            data->oppoLiveCount,
            isPrimaryStriker() ? "YES" : "NO"
        );
        
        msg += getComLogString();

        msg += format(
            "DEBUG:\n\tcom: %s\tlogFile: %s\tlogTCP: %s\n\tvxFactor: %.2f\tyawOffset: %.2f\n\tControlState: %d\n\tTickTime: %.0fms",
            config->enableCom ? "YES" : "NO",
            config->rerunLogEnableFile ? "YES" : "NO",
            config->rerunLogEnableTCP ? "YES" : "NO",
            config->vxFactor,
            config->yawOffset,
            tree->getEntry<int>("control_state"),
            msecsSince(data->lastTick)
        );
        prtDebug(msg);
    }
    data->lastTick = get_clock()->now();
}

string Brain::getComLogString() {
    stringstream ss;
    int onFieldCnt = 0;
    int aliveCnt = 0;
    int selfIdx = config->playerId - 1;
    vector<int> onFieldIdxs = {};
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (data->penalty[i] == PENALTY_NONE) {
            onFieldCnt += 1;
            onFieldIdxs.push_back(i);
        }

        if (data->tmStatus[i].isAlive) aliveCnt += 1;
    }
    ss << CYAN_CODE << "COM: " << "\n";
    ss << "Teammates: OnField: " << onFieldCnt << "[";
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        ss << " P" << idx + 1 << " ";
    }
    ss << "]";
    ss << "  Alive: " << aliveCnt << "  TMCMDID: " << data->tmCmdId << "  ReceivedDMD: " << data->tmReceivedCmd << "\n";

    // Self info
    ss << "Self\tCost: " << format("%.1f", data->tmMyCost)
       << "\tCostRank: " << data->tmMyCostRank
       << "\tStrikerCostRank: " << data->tmMyStrikerCostRank
       << "\tLead: ";
    if (data->tmImLead)
        ss << GREEN_CODE << "YES" << CYAN_CODE;
    else
        ss << RED_CODE << "NO" << CYAN_CODE;
    ss << "    TMCMD: " << data->tmCmdId << format("\tCMD: [%d]%d", data->tmMyCmdId, data->tmMyCmd);
    ss << "\n";

    // Teammates info
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        auto status = data->tmStatus[idx];
        ss << "P" << idx + 1 << "[";
        if (status.isAlive)
            ss << GREEN_CODE << "★" << CYAN_CODE;
        else 
            ss << RED_CODE << "☆" << CYAN_CODE;
        ss << "]\tCost: " << format("%.1f", status.cost);
        ss << "\tLead: ";
        if (status.isLead)
            ss << GREEN_CODE << "YES" << CYAN_CODE;
        else
            ss << RED_CODE << "NO" << CYAN_CODE;
        ss << "\tCMD: " << format("[%d]%d", status.cmdId, status.cmd);
        ss << "\tLag: " << format("%.0f", msecsSince(status.timeLastCom)) << "ms" << "\n";
    }
    ss << "\n";
    
    return ss.str();
}

bool Brain::isFreekickStartPlacing() {
    return (tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK" && tree->getEntry<string>("gc_game_state") == "PLAY" && tree->getEntry<string>("gc_game_sub_state") == "GET_READY");
}

void Brain::playSoundForFun() {
    string soundPack = config->soundPack;
    if (config->soundEnable && soundPack != "espeak") {
        static string gcGameState_last;
        string gcGameState = tree->getEntry<string>("gc_game_state");

        static bool gameStarted = false;
        if (gcGameState == "PLAY") gameStarted = true;
        if (gcGameState == "READY")
        {
            if (!gameStarted) playSound(soundPack + "-ready", 2000);
            else if (tree->getEntry<bool>("we_just_scored")) playSound(soundPack + "-celebrate", 2000);
            else playSound(soundPack + "-regret", 2000);   
        }

        if (gcGameState == "PLAY") {
            auto decision = tree->getEntry<string>("decision"); 
            if (decision == "chase") playSound(soundPack + "-chase", 5000);
            else if (decision == "adjust") playSound(soundPack + "-adjust", 2000);
            else if (decision == "kick") playSound(soundPack + "-kick", 2000);
        }
    }
}

// =============================================================================
//  请将以下代码追加到 src/brain/src/brain.cpp 的最末尾 (在最后的 } 之后或者之前均可，确保在 namespace 外或类定义外)
// =============================================================================

/**
 * 计算当前向 dir 方向踢球的价值的大小.
 */
double Brain::kickValue(double dir)
{
    // 简单的价值评估：如果是朝向对方球门方向，价值高
    // 这里是一个基础实现，你可以根据策略需要修改
    auto fd = config->fieldDimensions;
    double goalDir = atan2(0.0 - data->robotPoseToField.y, fd.length/2.0 - data->robotPoseToField.x);
    double diff = fabs(toPInPI(dir - goalDir));
    
    // 角度偏差越小，价值越高。范围 [0, 1]
    return max(0.0, 1.0 - diff / M_PI); 
}

/**
 * 计算当前的急迫度
 * 0: 安全, 1: 有威胁, 2: 危险
 */
double Brain::threatLevel()
{
    // 基础实现：如果有敌人在附近(2米内)，则认为有威胁
    double minOpponentDist = 100.0;
    auto robots = data->getRobots();
    for(const auto& r : robots) {
        if(r.label == "Opponent") {
            double dist = r.range;
            if(dist < minOpponentDist) minOpponentDist = dist;
        }
    }

    if (minOpponentDist < 1.0) return 2.0;
    if (minOpponentDist < 2.5) return 1.0;
    return 0.0;
}

/**
 * 判断是否适合定向踢球
 */
bool Brain::isAngleGoodForDirectionalKick(double goalPostMargin)
{
    // 复用通用的角度判断逻辑，或者添加特定逻辑
    return isAngleGood(goalPostMargin, "kick");
}

/**
 * 检查前方扇形区域是否有障碍物
 */
bool Brain::isFrontRangeClear(double startAngle, double endAngle, double safeDist, double step)
{
    // 遍历角度范围，检查每个角度上的最近障碍物距离
    for (double ang = startAngle; ang <= endAngle; ang += step) {
        if (distToObstacle(ang) < safeDist) {
            return false; // 发现障碍物，不空闲
        }
    }
    return true; // 区域内无障碍物
}

/**
 * 发布视觉校准参数
 */
/**
 * 发布视觉校准参数
 */
void Brain::pubCalParamMsg(double pitch, double yaw, double z)
{
    if (pubCalParam) {
        vision_interface::msg::CalParam msg;
        msg.pitch_compensation = pitch;
        msg.yaw_compensation = yaw;
        msg.z_compensation = z;
        
        pubCalParam->publish(msg);
    }
}
