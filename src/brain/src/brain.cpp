#include <iostream>
#include <string>

#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"
#include "joy_msg.h"

using namespace std;
using std::placeholders::_1;

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Note that the parameters must be declared here first, otherwise they cannot be read in the program either.
    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "");

    declare_parameter<string>("robot.robot_name", "");
    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.95);
    declare_parameter<double>("robot.yaw_offset", 0.1);
    declare_parameter<string>("robot.joystick", "");

    declare_parameter<bool>("rerunLog.enable", false);
    declare_parameter<string>("rerunLog.server_addr", "");
    declare_parameter<int>("rerunLog.img_interval", 10);

    // The tree_file_path is configured in launch.py and not placed in config.yaml.
    declare_parameter<string>("tree_file_path", "");
}

void Brain::init()
{
    // Make sure to load the configuration first, and then the config can be used.
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    locator->init(config->fieldDimensions, 4, 0.5);

    tree->init();

    client->init(config->robotName);

    log->prepare();

    data->lastSuccessfulLocalizeTime = get_clock()->now();

    joySubscription = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&Brain::joystickCallback, this, _1));
    gameControlSubscription = create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 1, bind(&Brain::gameControlCallback, this, _1));
    
    string topic_suffix = "";
    if(config->robotName != "") {
        RCLCPP_INFO(this->get_logger(), "Robot name set to: %s", config->robotName.c_str());
        topic_suffix = "/" + config->robotName;
    }
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection" + topic_suffix, 1, bind(&Brain::detectionsCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state" + topic_suffix, 1, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state" + topic_suffix, 1, bind(&Brain::lowStateCallback, this, _1));
    imageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw" + topic_suffix, 1, bind(&Brain::imageCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose" + topic_suffix, 1, bind(&Brain::headPoseCallback, this, _1));
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);

    get_parameter("robot.robot_name", config->robotName);
    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.joystick", config->joystick);

    get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.server_addr", config->rerunLogServerAddr);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("tree_file_path", config->treeFilePath);

    // handle the parameters
    config->handle();

    // debug after handle the parameters
    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}

/**
 * will be called in the Ros2 loop
 */
void Brain::tick()
{
    updateMemory();
    tree->tick();
}

void Brain::updateMemory()
{
    updateBallMemory();

    static Point ballPos;
    static rclcpp::Time kickOffTime;
    if (
        tree->getEntry<string>("player_role") == "striker" && ((tree->getEntry<string>("gc_game_state") == "SET" && !tree->getEntry<bool>("gc_is_kickoff_side")) || (tree->getEntry<string>("gc_game_sub_state") == "SET" && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side"))))
    {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    }
    else if (tree->getEntry<bool>("wait_for_opponent_kickoff"))
    {
        if (
            norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y) > 0.3 || (get_clock()->now() - kickOffTime).seconds() > 10.0)
        {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

void Brain::updateBallMemory()
{
    // update Pose to field from Pose to robot (based on odom)
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(data->robotPoseToField.theta) * data->robotPoseToField.x - cos(data->robotPoseToField.theta) * data->robotPoseToField.y;
    xfr = -cos(data->robotPoseToField.theta) * data->robotPoseToField.x - sin(data->robotPoseToField.theta) * data->robotPoseToField.y;
    thetafr = -data->robotPoseToField.theta;
    transCoord(
        data->ball.posToField.x, data->ball.posToField.y, 0,
        xfr, yfr, thetafr,
        data->ball.posToRobot.x, data->ball.posToRobot.y, data->ball.posToRobot.z);

    data->ball.range = sqrt(data->ball.posToRobot.x * data->ball.posToRobot.x + data->ball.posToRobot.y * data->ball.posToRobot.y);
    tree->setEntry<double>("ball_range", data->ball.range);
    data->ball.yawToRobot = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
    data->ball.pitchToRobot = asin(config->robotHeight / data->ball.range);

    // mark ball as lost if long time no see
    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    {
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
    }

    // log mem ball pos
    log->setTimeNow();
    log->log("field/memball",
             rerun::LineStrips2D({
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x - 0.2, -data->ball.posToField.y}, {data->ball.posToField.x + 0.2, -data->ball.posToField.y}},
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x, -data->ball.posToField.y - 0.2}, {data->ball.posToField.x, -data->ball.posToField.y + 0.2}},
                                 })
                 .with_colors({tree->getEntry<bool>("ball_location_known") ? 0xFFFFFFFF : 0xFF0000FF})
                 .with_radii({0.005})
                 .with_draw_order(30));
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY;
    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    for (int i = 0; i < data->goalposts.size(); i++)
    {
        auto post = data->goalposts[i];
        if (post.info == "oppo-left")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.info == "oppo-right")
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

void Brain::calibrateOdom(double x, double y, double theta)
{
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;

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
        data->ball.posToField.x, data->ball.posToField.y, placeHolder);

    // opponents
    for (int i = 0; i < data->opponents.size(); i++)
    {
        auto obj = data->opponents[i];
        transCoord(
            obj.posToRobot.x, obj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            obj.posToField.x, obj.posToField.y, placeHolder);
    }
}

double Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}

void Brain::joystickCallback(const sensor_msgs::msg::Joy &msg)
{
    JoyMsg joy(msg);

    if (!joy.BTN_LT && !joy.BTN_RT && !joy.BTN_LB && !joy.BTN_RB)
    {
        if (joy.BTN_B)
        {
            tree->setEntry<bool>("B_pressed", true);
            prtDebug("B is pressed");
        }
        else if (!joy.BTN_B && tree->getEntry<bool>("B_pressed"))
        {
            tree->setEntry<bool>("B_pressed", false);
            prtDebug("B is released");
        }
    }
    else if (joy.BTN_LT && !joy.BTN_RT && !joy.BTN_LB && !joy.BTN_RB)
    {
        if (joy.AX_DX || joy.AX_DY)
        {
            config->vxFactor += 0.01 * joy.AX_DX;
            config->yawOffset += 0.01 * joy.AX_DY;
            prtDebug(format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset));
        }

        if (joy.BTN_X)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        else if (joy.BTN_A)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        else if (joy.BTN_B)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (joy.BTN_Y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    auto lastGameState = tree->getEntry<string>("gc_game_state");
    vector<string> gameStateMap = {
        "INITIAL", // Initialization state, players are ready outside the field.
        "READY",   // Ready state, players enter the field and walk to their starting positions.
        "SET",     // Stop action, waiting for the referee machine to issue the instruction to start the game.
        "PLAY",    // Normal game.
        "END"      // The game is over.
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId);
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    string gameSubStateType = static_cast<int>(msg.secondary_state) == 0 ? "NONE" : "FREE_KICK";
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId);
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    game_controller_interface::msg::TeamInfo myTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
    }
    else
    {

        prtErr("received invalid game controller message");
        return;
    }

    data->penalty[0] = static_cast<int>(myTeamInfo.players[0].penalty);
    data->penalty[1] = static_cast<int>(myTeamInfo.players[1].penalty);
    data->penalty[2] = static_cast<int>(myTeamInfo.players[2].penalty);
    data->penalty[3] = static_cast<int>(myTeamInfo.players[3].penalty);
    double isUnderPenalty = (data->penalty[config->playerId] != 0);
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);

    int curScore = static_cast<int>(myTeamInfo.score);
    if (curScore > data->lastScore)
    {
        tree->setEntry<bool>("we_just_scored", true);
        data->lastScore = curScore;
    }
    if (gameState == "SET")
    {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    auto gameObjects = getGameObjects(msg);

    vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            if (tree->getEntry<bool>("treat_person_as_robot"))
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    detectProcessBalls(balls);
    detectProcessMarkings(markings);

    if (!log->isEnabled())
        return;

    // log detection boxes to rerun
    auto detection_time_stamp = msg.header.stamp;
    rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto now = get_clock()->now();

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(format("%s x:%.2f y:%.2f c:%.2f", obj.label.c_str(), obj.posToRobot.x, obj.posToRobot.y, obj.confidence)));
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y});
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            colors.push_back(detectColorMap[label]);
        }
        else
        {
            colors.push_back(rerun::Color(0xFFFFFFFF));
        }
    }

    double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    log->setTimeSeconds(time);
    log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
             // .with_labels(labels)
    );
    log->log("robotframe/detection_points",
             rerun::Points2D(points_r)
                 .with_colors(colors)
             // .with_labels(labels)
    );
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    log->setTimeNow();
    log->log("field/robot",
             rerun::Points2D({{data->robotPoseToField.x, -data->robotPoseToField.y}, {data->robotPoseToField.x + 0.1 * cos(data->robotPoseToField.theta), -data->robotPoseToField.y - 0.1 * sin(data->robotPoseToField.theta)}})
                 .with_radii({0.2, 0.1})
                 .with_colors({0xFF6666FF, 0xFF0000FF}));
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;

    log->setTimeNow();

    log->log("low_state_callback/imu/rpy/roll", rerun::Scalar(msg.imu_state.rpy[0]));
    log->log("low_state_callback/imu/rpy/pitch", rerun::Scalar(msg.imu_state.rpy[1]));
    log->log("low_state_callback/imu/rpy/yaw", rerun::Scalar(msg.imu_state.rpy[2]));
    log->log("low_state_callback/imu/acc/x", rerun::Scalar(msg.imu_state.acc[0]));
    log->log("low_state_callback/imu/acc/y", rerun::Scalar(msg.imu_state.acc[1]));
    log->log("low_state_callback/imu/acc/z", rerun::Scalar(msg.imu_state.acc[2]));
    log->log("low_state_callback/imu/gyro/x", rerun::Scalar(msg.imu_state.gyro[0]));
    log->log("low_state_callback/imu/gyro/y", rerun::Scalar(msg.imu_state.gyro[1]));
    log->log("low_state_callback/imu/gyro/z", rerun::Scalar(msg.imu_state.gyro[2]));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{
    if (!config->rerunLogEnable)
        return;

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {

        cv::Mat imageBGR(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        cv::Mat imageRGB;
        cv::cvtColor(imageBGR, imageRGB, cv::COLOR_BGR2RGB);

        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10};
        cv::imencode(".jpg", imageRGB, compressed_image, compression_params);

        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        log->setTimeSeconds(time);
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose &msg)
{

    // --- for test:
    // if (config->rerunLogEnable) {
    if (false)
    {
        auto x = msg.position.x;
        auto y = msg.position.y;
        auto z = msg.position.z;

        auto orientation = msg.orientation;

        auto roll = rad2deg(atan2(2 * (orientation.w * orientation.x + orientation.y * orientation.z), 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)));
        auto pitch = rad2deg(asin(2 * (orientation.w * orientation.y - orientation.z * orientation.x)));
        auto yaw = rad2deg(atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)));

        log->setTimeNow();

        log->log("head_to_base/text",
                 rerun::TextLog("x: " + to_string(x) + " y: " + to_string(y) + " z: " + to_string(z) + " roll: " + to_string(roll) + " pitch: " + to_string(pitch) + " yaw: " + to_string(yaw)));
        log->log("head_to_base/x",
                 rerun::Scalar(x));
        log->log("head_to_base/y",
                 rerun::Scalar(y));
        log->log("head_to_base/z",
                 rerun::Scalar(z));
        log->log("head_to_base/roll",
                 rerun::Scalar(roll));
        log->log("head_to_base/pitch",
                 rerun::Scalar(pitch));
        log->log("head_to_base/yaw",
                 rerun::Scalar(yaw));
    }
}

vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    auto timestamp = detections.header.stamp;

    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        {
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
        }
        else
        {
            gObj.posToRobot.x = obj.position_projection[0];
            gObj.posToRobot.y = obj.position_projection[1];
        }

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range);

        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // Parameters
    const double confidenceValve = 0.35;        // If the confidence is lower than this threshold, it is considered not a ball (note that the target confidence passed in by the detection module is currently all > 0.2).
    const double pitchLimit = deg2rad(0);       // When the pitch of the ball relative to the front of the robot (downward is positive) is lower than this value, it is considered not a ball. (Because the ball won't be in the sky.)
    const int timeCountThreshold = 5;           // Only when the ball is detected in consecutive several frames is it considered a ball. This is only used in the ball-finding strategy.
    const unsigned int detectCntThreshold = 3;  // The maximum count. Only when the target is detected in such a number of frames is it considered that the target is truly identified. (Currently only used for ball detection.)
    const unsigned int diffConfidThreshold = 4; // The threshold for the difference times between the tracked ball and the high-confidence ball. After reaching this threshold, the high-confidence ball will be adopted.

    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // Which ball is considered to be the real one. -1 indicates that no ball has been detected.
    int indexTraceBall = -1; // Track the ball according to the pixel distance. -1 indicates that no target has been tracked.

    // Find the most likely real ball.
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        // Judgment: If the confidence is too low, it is considered a false detection.
        if (ballObj.confidence < confidenceValve)
            continue;

        // Prevent the lights in the sky from being recognized as balls.
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        // Find the one with the highest confidence among the remaining balls.
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    {
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];

        tree->setEntry<bool>("ball_location_known", true);
    }
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }

    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = 0.1;

    data->markings.clear();

    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        if (marking.confidence < confidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;

        data->markings.push_back(marking);
    }
}