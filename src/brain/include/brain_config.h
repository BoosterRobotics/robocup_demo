#pragma once

#include <string>
#include <ostream>

#include "types.h"
#include "utils/math.h"

using namespace std;

/**
 * Stores configuration values required by the Brain. These values should be confirmed during initialization
 * and remain read-only during the robot's decision-making process.
 * Values that need to change during the decision process should be placed in BrainData.
 *
 * Note:
 * 1. The configuration file will be read from config/config.yaml.
 * 2. If config/config_local.yaml exists, its values will override those in config/config.yaml.
 */

class BrainConfig
{
public:
    // ---------- start config from config.yaml ---------------------------------------------
    // These variables are the raw values directly read from the configuration file.
    // If new configurations are added to the configuration file, corresponding variables should be added here to store them.
    // These values will be overwritten in BrainNode, so even if a configuration is not explicitly defined in config.yaml,
    // the default values here will not take effect.
    // The actual default values should be configured in the BrainNode's declare_parameter.
    int teamId;            // Corresponds to game.team_id
    int playerId;          // Corresponds to game.player_id
    string fieldType;      // Corresponds to game.field_type  "adult_size"(14*9) | "kid_size" (9*6)
    string playerRole;     // Corresponds to game.player_role   "striker" | "goal_keeper"
    string playerStartPos; // Corresponds to game.player_start_post  "left" | "right"

    string robotName;      // Corresponds to robot.robot_name  Name of the robot to use in multi-robot setup
    double robotHeight;     // Corresponds to robot.robot_height
    double robotOdomFactor; // Corresponds to robot.odom_factor odom
    double vxFactor;        // Corresponds to robot.vx_factor fix the issue where the actual vx is larger than the command
    double yawOffset;       // Corresponds to robot.yaw_offset fix the issue of leftward bias during distance measurement
    string joystick;        // Corresponds to robot.joystick "logicall" | "beitong"

    bool rerunLogEnable;       // Corresponds to rerunLog.enable  Whether to enable rerunLog
    string rerunLogServerAddr; // Corresponds to rerunLog.server_addr  rerunLog address
    int rerunLogImgInterval;   // Corresponds to rerunLog.img_interval the interval to record the images

    string treeFilePath; //  It is no longer placed in config.yaml; the path to the behavior-tree file is now specified in launch.py.
    // ----------  end config from config.yaml ---------------------------------------------

    // game parameters
    FieldDimensions fieldDimensions;

    // Camera resolution
    double camPixX = 1280;
    double camPixY = 720;

    // Camera angle
    double camAngleX = deg2rad(90);
    double camAngleY = deg2rad(65);

    // Head rotation soft limit
    double headYawLimitLeft = 1.1;
    double headYawLimitRight = -1.1;
    double headPitchLimitUp = 0.0;

    // Speed limit
    double vxLimit = 1.2;
    double vyLimit = 0.4;
    double vthetaLimit = 1.5;

    // Strategy parameters
    double safeDist = 2.0; // Safety distance for collision detection. If the distance is smaller than this value, a collision is considered.
    double goalPostMargin = 0.4;
    // Calculate the margin of the goalpost, used to compute the angle of the goalpost. The larger the margin, the smaller the goal appears during calculations.
    // During a touch event, this margin is different and is typically smaller.
    double goalPostMarginForTouch = 0.1;
    double memoryLength = 3.0; // The number of seconds during which the ball is not visible, after which it is considered lost.

    // After Brain fills in the parameters, it calls handle() to process the parameters (such as calibration, calculations, etc.),
    void handle();

    // Output the configuration information to the specified output stream (for debugging).
    void print(ostream &os);
};