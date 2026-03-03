#include "brain_config.h"
#include "utils/print.h"
#include <rclcpp/rclcpp.hpp>


// simple replace_all helper used by get_depth_image_topic
static inline string replace_all(const string &s, const string &from, const string &to) {
    if (from.empty()) return s;
    string result = s;
    size_t pos = 0;
    while ((pos = result.find(from, pos)) != string::npos) {
        result.replace(pos, from.length(), to);
        pos += to.length();
    }
    return result;
}


// Forward declaration of Brain class to support static_cast without including brain.h (which includes OpenCV headers that cause compilation issues)
class Brain : public rclcpp::Node {
};

BrainConfig::BrainConfig(Brain *argBrain) : brain(argBrain)
{

}

string BrainConfig::get_tree_file_path() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter("tree_file_path").as_string();
}

string BrainConfig::get_game_control_ip() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game_control_ip", string("127.0.0.1"));
}

int BrainConfig::get_player_id() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.player_id", 1);
}

int BrainConfig::get_team_id() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.team_id", 1);
}

string BrainConfig::get_player_role() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.player_role", string("striker"));
}

string BrainConfig::get_field_type() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.field_type", string("adult_size"));
}

int BrainConfig::get_num_of_players() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.number_of_players", 2);
}

bool BrainConfig::get_treat_person_as_robot() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("game.treat_person_as_robot", false);
}

string BrainConfig::get_robot_name() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.robot_name", string(""));
}

double BrainConfig::get_robot_height() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.robot_height", 0.9);
}

double BrainConfig::get_robot_odom_factor() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.odom_factor", 1.2);
}

double BrainConfig::get_vx_factor() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.vx_factor", 1.0);
}

double BrainConfig::get_yaw_offset() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.yaw_offset", 0.0);
}

double BrainConfig::get_vx_limit() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.vx_limit", 0.8);
}

double BrainConfig::get_vy_limit() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.vy_limit", 0.4);
}

double BrainConfig::get_vtheta_limit() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.vtheta_limit", 1.2);
}

double BrainConfig::get_head_yaw_limit_left() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.head_yaw_limit_left", 1.1);
}

double BrainConfig::get_head_yaw_limit_right() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.head_yaw_limit_right", -1.1);
}

double BrainConfig::get_head_pitch_limit_up() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.head_pitch_limit_up", 0.45);
}

double BrainConfig::get_min_vx() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.min_vx", 0.4);
}

double BrainConfig::get_min_vy() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.min_vy", 0.3);
}

double BrainConfig::get_min_vtheta() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("robot.min_vtheta", 0.2);
}

double BrainConfig::get_ball_confidence_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.ball_confidence_threshold", 50.0);
}

double BrainConfig::get_ball_memory_timeout() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.ball_memory_timeout", 5.0);
}

double BrainConfig::get_tm_ball_dist_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.tm_ball_dist_threshold", 2.0);
}

bool BrainConfig::get_limit_near_ball_speed() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.limit_near_ball_speed", true);
}

double BrainConfig::get_near_ball_speed_limit() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.near_ball_speed_limit", 0.2);
}

double BrainConfig::get_near_ball_range() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.near_ball_range", 3.0);
}

double BrainConfig::get_ball_out_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.ball_out_threshold", 2.0);
}

bool BrainConfig::get_abort_kick_when_ball_moved() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.abort_kick_when_ball_moved", true);
}

bool BrainConfig::get_enable_role_switch() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.cooperation.enable_role_switch", false);
}

double BrainConfig::get_ball_control_cost_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("strategy.cooperation.ball_control_cost_threshold", 2.0);
}


bool BrainConfig::get_enable_auto_visual_kick() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("RLVisionKick.enableAutoVisualKick", true);
}

double BrainConfig::get_auto_visual_kick_enable_dist_min() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("RLVisionKick.autoVisualKickEnableDistMin", 0.5);
}

double BrainConfig::get_auto_visual_kick_enable_dist_max() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("RLVisionKick.autoVisualKickEnableDistMax", 1.5);
}

double BrainConfig::get_auto_visual_kick_enable_angle() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("RLVisionKick.autoVisualKickEnableAngle", 0.5);
}


int BrainConfig::get_min_marker_count() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("locator.min_marker_count", 5);
}

double BrainConfig::get_max_residual() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("locator.max_residual", 0.3);
}



bool BrainConfig::get_enable_com() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("enable_com", false);
}


int BrainConfig::get_depth_sample_step() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.depth_sample_step", 16);
}

double BrainConfig::get_obstacle_min_height() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.obstacle_min_height", 0.05);
}

double BrainConfig::get_grid_size() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.grid_size", 0.1);
}

double BrainConfig::get_max_x() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.max_x", 6.0);
}

double BrainConfig::get_max_y() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.max_y", 4.0);
}

double BrainConfig::get_exclusion_x() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.exclusion_x", 0.5);
}

double BrainConfig::get_exclusion_y() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.exclusion_y", 0.5);
}

double BrainConfig::get_ball_exclusion_radius() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.ball_exclusion_radius", 0.3);
}

double BrainConfig::get_ball_exclusion_height() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.ball_exclusion_height", 0.5);
}

double BrainConfig::get_occupancy_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.occupancy_threshold", 3.0);
}

bool BrainConfig::get_enable_obstacle_avoidance() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.enable", false);
}

double BrainConfig::get_freekick_start_placing_safe_distance() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.freekick_start_placing_safe_distance", 3.0);
}

double BrainConfig::get_freekick_start_placing_avoid_secs() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.freekick_start_placing_avoid_secs", 3.0);
}

double BrainConfig::get_obstacle_memory_msecs() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.obstacle_memory_msecs", 500.0);
}

bool BrainConfig::get_avoid_during_kick() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.avoid_during_kick", false);
}

double BrainConfig::get_kick_ao_safe_dist() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.kick_ao_safe_dist", 1.0);
}

bool BrainConfig::get_avoid_during_chase() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.avoid_during_chase", true);
}

double BrainConfig::get_chase_ao_safe_dist() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.chase_ao_safe_dist", 2.0);
}

double BrainConfig::get_collision_threshold() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.collision_threshold", 0.2);
}

double BrainConfig::get_safe_distance() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.safe_distance", 0.5);
}

double BrainConfig::get_avoid_secs() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("obstacle_avoidance.avoid_secs", 2.0);
}


int BrainConfig::get_retry_max_count() {
    return static_cast<rclcpp::Node*>(brain)->get_parameter_or("recovery.retry_max_count", 3);
}


string BrainConfig::get_image_camera_info_topic() {
    string robot_name = get_robot_name();
    if (robot_name.empty()) {
        robot_name = "robot0";
    }
    return replace_all(static_cast<rclcpp::Node*>(brain)->get_parameter_or("vision.image_camera_info_topic", string("/camera/color/camera_info")), "robot0", robot_name);
}

string BrainConfig::get_depth_image_topic() {
    string robot_name = get_robot_name();
    if (robot_name.empty()) {
        robot_name = "robot0";
    }
    return replace_all(static_cast<rclcpp::Node*>(brain)->get_parameter_or("vision.depth_image_topic", string("/camera/depth/image_raw")), "robot0", robot_name);
}

string BrainConfig::get_depth_camera_info_topic() {
    string robot_name = get_robot_name();
    if (robot_name.empty()) {
        robot_name = "robot0";
    }
    return replace_all(static_cast<rclcpp::Node*>(brain)->get_parameter_or("vision.depth_camera_info_topic", string("/camera/depth/camera_info")), "robot0", robot_name);
}

void BrainConfig::calcMapLines() {
    auto fd = fieldDimensions;
    mapLines.clear();
    
    FieldLine oppoGoalLine;
    oppoGoalLine.posToField = {fd.length / 2, -fd.width / 2, fd.length / 2, fd.width / 2};
    oppoGoalLine.half = LineHalf::Opponent;
    oppoGoalLine.side = LineSide::NA;
    oppoGoalLine.dir = LineDir::Horizontal;
    oppoGoalLine.type = LineType::GoalLine;
    mapLines.push_back(oppoGoalLine);

    FieldLine selfGoalLine;
    selfGoalLine.posToField = {-fd.length / 2, -fd.width / 2, -fd.length / 2, fd.width / 2};
    selfGoalLine.half = LineHalf::Self;
    selfGoalLine.side = LineSide::NA;
    selfGoalLine.dir = LineDir::Horizontal;
    selfGoalLine.type = LineType::GoalLine;
    mapLines.push_back(selfGoalLine);

    FieldLine leftTouchLine;
    leftTouchLine.posToField = {-fd.length / 2, fd.width / 2, fd.length / 2, fd.width / 2};
    leftTouchLine.half = LineHalf::NA;
    leftTouchLine.side = LineSide::Left;
    leftTouchLine.dir = LineDir::Vertical;
    leftTouchLine.type = LineType::TouchLine;
    mapLines.push_back(leftTouchLine);

    FieldLine rightTouchLine;
    rightTouchLine.posToField = {-fd.length / 2, -fd.width / 2, fd.length / 2, -fd.width / 2};
    rightTouchLine.half = LineHalf::NA;
    rightTouchLine.side = LineSide::Right;
    rightTouchLine.dir = LineDir::Vertical;
    rightTouchLine.type = LineType::TouchLine;
    mapLines.push_back(rightTouchLine);

    FieldLine middleLine;
    middleLine.posToField = {0, -fd.width / 2, 0, fd.width / 2};
    middleLine.half = LineHalf::NA;
    middleLine.side = LineSide::NA;
    middleLine.dir = LineDir::Horizontal;
    middleLine.type = LineType::MiddleLine;
    mapLines.push_back(middleLine);

    FieldLine goalAreaLO;
    goalAreaLO.posToField = {fd.length / 2, -fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, -fd.goalAreaWidth / 2};
    goalAreaLO.half = LineHalf::Opponent;
    goalAreaLO.side = LineSide::Left;
    goalAreaLO.dir = LineDir::Vertical;
    goalAreaLO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaLO);

    FieldLine goalAreaRO;
    goalAreaRO.posToField = {fd.length / 2, fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaRO.half = LineHalf::Opponent;
    goalAreaRO.side = LineSide::Right;
    goalAreaRO.dir = LineDir::Vertical;
    goalAreaRO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaRO);

    FieldLine goalAreaHO;
    goalAreaHO.posToField = {fd.length / 2 - fd.goalAreaLength, -fd.goalAreaWidth / 2, fd.length / 2 - fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaHO.half = LineHalf::Opponent;
    goalAreaHO.side = LineSide::NA;
    goalAreaHO.dir = LineDir::Horizontal;
    goalAreaHO.type = LineType::GoalArea;
    mapLines.push_back(goalAreaHO);

    FieldLine penaltyAreaLO;
    penaltyAreaLO.posToField = {fd.length / 2, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2};
    penaltyAreaLO.half = LineHalf::Opponent;
    penaltyAreaLO.side = LineSide::Left;
    penaltyAreaLO.dir = LineDir::Vertical;
    penaltyAreaLO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaLO);

    FieldLine penaltyAreaRO;
    penaltyAreaRO.posToField = {fd.length / 2, fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaRO.half = LineHalf::Opponent;
    penaltyAreaRO.side = LineSide::Right;
    penaltyAreaRO.dir = LineDir::Vertical;
    penaltyAreaRO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaRO);

    FieldLine penaltyAreaHO;
    penaltyAreaHO.posToField = {fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaHO.half = LineHalf::Opponent;
    penaltyAreaHO.side = LineSide::NA;
    penaltyAreaHO.dir = LineDir::Horizontal;
    penaltyAreaHO.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaHO);

    FieldLine goalAreaLS;
    goalAreaLS.posToField = {-fd.length / 2, -fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, -fd.goalAreaWidth / 2};
    goalAreaLS.half = LineHalf::Self;
    goalAreaLS.side = LineSide::Left;
    goalAreaLS.dir = LineDir::Vertical;
    goalAreaLS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaLS);

    FieldLine goalAreaRS;
    goalAreaRS.posToField = {-fd.length / 2, fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaRS.half = LineHalf::Self;
    goalAreaRS.side = LineSide::Right;
    goalAreaRS.dir = LineDir::Vertical;
    goalAreaRS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaRS);

    FieldLine goalAreaHS;
    goalAreaHS.posToField = {-fd.length / 2 + fd.goalAreaLength, -fd.goalAreaWidth / 2, -fd.length / 2 + fd.goalAreaLength, fd.goalAreaWidth / 2};
    goalAreaHS.half = LineHalf::Self;
    goalAreaHS.side = LineSide::NA;
    goalAreaHS.dir = LineDir::Horizontal;
    goalAreaHS.type = LineType::GoalArea;
    mapLines.push_back(goalAreaHS);

    FieldLine penaltyAreaLS;
    penaltyAreaLS.posToField = {-fd.length / 2, -fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2};
    penaltyAreaLS.half = LineHalf::Self;
    penaltyAreaLS.side = LineSide::Left;
    penaltyAreaLS.dir = LineDir::Vertical;
    penaltyAreaLS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaLS);

    FieldLine penaltyAreaRS;
    penaltyAreaRS.posToField = {-fd.length / 2, fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaRS.half = LineHalf::Self;
    penaltyAreaRS.side = LineSide::Right;
    penaltyAreaRS.dir = LineDir::Vertical;
    penaltyAreaRS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaRS);

    FieldLine penaltyAreaHS;
    penaltyAreaHS.posToField = {-fd.length / 2 + fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, -fd.length / 2 + fd.penaltyAreaLength, fd.penaltyAreaWidth / 2};
    penaltyAreaHS.half = LineHalf::Self;
    penaltyAreaHS.side = LineSide::NA;
    penaltyAreaHS.dir = LineDir::Horizontal;
    penaltyAreaHS.type = LineType::PenaltyArea;
    mapLines.push_back(penaltyAreaHS);
}

void BrainConfig::calcMapMarkings() {
    auto fd = fieldDimensions;
    mapMarkings.clear();

    vector<string> halves = {"S", "O"};
    vector<string> sides = {"L", "R"};
    for (auto half : halves) {
        double xSign = half == "S" ? -1.0 : 1.0;
        for (auto side : sides) {
            double ySign = side == "L"? 1.0 : -1.0;
            
            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.width / 2,
                "LCross",
                "L" + half + side + "B"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.penaltyAreaWidth / 2,
                "TCross",
                "T" + half + side + "P"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * fd.length / 2,
                ySign * fd.goalAreaWidth / 2,
                "TCross",
                "T" + half + side + "G"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * (fd.length / 2 - fd.penaltyAreaLength),
                ySign * fd.penaltyAreaWidth / 2,
                "LCross",
                "L" + half + side + "P"
            }));

            mapMarkings.push_back(MapMarking({
                xSign * (fd.length / 2 - fd.goalAreaLength),
                ySign * fd.goalAreaWidth / 2,
                "LCross",
                "L" + half + side + "G"
            }));
        }
    }

    // Four points on the middle line
    for (auto side: sides) {
        double ySign = side == "L"? 1.0 : -1.0;

        mapMarkings.push_back(MapMarking({
            0,
            ySign * fd.width / 2,
            "TCross",
            "TM" + side + "B"
        }));

        mapMarkings.push_back(MapMarking({
            0,
            ySign * fd.circleRadius,
            "XCross",
            "XM" + side + "C"
        }));
    }

    // Two penalty points
    for (auto half : halves) {
        double xSign = half == "S"? -1.0 : 1.0;

        mapMarkings.push_back(MapMarking({
            xSign * (fd.length / 2 - fd.penaltyDist),
            0,
            "PenaltyPoint",
            "P" + half + "MP"
        }));
    }
}

void BrainConfig::handle()
{


    // fieldType [adult_size, kid_size]
    if (get_field_type() == "adult_size")
    {
        fieldDimensions = FD_ADULTSIZE;
    }
    else if (get_field_type() == "kid_size")
    {
        fieldDimensions = FD_KIDSIZE;
    }
    else if (get_field_type() == "robo_league")
    {
        fieldDimensions = FD_ROBOLEAGUE;
    }
    else
    {
        throw invalid_argument("[Error] fieldType must be one of [adult_size, kid_size, robo_league]. Got: " + get_field_type());
    }
    calcMapLines();
    calcMapMarkings();
}

void BrainConfig::print(ostream &os)
{
    os << "Configs:" << endl;
    os << "----------------------------------------" << endl;
    os << "Game:" << endl;
    os << "    fieldType = " << get_field_type() << endl;
    os << "    treatPersonAsRobot = " << get_treat_person_as_robot() << endl;
    os << "    numOfPlayers = " << get_num_of_players() << endl;
    os << "----------------------------------------" << endl;
    os << "Robot:" << endl;
    os << "    robotName = " << get_robot_name() << endl;
    os << "    robotHeight = " << get_robot_height() << endl;
    os << "    robotOdomFactor = " << get_robot_odom_factor() << endl;
    os << "    vxFactor = " << get_vx_factor() << endl;
    os << "    yawOffset = " << get_yaw_offset() << endl;
    os << "    vxLimit = " << get_vx_limit() << endl;
    os << "    vyLimit = " << get_vy_limit() << endl;
    os << "    vthetaLimit = " << get_vtheta_limit() << endl;
    os << "----------------------------------------" << endl;
    os << "Strategy:" << endl;
    os << "    ballConfidenceThreshold = " << get_ball_confidence_threshold() << endl;
    os << "----------------------------------------" << endl;
    os << "Locator:" << endl;
    os << "    pfMinMarkerCnt = " << get_min_marker_count() << endl;
    os << "    pfMaxResidual = " << get_max_residual() << endl;
    os << "----------------------------------------" << endl;
    os << "Communication:" << endl;
    os << "    enableCom = " << get_enable_com() << endl;
    os << "----------------------------------------" << endl;
}