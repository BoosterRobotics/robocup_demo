/**
 * @file types.h
 * @brief Define structs and enums used in the `brain` project
 */
#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>

using namespace std;

/* ------------------ Struct ------------------------*/

// Field dimensions: measurements parallel to the baseline are lengths,
// measurements perpendicular to the baseline are widths. Names align with rules.
struct FieldDimensions
{
    double length;            // Field length
    double width;             // Field width
    double penaltyDist;       // Distance from penalty mark to baseline
    double goalWidth;         // Goal width
    double circleRadius;      // Center circle radius
    double penaltyAreaLength; // Penalty area length
    double penaltyAreaWidth;  // Penalty area width
    double goalAreaLength;    // Goal area length
    double goalAreaWidth;     // Goal area width
                              // Note: naming is chosen to align with competition rules.
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.6, 2.6, 2, 2.25, 6.9, 0.75, 3.9};

// Pose2D: records a 2D point and orientation
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, 0 along +x axis, increasing counter-clockwise
};

// PoseBox2D: constraints for localization search ranges
struct PoseBox2D
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double thetamin;
    double thetamax;
};

// FieldMarker: field marking point
struct FieldMarker
{
    char type;           // L|T|X|P indicating marker type; P = penalty mark
    double x, y;         // Marker position (m)
    double confidence;   // Detection confidence
};

// LocateResult: result of localization
struct LocateResult
{
    bool success = false;
    int code = -1;         // 0: success; 1: failed to generate particles (count=0); 2: unreasonable residual after convergence; 3: not converged; 4: insufficient markers; 5: all particle probabilities too low; -1: initial state
    double residual = 0;   // Mean residual error
    Pose2D pose;
    int msecs = 0;         // Localization time in milliseconds
};

// Point: 3D point
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D: 2D point
struct Point2D
{
    double x;
    double y;
};

// BoundingBox
struct BoundingBox
{
    double xmin, xmax, ymin, ymax;
};

// GameObject: stores important match entities such as Ball, Goalpost, etc.
// Compared to detection::DetectedObject from /detect, this contains richer information.
struct GameObject
{
    string label;
    string color;
    BoundingBox boundingBox;
    Point2D precisePixelPoint;
    double confidence;
    Point posToRobot;

    Point posToField;
    double range;
    double pitchToRobot, yawToRobot;
    rclcpp::Time timePoint; // Time when the object was detected

    int id;
    string name;
    double idConfidence;
    string info;
};

// Line
struct Line {
    double x0, y0, x1, y1;
};

enum class LineHalf {
    NA,
    Self,
    Opponent
};

enum class LineDir {
    NA,
    Horizontal,
    Vertical
};

enum class LineSide {
    NA,
    Left,
    Right
};

enum class LineType {
    NA,
    TouchLine,
    MiddleLine,
    GoalLine,
    GoalArea,
    PenaltyArea
};

struct FieldLine {
    Line posToField;
    Line posToRobot;
    Line posOnCam;
    rclcpp::Time timePoint;
    LineDir dir;
    LineHalf half;
    LineSide side;
    LineType type;
    double confidence;
};

struct MapMarking {
    double x;
    double y;
    string type; // TCross | LCross | XCross | PenaltyPoint
    string name;
};

// VisionBox: field of view bounds
struct VisionBox {
    vector<double> posToField;
    vector<double> posToRobot;
    rclcpp::Time timePoint; // Time when object was detected
};

struct TimestampedData {
    vector<double> data;
    rclcpp::Time timePoint;
};

// Robot recovery state data
struct RobotRecoveryStateData {
    uint8_t state;
    uint8_t is_recovery_available;
    uint8_t current_planner_index;
};

// Team-mate communication status
struct TMStatus {
    string role = "not initialized"; // striker, goal_keeper
    bool isAlive = false;
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.;
    double ballRange = 0.;
    double cost = 0.;
    bool isLead = true;
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir = 0.;
    double thetaRb = 0.;
    int cmd = 0;
    int cmdId = 0;
    rclcpp::Time timeLastCom;
};


enum class RobotRecoveryState {
    IS_READY = 0,
    IS_FALLING = 1,
    HAS_FALLEN = 2,
    IS_GETTING_UP = 3
};
