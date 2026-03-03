#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <vector>

class VisualizationPublisher
{
public:
    VisualizationPublisher(rclcpp::Node *node);

    /**
     * @brief Create a robot marker showing position and orientation (arrow)
     * @param x Robot x coordinate
     * @param y Robot y coordinate
     * @param theta Robot heading angle (radians)
     * @param frame_id Frame id, typically "map" or "odom"
     */
    visualization_msgs::msg::Marker createRobotMarker(double x, double y, double theta, const std::string &frame_id = "map");

    /**
     * @brief Create a ball marker (sphere)
     * @param x Ball x coordinate
     * @param y Ball y coordinate
     * @param z Ball z coordinate (default 0.043m, approximate football radius)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createBallMarker(double x, double y, double z = 0.043, const std::string &frame_id = "map");

    /**
     * @brief Create a mark point marker (cylinder)
     * @param x Mark point x coordinate
     * @param y Mark point y coordinate
     * @param marker_type Mark type ('X', 'T', 'L', 'P')
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createMarkPointMarker(double x, double y, char marker_type = 'X', const std::string &frame_id = "map");

    /**
     * @brief Create field center line marker
     * @param field_length Field length (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createFieldCenterLineMarker(double field_length, const std::string &frame_id = "map");

    /**
     * @brief Create field center circle marker
     * @param center_radius Center circle radius (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createFieldCenterCircleMarker(double center_radius, const std::string &frame_id = "map");

    /**
     * @brief Create field boundary marker (rectangular outline)
     * @param field_length Field length (m)
     * @param field_width Field width (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createFieldBoundaryMarker(double field_length, double field_width, const std::string &frame_id = "map");

    /**
     * @brief Create goal area marker
     * @param is_our_side Whether it is our goal area
     * @param field_length Field length (m)
     * @param goal_area_length Goal area length (m)
     * @param goal_area_width Goal area width (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createGoalAreaMarker(bool is_our_side, double field_length, double goal_area_length, double goal_area_width, const std::string &frame_id = "map");

    /**
     * @brief Create penalty area (large penalty box) marker
     * @param is_our_side Whether it is our penalty area
     * @param field_length Field length (m)
     * @param penalty_area_length Penalty area length (m)
     * @param penalty_area_width Penalty area width (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createPenaltyAreaMarker(bool is_our_side, double field_length, double penalty_area_length, double penalty_area_width, const std::string &frame_id = "map");

    /**
     * @brief Create penalty point marker
     * @param is_our_side Whether it is our penalty point
     * @param field_length Field length (m)
     * @param penalty_dist Distance from penalty point to baseline (m)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createPenaltyPointMarker(bool is_our_side, double field_length, double penalty_dist, const std::string &frame_id = "map");

    /**
     * @brief Create a teammate pose marker
     * @param team_id Team ID
     * @param player_id Player ID
     * @param x Teammate x coordinate
     * @param y Teammate y coordinate
     * @param theta Teammate heading angle (radians)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createTeammateMarker(int team_id, int player_id, double x, double y, double theta, const std::string &frame_id = "map");

    /**
     * @brief Create a marker for the ball observed by a teammate
     * @param team_id Team ID
     * @param player_id Player ID
     * @param x Ball x coordinate
     * @param y Ball y coordinate
     * @param z Ball z coordinate
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createTeammateBallMarker(int team_id, int player_id, double x, double y, double z = 0.043, const std::string &frame_id = "map");

    /**
     * @brief Create markers for multiple observed mark points (includes removal of old markers and addition of new ones)
     * @param mark_points List of mark points with coordinates and type (x, y, type)
     * @param frame_id Frame id
     * @return Array containing old-marker removals and new markers
     */
    std::vector<visualization_msgs::msg::Marker> createObservedMarkPointMarkers(
        const std::vector<std::tuple<double, double, char>> &mark_points, 
        const std::string &frame_id = "map");

    /**
     * @brief Create markers for multiple observed field lines (includes removal of old markers and addition of new ones)
     * @param lines List of lines represented by point coordinates
     * @param frame_id Frame id
     * @return Array containing old-marker removals and new markers
     */
    std::vector<visualization_msgs::msg::Marker> createObservedFieldLineMarkers(
        const std::vector<std::vector<geometry_msgs::msg::Point>> &lines, 
        const std::string &frame_id = "map");

    /**
     * @brief Publish all markers
     * @param markers Marker array
     */
    void publishMarkers(const visualization_msgs::msg::MarkerArray &markers);

    /**
     * @brief Create GameController info marker (displays match state, score, etc.)
     * @param my_score Our score
     * @param oppo_score Opponent score
     * @param remaining_time Remaining time (seconds)
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createGameControllerInfoMarker(
        int my_score,
        int oppo_score,
        int remaining_time = -1,
        const std::string &frame_id = "map");

    /**
     * @brief Create GameController state marker (displays match state and substate)
     * @param game_state Game state (e.g. "INITIAL", "READY", "SET", "PLAY", "FINISHED")
     * @param game_sub_state Game sub-state (e.g. "NORMAL", "TIMEOUT")
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createGameControllerStateMarker(
        const std::string &game_state,
        const std::string &game_sub_state,
        const std::string &frame_id = "map");

    /**
     * @brief Create Decision info marker (displays decision information)
     * @param role Role type ("striker" or "goalie")
     * @param decision Decision type (e.g. "find", "chase", "adjust", "kick")
     * @param ball_range Ball distance (m)
     * @param ball_yaw Ball yaw (radians)
     * @param kick_dir Kick direction (radians)
     * @param rb_dir Robot-ball angle (radians)
     * @param angle_good Whether angle is favorable
     * @param is_lead Whether robot is the lead
     * @param frame_id Frame id
     */
    visualization_msgs::msg::Marker createDecisionInfoMarker(
        const std::string &role,
        const std::string &decision,
        double ball_range,
        double ball_yaw,
        double kick_dir,
        double rb_dir,
        bool angle_good,
        bool is_lead,
        const std::string &frame_id = "map");

    /**
     * @brief Publish point cloud data
     * @param points Point cloud data (x, y, z)
     * @param frame_id Frame id
     */
    void publishPointCloud(const std::vector<std::tuple<float, float, float>> &points, const std::string &frame_id = "map");

    /**
     * @brief Publish obstacle occupancy grid
     * @param grid_data Grid data (0-100 occupancy probability, -1 unknown)
     * @param width Grid width (cells)
     * @param height Grid height (cells)
     * @param resolution Grid resolution (m per cell)
     * @param origin_x Origin x coordinate (m)
     * @param origin_y Origin y coordinate (m)
     * @param frame_id Frame id
     */
    void publishObstacleGrid(const std::vector<int8_t> &grid_data, 
                            uint32_t width, uint32_t height, 
                            float resolution,
                            double origin_x = 0.0, double origin_y = 0.0,
                            const std::string &frame_id = "map");

    /**
     * @brief Publish player decision information
     * @param message Decision message content
     */
    void publishPlayerDecision(const std::string &message);

    /**
     * @brief Get a ColorRGBA for markers
     */
    static std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a = 1.0f);

private:
    rclcpp::Node *node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_grid_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubPlayerDecision_;


    // Fixed marker IDs - robot itself
    static constexpr uint32_t ROBOT_MARKER_ID = 0;
    static constexpr uint32_t BALL_MARKER_ID = 1;
    static constexpr uint32_t GAME_CONTROLLER_STATE_ID = 2;
    static constexpr uint32_t GAME_CONTROLLER_INFO_ID = 3;
    static constexpr uint32_t DECISION_INFO_ID = 4;

    // Field map marker IDs (fixed)
    static constexpr uint32_t FIELD_CENTER_LINE_ID = 100;
    static constexpr uint32_t FIELD_CENTER_CIRCLE_ID = 101;
    static constexpr uint32_t FIELD_BOUNDARY_ID = 102;
    static constexpr uint32_t FIELD_GOAL_AREA_OUR_ID = 103;
    static constexpr uint32_t FIELD_GOAL_AREA_OPP_ID = 104;
    static constexpr uint32_t FIELD_PENALTY_AREA_OUR_ID = 105;
    static constexpr uint32_t FIELD_PENALTY_AREA_OPP_ID = 106;
    static constexpr uint32_t FIELD_PENALTY_POINT_OUR_ID = 107;
    static constexpr uint32_t FIELD_PENALTY_POINT_OPP_ID = 108;

    // Teammate marker ID range (fixed, computed by team_id and player_id)
    // ID = TEAMMATE_MARKER_ID_BASE + team_id * 100 + player_id
    static constexpr uint32_t TEAMMATE_MARKER_ID_BASE = 1000;
    
    // Teammate-observed ball marker ID range (fixed, computed by team_id and player_id)
    // ID = TEAMMATE_BALL_MARKER_ID_BASE + team_id * 100 + player_id
    static constexpr uint32_t TEAMMATE_BALL_MARKER_ID_BASE = 2000;


    // Dynamic marker ID ranges (remove old markers and add new ones each update)
    static constexpr uint32_t OBSERVED_MARK_POINT_ID_START = 5000;  // Starting ID for observed mark points
    static constexpr uint32_t OBSERVED_FIELD_LINE_ID_START = 6000;  // Starting ID for observed field lines
    static constexpr uint32_t MAX_OBSERVED_MARKS = 100;  // Max observed mark points
    static constexpr uint32_t MAX_OBSERVED_LINES = 100;  // Max observed field lines

    // Track current counts of dynamic observed markers
    uint32_t current_observed_marks_count_ = 0;
    uint32_t current_observed_lines_count_ = 0;

    // predefined colors for different marker types
    static constexpr float ROBOT_COLOR_R = 0.0f;
    static constexpr float ROBOT_COLOR_G = 1.0f;
    static constexpr float ROBOT_COLOR_B = 0.0f;

    static constexpr float BALL_COLOR_R = 1.0f;
    static constexpr float BALL_COLOR_G = 1.0f;
    static constexpr float BALL_COLOR_B = 0.0f;

    static constexpr float LINE_COLOR_R = 1.0f;
    static constexpr float LINE_COLOR_G = 1.0f;
    static constexpr float LINE_COLOR_B = 1.0f;
};
