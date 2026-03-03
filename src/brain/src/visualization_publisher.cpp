#include "visualization_publisher.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <iomanip>

VisualizationPublisher::VisualizationPublisher(rclcpp::Node *node)
    : node_(node)
{
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/booster_soccer/visualization_markers", 10);
    point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/booster_soccer/visualization_point_cloud", 10);
    obstacle_grid_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/booster_soccer/visualization_obstacle_grid", 10);
    pubPlayerDecision_ = node_->create_publisher<std_msgs::msg::String>(
        "/booster_soccer/player_decision", 10);
}

visualization_msgs::msg::Marker VisualizationPublisher::createRobotMarker(
    double x, double y, double theta, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "robot";
    marker.id = ROBOT_MARKER_ID;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;

    // Orientation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = 0.3;  // Arrow length
    marker.scale.y = 0.1;  // Arrow width
    marker.scale.z = 0.1;  // Arrow height

    marker.color = getColor(ROBOT_COLOR_R, ROBOT_COLOR_G, ROBOT_COLOR_B);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createBallMarker(
    double x, double y, double z, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "ball";
    marker.id = BALL_MARKER_ID;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    // Size (soccer ball diameter approximately 0.22m)
    marker.scale.x = 0.22;
    marker.scale.y = 0.22;
    marker.scale.z = 0.22;

    // Color (yellow)
    marker.color = getColor(BALL_COLOR_R, BALL_COLOR_G, BALL_COLOR_B);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createMarkPointMarker(
    double x, double y, char marker_type, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field_markers";
    marker.id = OBSERVED_MARK_POINT_ID_START;  // Default to the starting ID for observed mark points
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.05;  // Height

    // Size
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;   // Height

    // Choose color based on type
    float r = 1.0f, g = 0.0f, b = 0.0f;
    switch (marker_type)
    {
    case 'X':
        r = 1.0f;
        g = 0.0f;
        b = 0.0f;  // Red
        break;
    case 'T':
        r = 0.0f;
        g = 0.0f;
        b = 1.0f;  // Blue
        break;
    case 'L':
        r = 1.0f;
        g = 0.5f;
        b = 0.0f;  // Orange
        break;
    case 'P':
        r = 1.0f;
        g = 0.0f;
        b = 1.0f;  // Purple
        break;
    }

    marker.color = getColor(r, g, b);

    return marker;
}

void VisualizationPublisher::publishMarkers(const visualization_msgs::msg::MarkerArray &markers)
{
    marker_publisher_->publish(markers);
}

void VisualizationPublisher::publishPlayerDecision(const std::string &message)
{
    std_msgs::msg::String msg;
    msg.data = message;
    pubPlayerDecision_->publish(msg);
}

std_msgs::msg::ColorRGBA VisualizationPublisher::getColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

// ======================== Field Markers ========================

visualization_msgs::msg::Marker VisualizationPublisher::createFieldCenterLineMarker(
    double field_length, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = FIELD_CENTER_LINE_ID;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Center line: from x=0, y=-field_width/2 to x=0, y=field_width/2
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.0;
    p1.y = -field_length / 2.0;
    p1.z = 0.0;
    p2.x = 0.0;
    p2.y = field_length / 2.0;
    p2.z = 0.0;
    
    marker.points.push_back(p1);
    marker.points.push_back(p2);

    marker.scale.x = 0.05;  // line width
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // permanent display

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createFieldCenterCircleMarker(
    double center_radius, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = FIELD_CENTER_CIRCLE_ID;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Create a circle with center at (0,0)
    const int segments = 64;
    for (int i = 0; i <= segments; ++i)
    {
        double angle = 2.0 * M_PI * i / segments;
        geometry_msgs::msg::Point p;
        p.x = center_radius * cos(angle);
        p.y = center_radius * sin(angle);
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker.scale.x = 0.05;  // line width
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createFieldBoundaryMarker(
    double field_length, double field_width, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = FIELD_BOUNDARY_ID;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Field boundary (rectangle)
    double half_length = field_length / 2.0;
    double half_width = field_width / 2.0;

    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x = -half_length; p1.y = -half_width; p1.z = 0.0;
    p2.x = half_length;  p2.y = -half_width; p2.z = 0.0;
    p3.x = half_length;  p3.y = half_width;  p3.z = 0.0;
    p4.x = -half_length; p4.y = half_width;  p4.z = 0.0;
    p5 = p1;  // Close the shape

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p5);

    marker.scale.x = 0.05;  // line width
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createGoalAreaMarker(
    bool is_our_side, double field_length, double goal_area_length, 
    double goal_area_width, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = is_our_side ? FIELD_GOAL_AREA_OUR_ID : FIELD_GOAL_AREA_OPP_ID;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Goal area position
    double x_pos = is_our_side ? (-field_length / 2.0) : (field_length / 2.0);
    double x_end = is_our_side ? (x_pos + goal_area_length) : (x_pos - goal_area_length);
    double half_width = goal_area_width / 2.0;

    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = x_pos; p1.y = -half_width; p1.z = 0.0;
    p2.x = x_end; p2.y = -half_width; p2.z = 0.0;
    p3.x = x_end; p3.y = half_width;  p3.z = 0.0;
    p4.x = x_pos; p4.y = half_width;  p4.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);

    marker.scale.x = 0.05;  // line width
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createPenaltyAreaMarker(
    bool is_our_side, double field_length, double penalty_area_length, 
    double penalty_area_width, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = is_our_side ? FIELD_PENALTY_AREA_OUR_ID : FIELD_PENALTY_AREA_OPP_ID;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Penalty area position
    double x_pos = is_our_side ? (-field_length / 2.0) : (field_length / 2.0);
    double x_end = is_our_side ? (x_pos + penalty_area_length) : (x_pos - penalty_area_length);
    double half_width = penalty_area_width / 2.0;

    geometry_msgs::msg::Point p1, p2, p3, p4;
    p1.x = x_pos; p1.y = -half_width; p1.z = 0.0;
    p2.x = x_end; p2.y = -half_width; p2.z = 0.0;
    p3.x = x_end; p3.y = half_width;  p3.z = 0.0;
    p4.x = x_pos; p4.y = half_width;  p4.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);

    marker.scale.x = 0.05;  // line width
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createPenaltyPointMarker(
    bool is_our_side, double field_length, double penalty_dist, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "field";
    marker.id = is_our_side ? FIELD_PENALTY_POINT_OUR_ID : FIELD_PENALTY_POINT_OPP_ID;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Penalty point position
    double x_pos = is_our_side ? (-field_length / 2.0 + penalty_dist) : (field_length / 2.0 - penalty_dist);

    marker.pose.position.x = x_pos;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    // Size (penalty point marker, smaller)
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.05;

    // Color (white)
    marker.color = getColor(LINE_COLOR_R, LINE_COLOR_G, LINE_COLOR_B);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

// ======================== Teammate Pose and Ball Markers ========================

visualization_msgs::msg::Marker VisualizationPublisher::createTeammateMarker(
    int team_id, int player_id, double x, double y, double theta, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "teammates";
    marker.id = TEAMMATE_MARKER_ID_BASE + team_id * 100 + player_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;

    // Orientation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    marker.pose.orientation = tf2::toMsg(q);

    // Size (slightly smaller than own robot)
    marker.scale.x = 0.25;  // Arrow length
    marker.scale.y = 0.08;  // Arrow width
    marker.scale.z = 0.08;  // Arrow height

    // Color (blue, different from own green)
    marker.color = getColor(0.0f, 0.5f, 1.0f);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createTeammateBallMarker(
    int team_id, int player_id, double x, double y, double z, const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "teammate_balls";
    marker.id = TEAMMATE_BALL_MARKER_ID_BASE + team_id * 100 + player_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    // Size (slightly smaller than a real soccer ball)
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    // Color (orange, different from own yellow ball)
    marker.color = getColor(1.0f, 0.5f, 0.0f);
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    return marker;
}

// ======================== Dynamic Marker Management ========================

std::vector<visualization_msgs::msg::Marker> VisualizationPublisher::createObservedMarkPointMarkers(
    const std::vector<std::tuple<double, double, char>> &mark_points,
    const std::string &frame_id)
{
    std::vector<visualization_msgs::msg::Marker> markers;
    
    // First, delete all observed mark points in the namespace (using DELETEALL to avoid ID conflicts)
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = node_->now();
    delete_all.ns = "observed_marks";
    delete_all.id = 0;  // ID is irrelevant when using DELETEALL
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.push_back(delete_all);
    
    // Create new mark point markers
    uint32_t new_count = 0;
    for (const auto &[x, y, marker_type] : mark_points)
    {
        if (new_count >= MAX_OBSERVED_MARKS)
        {
            RCLCPP_WARN(node_->get_logger(), 
                "Observed mark points exceed maximum limit (%d)", MAX_OBSERVED_MARKS);
            break;
        }
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "observed_marks";
        marker.id = OBSERVED_MARK_POINT_ID_START + new_count;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.05;  // Height

        // Size
        marker.scale.x = 0.1;   // Diameter
        marker.scale.y = 0.1;   // Diameter
        marker.scale.z = 0.1;   // Height

        // Choose color based on type
        float r = 1.0f, g = 0.0f, b = 0.0f;
        switch (marker_type)
        {
        case 'X':
            r = 1.0f; g = 0.0f; b = 0.0f;  // Red
            break;
        case 'T':
            r = 0.0f; g = 0.0f; b = 1.0f;  // Blue
            break;
        case 'L':
            r = 1.0f; g = 0.5f; b = 0.0f;  // Orange
            break;
        case 'P':
            r = 1.0f; g = 0.0f; b = 1.0f;  // Purple
            break;
        }

        marker.color = getColor(r, g, b);
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        markers.push_back(marker);
        new_count++;
    }
    
    // Update count
    current_observed_marks_count_ = new_count;
    
    return markers;
}

std::vector<visualization_msgs::msg::Marker> VisualizationPublisher::createObservedFieldLineMarkers(
    const std::vector<std::vector<geometry_msgs::msg::Point>> &lines,
    const std::string &frame_id)
{
    std::vector<visualization_msgs::msg::Marker> markers;
    
    // First, delete all observed field lines in the namespace (using DELETEALL to avoid ID conflicts)
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = node_->now();
    delete_all.ns = "observed_lines";
    delete_all.id = 0;  // ID is irrelevant when using DELETEALL
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.push_back(delete_all);
    
    // Create new field line markers
    uint32_t new_count = 0;
    for (const auto &line_points : lines)
    {
        if (new_count >= MAX_OBSERVED_LINES)
        {
            RCLCPP_WARN(node_->get_logger(), 
                "Observed field lines exceed maximum limit (%d)", MAX_OBSERVED_LINES);
            break;
        }
        
        if (line_points.empty())
        {
            continue;  // Skip empty lines
        }
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = node_->now();
        marker.ns = "observed_lines";
        marker.id = OBSERVED_FIELD_LINE_ID_START + new_count;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Add all points
        marker.points = line_points;

        // Line width
        marker.scale.x = 0.03;

        // Color (cyan, different from the white field map)
        marker.color = getColor(0.0f, 1.0f, 1.0f);
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        markers.push_back(marker);
        new_count++;
    }
    
    // Update count
    current_observed_lines_count_ = new_count;
    
    return markers;
}

// ======================== Point Cloud Publishing ========================

void VisualizationPublisher::publishPointCloud(
    const std::vector<std::tuple<float, float, float>> &points,
    const std::string &frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = node_->now();
    
    // Set point cloud format
    cloud_msg.height = 1;
    cloud_msg.width = points.size();
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;
    
    // Set fields (x, y, z)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    
    modifier.resize(points.size());
    
    // Fill point cloud data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    
    for (const auto &point : points)
    {
        *iter_x = std::get<0>(point);
        *iter_y = std::get<1>(point);
        *iter_z = std::get<2>(point);
        
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    point_cloud_publisher_->publish(cloud_msg);
}

// ======================== Obstacle Grid Publishing ========================

void VisualizationPublisher::publishObstacleGrid(
    const std::vector<int8_t> &grid_data,
    uint32_t width, uint32_t height,
    float resolution,
    double origin_x, double origin_y,
    const std::string &frame_id)
{
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = frame_id;
    grid_msg.header.stamp = node_->now();
    
    // Set map metadata
    grid_msg.info.resolution = resolution;
    grid_msg.info.width = width;
    grid_msg.info.height = height;
    
    // Set origin (coordinates of the bottom-left corner of the map)
    grid_msg.info.origin.position.x = origin_x;
    grid_msg.info.origin.position.y = origin_y;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;
    
    // Check if data size matches
    if (grid_data.size() != width * height)
    {
        RCLCPP_ERROR(node_->get_logger(), 
            "Grid data size (%zu) does not match width*height (%d)", 
            grid_data.size(), width * height);
        return;
    }
    
    // Set grid data
    grid_msg.data = grid_data;
    
    obstacle_grid_publisher_->publish(grid_msg);
}

// ======================== GameController Info Publishing ========================

visualization_msgs::msg::Marker VisualizationPublisher::createGameControllerInfoMarker(
    int my_score,
    int oppo_score,
    int remaining_time,
    const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "game_controller_info";
    marker.id = GAME_CONTROLLER_INFO_ID;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position: Display outside the field boundary (does not affect in-field view)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 8.0;  // Place outside one side of the field boundary
    marker.pose.position.z = 2.5;  // Height 2.5 meters for better visibility
    marker.pose.orientation.w = 1.0;

    // Size (text height)
    marker.scale.z = 0.35;  // Text height 35cm

    float r = 1.0f, g = 1.0f, b = 1.0f;

    marker.color = getColor(r, g, b);

    // Construct display text (using actual newline characters)
    std::stringstream ss;
    ss << "\nScore: " << my_score << " - " << oppo_score;
    if (remaining_time >= 0) {
        int minutes = remaining_time / 60;
        int seconds = remaining_time % 60;
        ss << "\nTime: " << minutes << ":" << (seconds < 10 ? "0" : "") << seconds;
    }
    
    marker.text = ss.str();
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display

    return marker;
}


visualization_msgs::msg::Marker VisualizationPublisher::createGameControllerStateMarker(
    const std::string &game_state,
    const std::string &game_sub_state,
    const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "game_controller_state";
    marker.id = GAME_CONTROLLER_STATE_ID;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position: Display outside the field boundary (does not affect in-field view)
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 8.0;  // Place outside one side of the field boundary
    marker.pose.position.z = 3.5;  // Height 3.5 meters for better visibility
    marker.pose.orientation.w = 1.0;

    // Size (text height)
    marker.scale.z = 0.45;  // Text height 45cm

    // Color: Set different colors based on game state
    float r = 1.0f, g = 1.0f, b = 1.0f;
    if (game_state == "INITIAL") {
        r = 1.0f; g = 1.0f; b = 0.0f;  // Yellow
    } else if (game_state == "READY") {
        r = 0.0f; g = 1.0f; b = 1.0f;  // Cyan
    } else if (game_state == "SET") {
        r = 1.0f; g = 0.5f; b = 0.0f;  // Orange
    } else if (game_state == "PLAY") {
        r = 0.0f; g = 1.0f; b = 0.0f;  // Green
    } else if (game_state == "FINISHED") {
        r = 1.0f; g = 0.0f; b = 0.0f;  // Red
    }
    marker.color = getColor(r, g, b);

    // Construct display text (using actual newline characters)
    std::stringstream ss;
    ss << "State: " << game_state;
    if (!game_sub_state.empty() && game_sub_state != "NORMAL") {
        ss << " (" << game_sub_state << ")";
    }
    
    marker.text = ss.str();
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display

    return marker;
}

visualization_msgs::msg::Marker VisualizationPublisher::createDecisionInfoMarker(
    const std::string &role,
    const std::string &decision,
    double ball_range,
    double ball_yaw,
    double kick_dir,
    double rb_dir,
    bool angle_good,
    bool is_lead,
    const std::string &frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "decision_info";
    marker.id = DECISION_INFO_ID;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Position
    marker.pose.position.x = -10.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 2.5;   // Height 2.5 meters for better visibility
    marker.pose.orientation.w = 1.0;

    // Size (text height)
    marker.scale.z = 0.25;  // Text height 25cm

    // Color: Set different colors based on decision type
    float r = 1.0f, g = 1.0f, b = 1.0f;
    if (decision == "find") {
        r = 1.0f; g = 1.0f; b = 1.0f;  // White
    } else if (decision == "assist") {
        r = 0.0f; g = 1.0f; b = 1.0f;  // Cyan
    } else if (decision == "chase") {
        r = 0.0f; g = 0.0f; b = 1.0f;  // Blue
    } else if (decision == "adjust") {
        r = 1.0f; g = 1.0f; b = 0.0f;  // Yellow
    } else if (decision == "kick" || decision == "cross") {
        r = 0.0f; g = 1.0f; b = 0.0f;  // Green
    } else if (decision == "retreat") {
        r = 1.0f; g = 0.0f; b = 1.0f;  // Purple
    }
    marker.color = getColor(r, g, b);

    // Construct display text (using actual newline characters)
    std::stringstream ss;
    ss << "[" << role << "]";
    ss << " Decision: " << decision;
    ss << "\nBall Range: " << std::fixed << std::setprecision(2) << ball_range << " m";
    ss << "\nBall Yaw: " << std::fixed << std::setprecision(2) << ball_yaw << " rad";
    ss << "\nKick Dir: " << std::fixed << std::setprecision(2) << kick_dir << " rad";
    ss << "\nRB Dir: " << std::fixed << std::setprecision(2) << rb_dir << " rad";
    ss << "\nAngle Good: " << (angle_good ? "Yes" : "No");
    if (role == "striker") {
        ss << "\nIs Lead: " << (is_lead ? "Yes" : "No");
    }
    
    marker.text = ss.str();
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Permanent display

    return marker;
}

