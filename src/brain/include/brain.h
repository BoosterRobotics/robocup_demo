#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_interface/msg/detections.hpp>
#include <vision_interface/msg/line_segments.hpp>
#include <vision_interface/msg/cal_param.hpp>
#include <vision_interface/msg/segmentation_result.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include "brain/msg/kick.hpp"
#include <booster/robot/b1/b1_api_const.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "booster_interface/msg/odometer.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/raw_bytes_msg.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"

#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>

#include "brain_config.h"
#include "brain_data.h"
#include "brain_log.h"
#include "brain_tree.h"
#include "brain_communication.h"
#include "locator.h"
#include "robot_client.h"
#include "visualization_publisher.h"

using namespace std;

/**
 * Core `Brain` class. As shared pointers are passed to `BrainTree`, this class
 * must support shared_from_this semantics.
 * Data is encapsulated in sub-objects rather than stored directly in `Brain`.
 * Static configuration values live in `config`; dynamic runtime data lives in `data`.
 * TODO: Some data is also stored in the BehaviorTree blackboard; consider removing duplicates.
 * Current design passes the `brain` pointer into BehaviorTree nodes which directly access `brain->config` and `brain->data`.
 */
class Brain : public rclcpp::Node
{
public:
    // BrainConfig object, mainly contains configuration values needed at runtime (static)
    std::shared_ptr<BrainConfig> config;
    // BrainLog object, encapsulates log-related operations
    std::shared_ptr<BrainLog> log;
    // BrainData object, all runtime values of Brain are stored here
    std::shared_ptr<BrainData> data;
    // RobotClient object, contains all operations related to the robot
    std::shared_ptr<RobotClient> client;
    // Locator object, contains robot localization related operations
    std::shared_ptr<Locator> locator;
    // VisualizationPublisher object, used to publish visualization markers
    std::shared_ptr<VisualizationPublisher> visualizer;
    // BrainTree object, contains BehaviorTree related operations
    std::shared_ptr<BrainTree> tree;
    // Communication object, contains communication related operations, mainly for dual-machine communication and referee communication
    std::shared_ptr<BrainCommunication> communication;

    // Constructor: create ROS2 node (nodeName handled in implementation)
    Brain();

    ~Brain();

    // Initialize operations, only need to be called once in the main function. If the initialization process does not meet expectations, an exception can be thrown to terminate the program.
    void init();

    // Called in the ROS2 loop
    void tick();

    // Handle special states, such as kickoff, free kick, etc.
    void handleSpecialStates();

    void registerLocatorNodes(BT::BehaviorTreeFactory &factory)
    {
        RegisterLocatorNodes(factory, this);
    }

    /**
     * @brief Calculate the vector angles from the current ball position to the opponent's goalposts, in the field coordinate system
     *
     * @param  margin double, when calculating the angles, the returned values are moved inward from the actual goal positions by this distance, because the ball will be blocked by the goalposts at this angle. The larger this value, the higher the shooting accuracy, but the longer it takes to adjust the angle.
     *
     * @return vector<double> The returned vector contains vec[0] as the left goalpost and vec[1] as the right goalpost. Note that left and right are based on the opponent's direction as the front.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    double calcKickDir(double goalPostMargin = 0.3);

    // type: kick: dribble, shoot: rl kick
    bool isAngleGood(double goalPostMargin = 0.3, string type = "kick");

   
    bool isBallOut(double locCompareDist = 3.0, double lineCompareDist = 1.0);

    
    void updateBallOut();

    
    double distToBorder();

    bool isDefensing();

    bool isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio = 0.8, double yRatio = 0.8);

    void calibrateOdom(double x, double y, double theta);

    double msecsSince(rclcpp::Time time);

    bool isFreekickStartPlacing();

    rclcpp::Time timePointFromHeader(std_msgs::msg::Header header);


    void updateCostToKick();

    /**
     * @brief Publish visualization markers (robot position, ball position, field lines, mark points)
     */
    void publishVisualizationMarkers();

    /**
     * @brief Publish odom to map TF transform
     */
    void publishOdomToMapTF();

    /**
     * @brief Publish field dimensions as static messages
     */
    void publishFieldDimensions();

    /**
     * @brief Publish robot position (field coordinate system)
     */
    void publishRobotPose();

    /**
     * @brief Publish ball position (field coordinate system)
     */
    void publishBallPosition();

    /**
     * @brief Publish teammates' positions (field coordinate system)
     */
    void publishTeammatesPoses();

    void pubKickMsg();

    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------

    void joystickCallback(const booster_interface::msg::RemoteControllerState &msg);

    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);

    void detectionsCallback(const vision_interface::msg::Detections &msg);

    void fieldLineCallback(const vision_interface::msg::LineSegments &msg);

    void imageCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void depthCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    void compressedDepthImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    void processDepthImage(const cv::Mat &depthFloat, int width, int height, const std_msgs::msg::Header &header);

    void odometerCallback(const booster_interface::msg::Odometer &msg);

    void lowStateCallback(const booster_interface::msg::LowState &msg);

    void headPoseCallback(const geometry_msgs::msg::Pose &msg);

    void recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg);

    void updateRelativePos(GameObject &obj);

    void updateFieldPos(GameObject &obj);

    /**
     * @brief Calculate the collision distance
     * 
     * @param angle double, target angle
     * 
     * @return double, collision distance
     */
    double distToObstacle(double angle);

    vector<double> findSafeDirections(double startAngle, double safeDist, double step=deg2rad(10));

    double calcAvoidDir(double startAngle, double safeDist);


    /**
     * agent related
     */
    void agentCommandCallback(const std_msgs::msg::String::SharedPtr msg);

private:
    void loadConfig();

    void updateBallMemory();

    void updateRobotMemory();

    void updateObstacleMemory();

    void updateKickoffMemory();

    void updateMemory();

    void handleCooperation();

    // ------------------------------------------------------ Vision Processing ------------------------------------------------------

    int markCntOnFieldLine(const string MarkType, const FieldLine line, const double margin = 0.2);

    int goalpostCntOnFieldLine(const FieldLine line, const double margin = 0.2);

    bool isBallOnFieldLine(const FieldLine line, const double margin = 0.3); 

    void identifyFieldLine(FieldLine &line);

    void identifyMarking(GameObject& marking);

    void identifyGoalpost(GameObject& goalpost);

    void updateLinePosToField(FieldLine &line);

    vector<FieldLine> processFieldLines(vector<FieldLine> &fieldLines);

    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    void detectProcessBalls(const vector<GameObject> &ballObjs);

    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    void detectProcessRobots(const vector<GameObject> &robotObjs);

    void detectProcessGoalposts(const vector<GameObject> &goalpostObjs);

    void detectProcessVisionBox(const vision_interface::msg::Detections &msg);
    
    void logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid, vector<std::array<float, 3>> &points);

    void logDebugInfo();


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<vision_interface::msg::LineSegments>::SharedPtr subFieldLine;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr imageCameraInfoSubscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depthCameraInfoSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressedDepthImageSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recoveryStateSubscription;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVisualizationMarkers;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubFieldDimensions;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubRobotPose;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pubBallPosition;
    rclcpp::Publisher<brain::msg::Kick>::SharedPtr pubKickBall;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubTeammatesPoses;

    // ------------------------------------------------------ debug logs related ------------------------------------------------------
    void logLags();
    void logStatusToConsole();
    string getComLogString(); 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ------------------------------------------------------ agent related ------------------------------------------------------
    // In agent mode, subscribe to parameter changes and handle callbacks
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> team_id_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> player_role_handle_;
};
