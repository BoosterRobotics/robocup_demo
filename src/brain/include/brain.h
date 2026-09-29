#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <limits>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_interface/msg/detections.hpp>
#include <vision_interface/msg/line_segments.hpp>
#include <vision_interface/msg/cal_param.hpp>
#include <vision_interface/msg/segmentation_result.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <booster/robot/b1/b1_loco_api.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "booster_interface/msg/odometer.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/raw_bytes_msg.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"
#include "booster_msgs/msg/rpc_resp_msg.hpp"

#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>
#include "brain/msg/kick.hpp"

#include "brain_config.h"
#include "brain_data.h"
#include "robot_path_planner_policy.h"
#include "brain_log.h"
#include "obstacle_avoidance_logger.h"
#include "odom_diagnostic_logger.h"
#include "brain_tree.h"
#include "brain_communication.h"
#include "locator.h"
#include "robot_client.h"


using namespace std;

/**
 * Core Brain class. It inherits enable_shared_from_this so BrainTree can receive a shared pointer.
 * State belongs in the appropriate child object rather than directly in Brain:
 * static configuration in config and dynamic runtime state in data.
 * TODO:
 * The behavior-tree blackboard also stores some state that may duplicate BrainData.
 * Behavior-tree nodes receive a Brain pointer and access brain->config and brain->data directly.
 */
class Brain : public rclcpp::Node
{
public:
    // Static runtime configuration.
    std::shared_ptr<BrainConfig> config;
    // Rerun logging operations.
    std::shared_ptr<BrainLog> log;
    // Dynamic Brain runtime state.
    std::shared_ptr<BrainData> data;
    // Robot operations.
    std::shared_ptr<RobotClient> client;
    // Localization engine.
    std::shared_ptr<Locator> locator;
    // Ball-position predictor.

    // Behavior-tree operations.
    std::shared_ptr<BrainTree> tree;
    // Team and GameController communication.
    std::shared_ptr<BrainCommunication> communication;

    // Construct a ROS 2 node with nodeName.
    Brain();

    ~Brain();

    // Initialize once from main; throw on invalid startup state to terminate immediately.
    void init();

    // Called from the ROS 2 loop.
    void tick();

    // Process special states such as kickoffs and free kicks.
    void handleSpecialStates();

    // Freeze and synchronize the free-kick plan after possession and formation updates.
    void handleFreeKickPlan();

    bool isOwnFreeKickExecutionActive() const;
    bool isPlannedFreeKickActor() const;
    void markFreeKickKickStarted();
    void markFreeKickKickCompleted();
    void cancelFreeKickKickAttempt();
    void logFreeKickPlanEvent(
        const char *reason, const FreeKickPlanState &plan) const;

    // Atomically commit the outbound team-communication snapshot from the decision thread.
    void refreshTeamCommunicationSnapshot();

    // Return a GameController response snapshot consistent with decision and perception state.
    RoboCupGameControlReturnData gameControllerReturnSnapshot();

    /**
     * @brief Calculate field-frame angles from the ball to both opponent goalposts.
     *
     * @param margin Inset from each physical post; larger values improve safety but require more alignment time.
     *
     * @return Angles for the left and right posts, viewed facing the opponent goal.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    // Deprecated; use CalcKickDir. Calculate a shooting angle requiring minimal adjustment while respecting goalPostMargin.
    double calcKickDir(double goalPostMargin = 0.3);

    // Determine whether the current robot-to-ball angle can score while respecting goalPostMargin.
    // type: "kick" for a touch, "shoot" for an RL kick.
    bool isAngleGood(double goalPostMargin = 0.3, string type = "kick");
    bool isAngleGoodForDirectionalKick(double goalPostMargin = 0.3);

    // Return whether a powerful kick is safe from the current position.
    bool isPositionGoodForPowerShoot();

    // Evaluate the utility of kicking in direction dir.
    double kickValue(double dir);

    // Calculate urgency: 0 no opponent, 1 distant opponent, 2 nearby opponent.
    double threatLevel();

    // Return whether the ball is out of bounds.
    bool isBallOut(double locCompareDist = 3.0, double lineCompareDist = 1.0);

    // A ball is stable when its maximum displacement over msecs is within distThreshold and enough samples exist.
    bool isBallStable(double msecs = 500, double distThreshold = 0.5, int minDataCnt = 5);

    // Update the blackboard ball_is_out entry from current state.
    void updateBallOut();

    // Update the predicted ball trajectory.
    void updateBallPrediction();

    // robot's largest dist out of border line. negative value if not out of border
    double distToBorder();

    // Return whether the robot is in a defensive state, such as before an opponent free kick.
    bool isDefensing();

    // Return whether a bounding box is within the screen-center region.
    bool isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio = 0.8, double yRatio = 0.8);

    // Calibrate odometry from a known field-frame robot pose.
    void calibrateOdom(
        double x,
        double y,
        double theta,
        const std::string &source = "unspecified");

    double msecsSince(rclcpp::Time time);

    bool isFreekickStartPlacing();

    // Extract a timestamp from a message header.
    rclcpp::Time timePointFromHeader(std_msgs::msg::Header header);

    // Play a predefined sound; block new sounds for blockMsecs and optionally suppress consecutive duplicates.
    void playSound(string soundName, double blockMsecs = 1000, bool allowRepeat = false);

    // Use eSpeak for local text-to-speech; text must be English.
    void speak(string text, bool allowRepeat = false);

    // Internal test helper: publish the ball position and kick direction for motion-control kick testing.
    void pubKickMsg();

    // Adjust vision calibration parameters dynamically.
    void pubCalParamMsg(double pitch, double yaw, double z);

    bool isPrimaryStriker();

    // The goalkeeper competes for possession only when handling a nearby dangerous ball.
    bool canGoalkeeperClaimBall(
        bool ballDetected, double ballRange, double ballX, double ballY,
        double cost) const;

    // Calculate kick cost to select the controlling robot during team coordination.
    void updateCostToKick(
        const std::array<TMStatus, MAX_NUM_PLAYERS> &teamStatuses);

    // Maintain one goalkeeper based on penalties and restore roles when the original goalkeeper returns.
    void updatePlayerRoleForTeamAvailability(
        const std::array<TMStatus, MAX_NUM_PLAYERS> &teamStatuses);



    // ROS callbacks are public for executor registration.
    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------

    // Receive controller button state for manual debugging.
    void joystickCallback(const booster_interface::msg::RemoteControllerState &msg);

    // Receive and process GameController messages.
    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);

    // Process object-detection messages.
    void detectionsCallback(const vision_interface::msg::Detections &msg);

    // Process field-line detection messages.
    void fieldLineCallback(const vision_interface::msg::LineSegments &msg);

    // Process vision segmentation messages.
    void segmentationResultCallback(const vision_interface::msg::SegmentationResult &msg);

    // Process camera images for logging only.
    void imageCallback(const sensor_msgs::msg::Image &msg);

    // Process depth images.
    void depthImageCallback(const sensor_msgs::msg::Image &msg);

    // Keep depth projection synchronized with Vision runtime intrinsics.
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo &msg);

    // T2 timestamped head pose; legacy /head_pose remains as a compatibility entry point.
    void headPoseStampedCallback(const geometry_msgs::msg::PoseStamped &msg);

    // VisionNode publishes the head/camera-link pose after applying the
    // automatic-calibration robot-model adapter; camera.extrin is applied here.
    void headPoseTfCallback(const geometry_msgs::msg::TransformStamped &msg);

    // Process odometry messages.
    void odometerCallback(const booster_interface::msg::Odometer &msg);

    // Process low-level state such as IMU and head joints for camera-pose calculation.
    void lowStateCallback(const booster_interface::msg::LowState &msg);

    // Update the head-pose buffer.
    // void updateHeadPosBuffer(double pitch, double yaw);

    // Determine head stability from its pose buffer; distance estimates are trusted only when stable.
    // bool isHeadStable(double msecSpan = 200);

    // Process head pose relative to the feet frame for debugging.
    void headPoseCallback(const geometry_msgs::msg::Pose &msg);

    // Process fall-recovery state.
    void recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg);

    // New SDK recovery sequence: kPrepare -> GetUp -> kWalking -> kSoccer.
    void beginRecoveryPrepareSequence(booster::robot::b1::GetUpVersion version);
    bool isRecoveryDampingMode() const;
    bool isRecoveryOperationalMode() const;
    bool isRecoveryModeTransitionActive();
    bool isRecoveryActive();
    bool isRecoveryLocalizationBlocked();
    bool isPostRecoveryHeadingRealignActive();
    bool beginPostRecoveryHeadingRealignAttempt();
    bool applyPostRecoveryHeadingRealignCandidate(
        const Pose2D &candidatePose, double residual);

    // Update relative position fields from a GameObject's posToField.
    void updateRelativePos(GameObject &obj);

    // Update a GameObject's posToField from its relative position.
    void updateFieldPos(GameObject &obj);

    /**
     * @brief Calculate collision distance.
     *
     * @param angle Target direction.
     *
     * @return Collision distance.
     */
    double distToObstacle(
        double angle,
        const GameObject *ignoredFallenRobot = nullptr);

    // Distance to depth obstacles not already represented by a recognized
    // robot. Used while intentionally leaving an overlapping robot footprint.
    double distToDepthObstacle(double angle);

    // Effective collision radius for a recognized robot.  The configured
    // posture radius is enlarged when the measured visual footprint is wider.
    double robotObstacleRadius(const GameObject &robot) const;

    // Radius of this robot's own footprint used to inflate external obstacles
    // into centre-line collision regions.
    double selfRobotCollisionRadius() const;

    // Ignore recognized robot centres closer than this distance. Very close
    // detections are commonly projections of this robot's own body.
    double robotObstacleMinimumDistance() const;

    // Use the measured footprint midpoint as the collision center when it is
    // valid.  The detector's bbox center can be biased by a leaning/fallen
    // robot, while the footprint endpoints are projected on the ground.
    Point robotObstacleCenterToRobot(const GameObject &robot) const;

    // Estimated obstacle velocity transformed from field to robot frame.
    Point robotObstacleVelocityToRobot(const GameObject &robot) const;

    // Allow one high-confidence, close visual detection to trigger the local
    // safety layer before it reaches the normal multi-frame planner gate.
    bool robotObstacleEmergency(
        const GameObject &robot,
        double pathAngle = std::numeric_limits<double>::quiet_NaN()) const;

    // Plan a short local route around confirmed visual robots. The depth
    // occupancy grid is intentionally handled separately by distToObstacle.
    robot_path_planner_policy::PathPlan planRobotPath(
        double goalX,
        double goalY,
        double extraClearance = 0.12,
        double previousSide = 0.0,
        bool preferPreviousSide = false,
        bool hardLockPreviousSide = false) const;

    // Test whether a confirmed fallen-robot region intersects a robot-frame travel direction.
    bool findFallenRobotOnPath(
        double pathAngle,
        GameObject &nearestRobot,
        double *entryDistance = nullptr,
        const GameObject *ignoredRobot = nullptr);

    // Test whether a confirmed fallen robot enters the VisualKick forward protection sector.
    bool isFallenRobotVisualKickExitEnabled() const;
    bool hasFallenRobotInVisualKickZone(GameObject *nearestRobot = nullptr);

    /**
     * @brief Check for obstacles within a forward angular sector and distance.
     *
     * @param angle Target direction.
     *
     * @return Collision distance.
     */
    bool isFrontRangeClear(double startAngle, double endAngle, double safeDist = 3.0, double step = deg2rad(2));

    vector<double> findSafeDirections(double startAngle, double safeDist, double step=deg2rad(5));

    double calcAvoidDir(double startAngle, double safeDist);

    // Enrich and persist one obstacle-avoidance decision. The logger itself is
    // rate-limited and rotating, so callers may submit every control cycle.
    void logObstacleAvoidance(ObstacleAvoidanceLogRecord record);


private:
    void loadConfig();
    void updateFallDownState(const booster_interface::msg::LowState &msg);
    bool isFallen(double roll, double pitch) const;
    BrainData::FallDownSide getFallDownSide(double pitch) const;
    void setupRecoveryModeMonitor();
    void sendModeQueryIfNeeded(const rclcpp::Time &now, bool force = false);
    void locoApiResponseCallback(const booster_msgs::msg::RpcRespMsg &msg);
    void advanceRecoveryModeTransition(const rclcpp::Time &now);
    bool hasRecoveryModeSince(booster::robot::RobotMode mode, const rclcpp::Time &since) const;
    void beginRecoverySoccerTransition(bool waitForWalking = true);
    void cancelRecoveryModeTransition();

    // When the ball is not visible, update its relative pose from the remembered field position and odometry.
    void updateBallMemory();

    // Maintain remembered robots, removing entries after they have not been observed for a while.
    void updateRobotMemory();

    // Maintain remembered obstacles.
    void updateObstacleMemory();

    // Process kickoff logic.
    void updateKickoffMemory();

    // Maintain perception memory, expiring stale ball data and updating relative pose when the ball is hidden.
    void updateMemory();

    // Process multi-robot coordination.
    void handleCooperation();

    // Freeze the cost-elected owner for an own centre kickoff.
    void handleKickoffPlan();

    // ------------------------------------------------------ Vision processing ------------------------------------------------------

    int markCntOnFieldLine(const string MarkType, const FieldLine line, const double margin = 0.2);

    int goalpostCntOnFieldLine(const FieldLine line, const double margin = 0.2);

    bool isBallOnFieldLine(const FieldLine line, const double margin = 0.3);

    void identifyFieldLine(FieldLine &line);

    void identifyMarking(GameObject& marking);

    void identifyGoalpost(GameObject& goalpost);

    // Calculate field-line positions in the field frame.
    void updateLinePosToField(FieldLine &line);

    // Process a detected field line.
    vector<FieldLine> processFieldLines(vector<FieldLine> &fieldLines);

    // Enrich a detection with field-frame position, robot-relative angles, and other derived data.
    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    // Apply ball-specific detection processing.
    void detectProcessBalls(const vector<GameObject> &ballObjs);

    // Apply field-marking-specific detection processing.
    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    // Apply robot-specific detection processing.
    void detectProcessRobots(const vector<GameObject> &robotObjs);

    void detectProcessGoalposts(const vector<GameObject> &goalpostObjs);

    // Process field-of-view information from detection.
    void detectProcessVisionBox(const vision_interface::msg::Detections &msg);

    // log visionbox to rerun
    void logVisionBox(const rclcpp::Time &timePoint);

    // log detection to rerun
    void logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox = true);

    void logMemRobots();

    void logObstacles();

    void logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid, vector<rerun::Vec3D> &points);

    // Write important diagnostics to the Rerun log.
    void logDebugInfo();

    void updateLogFile();
    void initOdomDiagnosticLog();
    void initObstacleAvoidanceLog();
    double distToObstacleImpl(
        double angle,
        const GameObject *ignoredFallenRobot,
        bool includeRecognizedRobots);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<vision_interface::msg::LineSegments>::SharedPtr subFieldLine;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscription;
    rclcpp::Subscription<vision_interface::msg::SegmentationResult>::SharedPtr segmentationResultSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr headPoseStampedSubscription;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr headPoseTfSubscription;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recoveryStateSubscription;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr modeQueryPublisher_;
    rclcpp::Subscription<booster_msgs::msg::RpcRespMsg>::SharedPtr locoApiResponseSubscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSoundPlay;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSpeak;
    rclcpp::Publisher<brain::msg::Kick>::SharedPtr pubKickBall;
    rclcpp::Publisher<vision_interface::msg::CalParam>::SharedPtr pubCalParam;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------ Diagnostic logging ------------------------------------------------------
    void logObstacleDistance();
    void logLags();
    void statusReport();
    void logStatusToConsole();
    string getComLogString(); // Format team communication for the console and brain.log.
    void playSoundForFun();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<OdomDiagnosticLogger> odomDiagnosticLog_;
    std::unique_ptr<ObstacleAvoidanceLogger> obstacleAvoidanceLog_;
    uint64_t odomCallbackSequence_ = 0;
    uint64_t odomTransformRevision_ = 0;

    bool isRecoveryLocalizationHoldRequired();
    void updateRecoveryLocalizationPreHold(double roll, double pitch);
    void beginPostGetupOdomSettling(const rclcpp::Time &now);
    void updatePostGetupOdomSettling(const rclcpp::Time &now);
    void updateRecoveryLocalizationHold();
    void reanchorPoseToCurrentOdom(const Pose2D &pose);
    bool updateOdomThetaAlignment(
        const Pose2D &rawOdomPose,
        const VelocityCommandSnapshot &velocity,
        const rclcpp::Time &callbackTime,
        std::chrono::steady_clock::time_point callbackSteadyTime);
    void applyOdomThetaAlignment(
        const Pose2D &fieldPoseBeforeAlignment,
        const rclcpp::Time &callbackTime,
        std::chrono::steady_clock::time_point callbackSteadyTime);
    void rememberOdomThetaAlignmentAnchor(
        const Pose2D &fieldPose,
        const std::string &source);
    void updateCameraToRobot(const geometry_msgs::msg::Pose &msg);

    double odomThetaAlignmentOffset_ = 0.0;
    bool odomThetaAlignmentLocked_ = false;
    bool hasOdomThetaAlignmentSample_ = false;
    Pose2D odomThetaAlignmentPreviousRawPose_ = {};
    std::chrono::steady_clock::time_point odomThetaAlignmentLastSampleTime_;
    double odomThetaAlignmentCosSum_ = 0.0;
    double odomThetaAlignmentSinSum_ = 0.0;
    double odomThetaAlignmentDistance_ = 0.0;
    double odomThetaAlignmentConcentration_ = 0.0;
    bool hasOdomThetaAlignmentAnchor_ = false;
    Pose2D odomThetaAlignmentAnchorRawPose_ = {};
    Pose2D odomThetaAlignmentAnchorFieldPose_ = {};
    std::string odomThetaAlignmentAnchorSource_;

    mutable std::mutex recoveryModeMutex_;
    mutable std::recursive_mutex recoveryStateMutex_;
    struct TimedCameraTransform {
        rclcpp::Time stamp;
        Eigen::Matrix4d cameraToRobot = Eigen::Matrix4d::Identity();
    };
    bool cameraTransformValid_ = false;
    std::atomic<bool> timestampedHeadPoseSeen_{false};
    std::atomic<bool> visionCameraTransformSeen_{false};
    // Set when the configured depth stride cannot produce enough samples to
    // satisfy the occupancy threshold. The main tick uses it to avoid
    // resurrecting a stale obstacle after the depth callback clears it.
    std::atomic<bool> depthSamplingInvalid_{false};
    std::atomic<std::size_t> depthGridOccupiedCells_{0};
    std::atomic<int> depthGridMaxOccupancy_{0};
    std::atomic<std::size_t> depthGridConfirmedCells_{0};
    std::atomic<std::size_t> rawRobotDetectionCount_{0};
    std::atomic<std::size_t> mergedRobotDetectionCount_{0};
    std::deque<TimedCameraTransform> cameraTransformBuffer_;
    std::mutex recoveryTransitionMutex_;
    std::string pendingModeQueryUuid_;
    rclcpp::Time pendingModeQueryTime_;
    rclcpp::Time lastModeQueryTime_;
    booster::robot::RobotMode sdkRobotMode_ = booster::robot::RobotMode::kUnknown;
    int sdkRobotModeStatus_ = -1;
    rclcpp::Time sdkRobotModeTime_;

    enum class RecoveryModeTransitionPhase {
        Idle,
        WaitingForPrepare,
        PrepareDwell,
        WaitingForWalking,
        WaitingForSoccer
    };
    RecoveryModeTransitionPhase recoveryModeTransitionPhase_ = RecoveryModeTransitionPhase::Idle;
    booster::robot::b1::GetUpVersion recoveryGetUpVersion_ =
        booster::robot::b1::GetUpVersion::kV2;
    rclcpp::Time recoveryModeTransitionStarted_;
    rclcpp::Time recoveryLastModeQueryTime_;
    rclcpp::Time recoveryLastSoccerRequestTime_;
    bool recoverySoccerTimeoutLogged_ = false;
    bool recoveryLocalizationPreHoldActive_ = false;
    rclcpp::Time recoveryLocalizationPreHoldReleaseCandidateAt_;
    int64_t recoveryUprightStableSinceUs_ = 0;
    int64_t recoveryGetUpPostureStableSinceUs_ = 0;
    bool recoveryPostureConfirmed_ = false;
    bool recoveryLocalizationHoldActive_ = false;
    Pose2D recoveryLocalizationHoldPose_ = {};
    Pose2D recoveryLocalizationHoldStartOdomPose_ = {};
    double recoveryLocalizationHoldStartFieldTheta_ = 0.0;
    rclcpp::Time recoveryLocalizationReleasedAt_;
    rclcpp::Time recoveryLocalizationGuardStartedAt_;
    bool recoveryLocalizationHoldSawRecovery_ = false;
    bool recoveryLocalizationHoldPoseTrusted_ = false;
    struct RecoveryOdomSample {
        rclcpp::Time stamp;
        Pose2D pose = {};
    };
    std::deque<RecoveryOdomSample> recoveryPostGetupOdomSamples_;
    bool recoveryPostGetupOdomSettleActive_ = false;
    bool recoveryPostGetupOdomSettleTimedOut_ = false;
    rclcpp::Time recoveryPostGetupOdomSettleStartedAt_;
    std::string recoveryPostGetupOdomSettleReason_ = "inactive";
    double recoveryPostGetupOdomSettleMinimumMs_ = 3500.0;
    double recoveryPostGetupOdomSettleWindowMs_ = 750.0;
    double recoveryPostGetupOdomSettleMaximumMs_ = 7500.0;
    double recoveryPostGetupOdomSettleMaxXyRangeM_ = 0.015;
    double recoveryPostGetupOdomSettleMaxThetaRangeDeg_ = 0.75;
    double recoveryPostGetupOdomSettleMaxTiltRad_ = 0.30;
    double recoveryPostGetupOdomSettleMaxGyroRadPerSec_ = 0.35;
    double recoveryLatestImuRoll_ = 0.0;
    double recoveryLatestImuPitch_ = 0.0;
    double recoveryLatestGyroMagnitude_ = 0.0;
    bool recoveryLatestImuValid_ = false;
    double lastAvoidanceDirection_ = 0.0;
    rclcpp::Time lastAvoidanceDirectionAt_;
    int recoveryHeadingRealignAttemptsRemaining_ = 0;
    int recoveryHeadingRealignCandidateConfirmations_ = 0;
    double recoveryHeadingRealignCandidateDelta_ = 0.0;
    bool recoveryHeadingRealignAttemptInFlight_ = false;
    rclcpp::Time recoveryHeadingRealignLastAttemptAt_;
};
