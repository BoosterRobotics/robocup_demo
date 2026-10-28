#pragma once

#include <string>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_interface/msg/detections.hpp>
#include <vision_interface/msg/line_segments.hpp>
#include <vision_interface/msg/cal_param.hpp>
#include <vision_interface/msg/segmentation_result.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
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
#include "brain_log.h"
#include "brain_tree.h"
#include "brain_communication.h"
#include "locator.h"
#include "robot_client.h"
#include "ball_state_estimator.h"


using namespace std;

/**
 * Brain 核心类，因为要传递智能指针给 BrainTree，所以需要继承自 enable_shared_from_this
 * 数据封闭到各个子对象中，不要直接存在在 Brain 类
 * 如果是静态的配置值，放到 config
 * 运行时的动态数据，放到 data
 * TODO:
 * BehaviorTree 的 blackboard 里也存了一些数据，看看是不是有些重复存储了的，可以考虑去掉,
 * 目前的设计里，brain 指针传到了 BehaviorTree 的结点里了，在那里直接访问 brain->config 和 brain->data
 */
class Brain : public rclcpp::Node
{
public:
    // BrainConfig 对象，主要包含运行时需要的配置值（静态）
    std::shared_ptr<BrainConfig> config;
    // BrainLog 对象，封装 rerun log 相关的操作
    std::shared_ptr<BrainLog> log;
    // BrainData 对象，Brain 所有运行时的值都放在这里
    std::shared_ptr<BrainData> data;
    // RobotClient 对象，包含所有对机器人的操作
    std::shared_ptr<RobotClient> client;
    // locator 对象
    std::shared_ptr<Locator> locator;
    // posProjector 对象，用于预测球的位置

    // BrainTree 对象，里面包含 BehaviorTree 相关的操作
    std::shared_ptr<BrainTree> tree;
    // Communication 对象，里面包含通信相关的操作，主要是双机通信和裁判机通信
    std::shared_ptr<BrainCommunication> communication;
    std::unique_ptr<ball_state_estimator::BallStateEstimator> ballStateEstimator;

    // 构造函数，接受 nodeName 创建 ros2 结点
    Brain();

    ~Brain();

    // 初始化操作，只需要在 main 函数中调用一次，初始化过程如不符合预期，可以抛异常直接中止程序
    void init();

    // 在 ros2 循环中调用
    void tick();

    // 处理特殊状态, 如发球状态, 任意球发球状态等
    void handleSpecialStates();

    /**
     * @brief 计算当前球到对方两个球门柱的向量角度, 球场坐标系
     *
     * @param  margin double, 计算角度时, 返回值比实际的球门的位置向内移动这个距离, 因为这个角度球会被门柱挡住. 此值越大, 射门准确性越高, 但调整角度花的时间也越长.
     *
     * @return vector<double> 返回值中 vec[0] 是左门柱, vec[1] 是右门柱. 注意左右都是以对方的方向为前方.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    // 弃用. use CalcKickDir Node instead. 根据球与机器人的位置, 计算踢向对方球门的角度, 使得机器人可以以最少的位置调整达到此角度. goalPostMargin 代表计算角度时, 将门柱周围多少距离让出来, 以防止方向撞到门柱
    double calcKickDir(double goalPostMargin = 0.3);

    // 计算当前机器人与球之间的角度, 如果射门, 是否可以成功. goalPostMargin 代表计算角度时, 将门柱周围多少距离让出来, 以防止方向撞到门柱;
    // type: kick: 趟球, shoot: rl 踢球
    bool isAngleGood(double goalPostMargin = 0.3, string type = "kick");
    bool isAngleGoodForDirectionalKick(double goalPostMargin = 0.3);

    // 当前位置是否可以大力开球
    bool isPositionGoodForPowerShoot();

    // 计算当前向 dir 方向踢球的价值的大小.
    double kickValue(double dir);

    // 计算当前的急迫度, 没看见敌人: 0, 看见敌人, 但距离远: 1, 看见敌人, 距离近: 2
    double threatLevel();

    // 判断当前球是否在界外
    bool isBallOut(double locCompareDist = 3.0, double lineCompareDist = 1.0);

    // 判断当前球的位置是否稳定, 即没有在高速运动中, 方式为在 msecs 毫秒内, 球的最大位移没有超过 distThreshold. 如果时间范围内的数据量小于 minDataCnt, 也认为不稳定
    bool isBallStable(double msecs = 500, double distThreshold = 0.5, int minDataCnt = 5);

    // 根据当前所有信息, 更新 bb 中的 ball_is_out entry
    void updateBallOut();

    // 更新球路的预测
    void updateBallPrediction();

    // robot's largest dist out of border line. negative value if not out of border
    double distToBorder();

    // 判断当前是否处于防守位置, 如对方开任意球之前
    bool isDefensing();

    // bounding box 是否位于屏幕的中心范围内
    bool isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio = 0.8, double yRatio = 0.8);

    // 根据真值校准 odom. 参数 x, y, theta 代表在球场坐标系的机器人的 Pose 真值
    void calibrateOdom(double x, double y, double theta, bool alignToVisionTime = false);

    double msecsSince(rclcpp::Time time);

    bool isFreekickStartPlacing();

    // 从 msg 的 header 中获取时间点
    rclcpp::Time timePointFromHeader(std_msgs::msg::Header header);

    // 播放 sound_play node 的预定义声音, soundName 为声音名, 在 blockMsecs 时间内, 不接受新的声音. allowRepeat 为 false 时, 如果上一次播放的声音与本次播放的相同, 则不重复播放
    void playSound(string soundName, double blockMsecs = 1000, bool allowRepeat = false);

    // 使用 espeak 进行本地 tts, 文本必须是英文
    void speak(string text, bool allowRepeat = false);

    // [临时内部使用] 发布球的位置, 以及踢球方向消息, 用于测试运控的新 kick 动作
    void pubKickMsg();

    // 用于动态调整 vision 的校准参数
    void pubCalParamMsg(double pitch, double yaw, double z);

    bool isPrimaryStriker();

    // 在通信健康时按距离己方球门最近选守门员；通信不足时使用稳定回退规则。
    int selectGoaliePlayerId(bool *communicationHealthy = nullptr,
                             bool *stableEnough = nullptr,
                             bool *usedFallback = nullptr);

    // 计算当前踢球的成本, 用于多机配合时决定由谁来控制球
    void updateCostToKick();



    // 回调函数放到 public 下去
    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------

    // 需要使用手柄控制机器人进行调试时, 可以在这里获得手柄的按键状态.
    void joystickCallback(const booster_interface::msg::RemoteControllerState &msg);

    // 用于接收并处理裁判上位机的消息
    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);

    // 处理视觉识别消息
    void detectionsCallback(const vision_interface::msg::Detections &msg);

    // 处理视觉识别-球场标线识别消息
    void fieldLineCallback(const vision_interface::msg::LineSegments &msg);

    // 处理视觉识别-分割结果消息
    void segmentationResultCallback(const vision_interface::msg::SegmentationResult &msg);

    // 处理摄像头图像信息, 这里仅用于将图片记录入日志
    void imageCallback(const sensor_msgs::msg::Image &msg);

    // 处理深度图像信息
    void depthImageCallback(const sensor_msgs::msg::Image &msg);

    // 处理里程计消息
    void odometerCallback(const booster_interface::msg::Odometer &msg);

    // 处理底层状态信息, 如 imu 以及头部关节状态消息 (用以计算摄像头角度)
    void lowStateCallback(const booster_interface::msg::LowState &msg);

    // 更新头部位置的 buffer
    // void updateHeadPosBuffer(double pitch, double yaw);

    // 通过头部位置的 buffer 计算头部是否稳定 (稳定时才可以相信测距)
    // bool isHeadStable(double msecSpan = 200);

    // 处理头部相对于双脚坐标系的信息 (用以debug)
    void headPoseCallback(const geometry_msgs::msg::Pose &msg);

    // 处理跌到爬起状态信息
    void recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg);

    // 根据一个 GameObject 的 posToField 数据, 更新其它相对位置数据
    void updateRelativePos(GameObject &obj);

    // 根据一个 GameObject 的相对位置数据, 更新 obj 的 posToField 数据
    void updateFieldPos(GameObject &obj);

    /**
     * @brief 计算碰撞距离
     * 
     * @param angle double, 目标角度
     * 
     * @return double, 碰撞距离
     */
    double distToObstacle(double angle);

    /**
     * @brief 计算前方角度范围x米内是否有障碍物
     * 
     * @param angle double, 目标角度
     * 
     * @return double, 碰撞距离
     */
    bool isFrontRangeClear(double startAngle, double endAngle, double safeDist = 3.0, double step = deg2rad(2));

    vector<double> findSafeDirections(double startAngle, double safeDist, double step=deg2rad(10));

    double calcAvoidDir(double startAngle, double safeDist);


private:
    void loadConfig();

    // 看不见球时, 可以利用记忆中球在 Field 中的位置以及机器人 Odom 信息更新球的相对位置
    void updateBallMemory();

    // 处理记忆中的其它机器人的逻辑, 如一段时间看不到就从记忆中清除
    void updateRobotMemory();

    // 处理记忆中的障碍物
    void updateObstacleMemory();

    // 处理 kickoff 逻辑
    void updateKickoffMemory();

    // 处理记忆相关的逻辑, 例如多久看不到球时, 就可以认为记忆中的球的位置已经不可信了. 或没有看到球时, 根据记忆更新球相对于机器人的位置.
    void updateMemory();

    // 处理多机配合逻辑
    void handleCooperation();

    // 里程计与视觉/行为树回调共享 BrainData。tick 和回调跨线程运行，必须使用同一把锁。
    mutable std::recursive_mutex brainStateMutex_;

    void updateFallLocalizationHoldState(bool holdActive);
    bool isFallLocalizationHoldActive() const;
    bool isTrueFallRecoveryState(RobotRecoveryState state) const;
    void reanchorFrozenPoseToCurrentOdom(const Pose2D &targetPose);
    void storeRobotPoseToOdomSample(const rclcpp::Time &stamp, const Pose2D &pose);
    bool lookupRobotPoseToOdomAt(const rclcpp::Time &stamp, Pose2D &pose, double *ageMs = nullptr) const;

    struct OdomPoseSample
    {
        rclcpp::Time stamp;
        Pose2D pose = {};
    };
    mutable std::mutex odomPoseBufferMutex_;
    std::deque<OdomPoseSample> odomPoseBuffer_;

    // ------------------------------------------------------ 视觉处理 ------------------------------------------------------

    int markCntOnFieldLine(const string MarkType, const FieldLine line, const double margin = 0.2);

    int goalpostCntOnFieldLine(const FieldLine line, const double margin = 0.2);

    bool isBallOnFieldLine(const FieldLine line, const double margin = 0.3); 

    void identifyFieldLine(FieldLine &line);

    void identifyMarking(GameObject& marking);

    void identifyGoalpost(GameObject& goalpost);

    // 计算 fieldline 相对于球场坐标系的位置
    void updateLinePosToField(FieldLine &line);

    // 处理检测到的球场标线
    vector<FieldLine> processFieldLines(vector<FieldLine> &fieldLines);

    // 将 detection 中的消息进行加工丰富, 获得更完整的对象信息. 例如计算 Field 坐标系中的位置, 相对于机器人的角度等
    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    // 进一步处理检测到的足球
    void detectProcessBalls(const vector<GameObject> &ballObjs);

    // 进一步处理检测到的场地标志点
    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    // 进一步处理检测到的机器人信息
    void detectProcessRobots(const vector<GameObject> &robotObjs);

    void detectProcessGoalposts(const vector<GameObject> &goalpostObjs);

    // 处理检测返回的视野信息
    void detectProcessVisionBox(const vision_interface::msg::Detections &msg);

    // log visionbox to rerun
    void logVisionBox(const rclcpp::Time &timePoint);

    // log detection to rerun
    void logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox = true);

    void logMemRobots();

    void logObstacles();
    
    void logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid, vector<rerun::Vec3D> &points);

    // 输出重要的调试信息到 rerun log
    void logDebugInfo();

    void updateLogFile();
    void setupLowFrequencyDiagnostics();
    void sendModeQueryIfNeeded(const rclcpp::Time &now);
    void locoApiResponseCallback(const booster_msgs::msg::RpcRespMsg &msg);
    void logLowFrequencyDiagnostics(const rclcpp::Time &now);
    std::string makeLowFrequencyFieldAscii(
        const Pose2D &robotPose,
        const Point &ballField,
        bool ballKnown) const;
    std::string modeNameFromIndex(int mode) const;
    char headingChar(double theta) const;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<vision_interface::msg::LineSegments>::SharedPtr subFieldLine;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSubscription;
    rclcpp::Subscription<vision_interface::msg::SegmentationResult>::SharedPtr segmentationResultSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recoveryStateSubscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSoundPlay;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSpeak;
    rclcpp::Publisher<brain::msg::Kick>::SharedPtr pubKickBall;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr modeQueryPublisher_;
    rclcpp::Subscription<booster_msgs::msg::RpcRespMsg>::SharedPtr locoApiResponseSubscription_;
    rclcpp::Publisher<vision_interface::msg::CalParam>::SharedPtr pubCalParam;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------ 调试 log 相关 ------------------------------------------------------
    void logObstacleDistance();
    void logLags();
    void statusReport();
    void logStatusToConsole();
    string getComLogString(); // 将多机通讯信息打印到 console, 可以在 brain.log 中查看
    void playSoundForFun();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 倒地/起身期间里程计仍继续接收，但不允许它把场地坐标系拉跳。
    bool fallLocalizationHoldActive_ = false;
    Pose2D fallLocalizationHoldPose_ = {};
    Pose2D fallLocalizationHoldOdomToField_ = {};

    bool lowFrequencyStatusEnable_ = false;
    bool lowFrequencyStatusModeEnable_ = true;
    bool lowFrequencyStatusFieldAsciiEnable_ = true;
    double lowFrequencyStatusLogIntervalMs_ = 2000.0;
    double lowFrequencyStatusModeQueryIntervalMs_ = 2000.0;
    double lowFrequencyStatusModeQueryTimeoutMs_ = 1000.0;
    double lowFrequencyStatusFieldAsciiIntervalMs_ = 5000.0;
    double lowFrequencyStatusFieldAsciiCharAspect_ = 2.0;
    int lowFrequencyStatusFieldAsciiWidth_ = 45;
    mutable std::mutex lowFrequencyStatusMutex_;
    std::string pendingModeQueryUuid_;
    rclcpp::Time pendingModeQueryTime_;
    rclcpp::Time lastModeQueryTime_;
    rclcpp::Time lastLowFrequencyStatusLogTime_;
    rclcpp::Time lastFieldAsciiLogTime_;
    int sdkRobotMode_ = -1;
    int sdkRobotModeStatus_ = -1;
    rclcpp::Time sdkRobotModeTime_;
    std::string sdkRobotModeError_ = "no_response_yet";
    bool lowFrequencyStatusBallSeenOnce_ = false;
};
