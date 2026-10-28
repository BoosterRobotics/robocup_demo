#pragma once

#include <string>
#include <cstdint>
#include <mutex>
#include <tuple>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"


#include "locator.h"
#include "types.h"
#include "RoboCupGameControlData.h"
#include "buffer.h"

using namespace std;

/**
 * BrainData 类，记录 Brain 在决策中需要用到的所在数据，区分于 BrainConfig，这里是运行时数据（动态）
 * 针对数据处理的一些工具函数，也可以放到这里来
 */
class BrainData
{
public:
    BrainData();
    /* ------------------------------------ 球赛相关状态量 ------------------------------------ */

    int score = 0;
    int oppoScore = 0;
    int penalty[HL_MAX_NUM_PLAYERS]{}; // 我方机器人 penalty 状态
    int oppoPenalty[HL_MAX_NUM_PLAYERS]{}; // 对方所有机器人 penalty 状态
    bool isKickingOff = false; // 是否在刚刚发球的状态, 持续 10 秒
    rclcpp::Time kickoffStartTime; // 发球开始的时间
    bool isOpponentKickingOff = false; // 对方刚刚开球的状态, 持续 10 秒
    rclcpp::Time opponentKickoffStartTime;
    bool isFreekickKickingOff = false; // 是否在任意球发球的开始状态, 持续 10 秒
    rclcpp::Time freekickKickoffStartTime; // 任意球发球开始的时间
    int liveCount = 0; // 已方存活机器人数量
    int oppoLiveCount = 0; // 对方存活机器人数量
    string realGameSubState; // 记录当前处于任意球, 门球等特殊状态. 因为 bb 上的 gc_game_sub_type 做了简化处理, 都看到任意球处理, 所以此处单独记录一下, 以便需要知道具体状态时使用.

    /* ------------------------------------ 数据记录 ------------------------------------ */
    
    // 身体位置 & 速度指令
    Pose2D robotPoseToOdom;  // 机器人在 Odom 坐标系中的 Pose, 通过 odomCallback 更新数据
    Pose2D odomToField;      // Odom 坐标系原点在 Field 坐标系中的位置和方向.  可通过已知位置进行校准, 例如上场时根据上场点校准
    Pose2D robotPoseToField; // 机器人当前在球场坐标系中的位置和方向. 球场中心为原点, x 轴指向对方球门(前方), y 轴指向左方. 逆时针为 theta 正方向.

    // 头部位置 通过 lowStateCallback 更新数据
    double headPitch = 0.0; // 当前头部的 pitch, 单位 rad, 0 点为水平向前, 向下为正.
    double headYaw = 0.0;   // 当前头部的 yaw, 单位 rad, 0 点为向前, 向左为正.
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity();  // 相机到机器人坐标系的变换矩阵，初始化为单位矩阵

    // vector<TimestampedData> headPosBuffer = {};
    // double headPosBufferMsecs = 2000; // 头部位置的 buffer 时间长度, 单位 ms

    // 足球
    bool ballDetected = false;    // 当前摄像头是否识别到了足球
    GameObject ball{};              // 球的各项信息记录, 包含位置, boundingbox 等
    GameObject tmBall{};            // 队友传来的球的信息
    double robotBallAngleToField = 0.0; // 机器人到球的向量, 在球场坐标系中与 X 轴的夹角, (-PI,PI]
    bool lose_ball = false;       // 视觉丢球状态, 用于 visual kick 的退出判定
    vector<array<double, 2>> predictedBallPos; // 预测的球的位置, 单位 m, 相对于球场坐标系. 第一维为 x, 第二维为 y
    rclcpp::Time ballPosPredictTime; // 上一次进行预测的球的位置的时间戳
    double ballEstVelX = 0.0;
    double ballEstVelY = 0.0;
    double ballEstSpeed = 0.0;
    bool ballEstIsRolling = false;
    Point2D ballEstEndPos{}; // 速度相关摩擦模型下的预计停球点
    bool reliableBallPredict = false;
    bool ballWillBreach = false; // 是否会从机器人身边穿过
    Point2D ballBreachPoint{}; // 如果会穿过, 穿过的时刻所在的位置
    rclcpp::Time ballBreachTime; // 预计穿过的时间点
    Point2D ballInterceptPoint{}; // 最佳拦截点
    rclcpp::Time ballInterceptTime; // 球经过拦截点的时间

    // 机器人
    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }

    // 球门柱
    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }

    // 识别到的球场上的标记交叉点
    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    // 识别到的球场上的标线
    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }

    // 识别到的球场上的障碍物
    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }

    /* ------------------------------------ Buffered Data ------------------------------------ */
    Buffer<sensor_msgs::msg::Image> imageBuffer{50};        // 图像缓冲区
    Buffer<sensor_msgs::msg::Image> depthImageBuffer{50};   // 深度图像缓冲区  
    Buffer<GameObject> detectionBuffer{50};                  // 检测对象缓冲区
    Buffer<booster_interface::msg::Odometer> odomBuffer{500}; // 里程计缓冲区，约保存 5 秒(100Hz)
    

    // 运动规划
    double kickDir = 0.; // 在决策中规划的踢球方向, field 坐标系
    string kickType = "shoot"; // "shoot" | "cross" | "block"
    bool isDirectShoot = false; // 在直接任意球开球的时候, 这个值会为 true; 执行了踢球动作或超过规定时间, 这个值会被 handleSpecialStates 重置为 false


    // 双机配合, tm: teammate
    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]{}; // 队友间通讯传达的状态, 数组的 idx 为 player_id - 1;
    int tmCmdId = 0; // 全队最新的 cmdId, 每个队员发布新指令时, 此值都加 1;
    rclcpp::Time tmLastCmdChangeTime; // 最后一次收到或发送新指令的时间
    int tmMyCmd = 0; // 我最后发出的指令;
    int tmMyCmdId = 0; // 我最后发出的指令的 ID; 
    int tmReceivedCmd = 0; // 收到队友的指令. 当 tmCmdId 与收到的 cmdId 不同时, 这个值会被更新为收到的 Cmd, 执行后会回到 0;
    bool tmImLead = true; // 当前我是否在控球.
    bool tmImAlive = true; // 当前我是否在上场中. 以裁判机为准.
    double tmMyCost = 0.; // 我接近球的成本, 用于多机配合. 基本上 cost 相当于我踢到球需要花的秒数.
    int tmMyCostRank = 0; // 我接近球的成本排名, 用于多机配合. 基本上 cost 相当于我踢到球需要花的秒数.
    int tmMyStrikerCostRank = 0; // 仅在前锋中按 cost 排名，用于我方普通开球阶段的攻守分组.
    int kickoffStrikerGroupRank = -1; // 开球阶段锁定的前锋分组排名(0/1 对方半场, 2/3 己方半场).
    bool kickoffStrikerGroupLatched = false;
    int kickoffBallHalf = 0; // 开球阶段球所在半区: -1 己方, 1 对方, 0 尚未确定.
    int myStrikerIDRank = 0; // 我的 ID 在前锋中的排名, 用于多机配合. 
    bool tmImInVisualKick = false; // 自己是否处于 visual kick 模式
    bool shouldExitRLVisionKick = false; // 是否需要主动退出 visual kick 模式
    string goalieVisualKickSubState = "none"; // none | block_kick | intercept_kick
    bool goalieVisualKickTargetValid = false;
    Point goalieVisualKickTargetField{0.0, 0.0, 0.0};
    int goalkeeperPlayerId = 0; // 裁判机指定的守门员，0 表示未知
    int activeGoaliePlayerId = 0; // 当前动态选举/交接后实际承担守门职责的球员

    // 通讯相关
    int discoveryMsgId = 0;
    rclcpp::Time discoveryMsgTime;
    int sendId = 0;
    rclcpp::Time sendTime;
    int receiveId[HL_MAX_NUM_PLAYERS]{};
    rclcpp::Time receiveTime[HL_MAX_NUM_PLAYERS]; 
    string tmIP;
    
    // 起身
    RobotRecoveryState recoveryState = RobotRecoveryState::IS_READY;
    bool isRecoveryAvailable = false; // 是否可以起身
    int currentRobotModeIndex = -1;
    int recoveryPerformedRetryCount = 0; // 记录起身次数
    bool recoveryPerformed = false;

    // 用于调试
    rclcpp::Time timeLastDet; // 上次收到 Detection 消息的时间戳时间
    rclcpp::Time timeLastOdom; // 上次收到里程计消息的时间戳
    uint64_t odomUpdateCount = 0; // 里程计回调累计次数，用于诊断断流
    bool camConnected = false; // 摄像头是否连接, 第一次收到图像消息时设置为 true
    rclcpp::Time timeLastLineDet; // 上次收到 FieldLine Detection 消息的时间戳时间
    rclcpp::Time lastSuccessfulLocalizeTime;
    rclcpp::Time timeLastGamecontrolMsg; // 上次收到 Gamecontrol 消息的时间戳时间
    rclcpp::Time timeLastLogSave; // 上次日志记录到文件的时间, 用于防止日志太大, 读的时候死机
    VisionBox visionBox;  
    rclcpp::Time lastTick; 


    /**
     * @brief 按类型获取 markings
     * 
     * @param type set<string>, 空 set 代表所有类型, 否则代表指定的类型 "LCross" | "TCross" | "XCross" | "PenaltyPoint"
     * 
     * @return vector<GameObject> 类型符合的 markings
     */
    vector<GameObject> getMarkingsByType(set<string> types={});

    // 获取 locator 可用的 Markers vector
    vector<FieldMarker> getMarkersForLocator();

    // 将一个 Pose 从 robot 坐标系转到 field 坐标系
    Pose2D robot2field(const Pose2D &poseToRobot);

    // 将一个 Pose 从 field 坐标系转到 robot 坐标系
    Pose2D field2robot(const Pose2D &poseToField);

private:
    vector<GameObject> _robots = {}; 
    mutable std::mutex _robotsMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

};
