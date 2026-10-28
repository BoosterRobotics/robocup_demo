#pragma once

#include <iostream>
#include <string>
#include <rerun.hpp>

#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"
#include "booster_internal/robot/b1/b1_loco_internal_api.hpp"

using namespace std;

class Brain; // 类相互依赖，向前声明


/**
 * RobotClient 类，调用 RobotSDK 操控机器人的操作都放在这里
 * 因为目前的代码里依赖 brain 里相关的一些东西，现在设计成跟 brain 相互依赖
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief 移动机器人的头部
     *
     * @param pitch
     * @param yaw
     *
     * @return int , 0 表示执行成功
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief 设置机器人的移动速度
     * 
     * @param x double, 向前(m/s)
     * @param y double, 向左(m/s)
     * @param theta double, 逆时针转动角度(rad/s)
     * @param applyMinX, applyMinY, applyMinTheta bool 当速度指令过小时, 是否调整指令大小以防止不响应.
     * 
     * @return int , 0 表示执行成功
     * 
    */
    int setVelocity(double x, double y, double theta, bool applyMinX=true, bool applyMinY=true, bool applyMinTheta=true);

    // 在不转向的情况下, 以某个角度和速度移动
    int crabWalk(double angle, double speed);

    /**
     * @brief 以速度模式走向球场坐标系中的某个 Pose, 注意最后朝向也要达到
     * 
     * @param tx, ty, ttheta double, 目标的 Pose, Field 坐标系
     * @param longRangeThreshold double, 距离超过这个值时, 优先转向目标点走过去, 而不是直接调整位置
     * @param turnThreshold double, 目标点所在的方向(注意不是最终朝向 ttheta) 与目前的角度相差大于这个值时, 先转身朝向目标
     * @param vxLimit, vyLimit, vthetaLimit double, 各方向速度的上限, m/s, rad/s
     * @param xTolerance, yTolerance, thetaTolerance double, 判断已经到达目标点的容差
     * @param avoidObstacle bool, 行进过程中是否避障
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief 新版本 以速度模式走向球场坐标系中的某个 Pose, 注意最后朝向也要达到   
     * 
     * @param tx, ty, ttheta double, 目标的 Pose, Field 坐标系
     * @param longRangeThreshold double, 距离超过这个值时, 优先转向目标点走过去, 而不是直接调整位置
     * @param turnThreshold double, 目标点所在的方向(注意不是最终朝向 ttheta) 与目前的角度相差大于这个值时, 先转身朝向目标
     * @param vxLimit, vyLimit, vthetaLimit double, 各方向速度的上限, m/s, rad/s
     * @param xTolerance, yTolerance, thetaTolerance double, 判断已经到达目标点的容差
     * @param avoidObstacle bool, 行进过程中是否避障
     * 
     * @return int 运控命令返回值, 0 代表成功
     */

    int moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);
    /**
     * @brief 短距离魔改版本，以速度模式走向球场坐标系中的某个 Pose, 注意最后朝向也要达到   
     * 
     * @param tx, ty, ttheta double, 目标的 Pose, Field 坐标系
     * @param longRangeThreshold double, 距离超过这个值时, 优先转向目标点走过去, 而不是直接调整位置
     * @param turnThreshold double, 目标点所在的方向(注意不是最终朝向 ttheta) 与目前的角度相差大于这个值时, 先转身朝向目标
     * @param vxLimit, vyLimit, vthetaLimit double, 各方向速度的上限, m/s, rad/s
     * @param xTolerance, yTolerance, thetaTolerance double, 判断已经到达目标点的容差
     * @param avoidObstacle bool, 行进过程中是否避障
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief 挥手
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief 起身
     */
    int standUp();

    /**
     * @brief 大脚踢球
     */
    int fancyKickBall(double kick_speed = 1.0, double kick_dir = 0.0, bool cancel = false);

    /**
     * @brief 踢球
     */
    int kickBall(double kick_speed = 1.0, double kick_dir = 0.0, bool cancel = false);

    /**
     * @brief 恢复行走模式
     */
    int walkMode();

    /**
     * @brief 切换到rl结合视觉的踢球
     */
    int RLVisionKick(bool start = true);

    /**
     * @brief 切换到robocup步态
     */
    int robocupWalk();

    /**
     * @brief Switch robocup mode (kSoccer) and exit VisualKick(false)
     */
     int changeRobocupMode();
     
    /**
     * @brief 进阻尼
     */
    int enterDamping();

    // 计算按某个某个速度行进, 多少毫秒会撞上障碍物
    double msecsToCollide(double vx, double vy, double vtheta, double maxTime=10000);

    // 估算目前是否是静止站立状态
    bool isStandingStill(double timeBuffer = 1000);

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx, _vy, _vtheta;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;
};
