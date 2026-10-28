#pragma once

#include <string>
#include <ostream>

#include "types.h"
#include "utils/math.h"
#include "RoboCupGameControlData.h"


using namespace std;

/**
 * 存储 Brain 需要的一些配置值，这些值应该是初始化就确认好了，在机器人决策过程中只读不改的
 * 需要在决策过程中变化的值，应该放到 BrainData 中
 * 注意：
 * 1、配置文件会从 config/config.yaml 中读取
 * 2、如果存在 config/config_local.yaml，将会覆盖 config/config.yaml 的值
 * 
 */
class BrainConfig
{
public:
    // ---------- start config from config.yaml ---------------------------------------------
    // 这部分的变量，是直接从配置文件中读取到的原始值，如果在配置文件中新增了配置，在这里添加对应的变量存储
    // 这些值会在 BrainNode 中被覆盖，所以即使 config.yaml 中没显示配置，这里的默认值不会生效。
    // 真正要设置默认值得在 BrainNode 的 declare_parameter 里配置
    int teamId;                 // 对应 game.team_id
    int playerId;               // 对应 game.player_id
    string fieldType;           // 对应 game.field_type  球场类型, "adult_size"(14*9) | "kid_size" (9*6)
    string playerRole;          // 对应 game.player_role   "striker" | "goal_keeper"
    string playerStartPos;      // 对应 game.player_start_post  "left" | "right", 从自己半场的左侧还是右侧上场
    
    double robotHeight;         // 对应 robot.robot_height 机器人的身高(m), 用于估算距离, 可以通过 SetParam 节点进行调试. In behaviortree xml: <SetParam code="robot_height=1.1" />
    double robotOdomFactor;     // 对应 robot.odom_factor odom 认为走的距离与机器人实际走的距离的比值, 用于修正 odom
    double vxFactor;            // 对应 robot.vx_factor 修正 vx 实际比指令大的问题
    double yawOffset;           // 对应 robot.yaw_offset 修正测距时往左偏的问题 

    bool enableCom;             // 对应 enable_com 是否开启通信

    bool rerunLogEnableTCP;     // 对应 rerunLog.enable_tcp  是否开启 rerunLog 的 TCP 传输
    string rerunLogServerIP;    // 对应 rerunLog.server_ip  rerunLog 服务器 IP
    bool rerunLogEnableFile;    // 对应 rerunLog.enable_file  是否开启 rerunLog 的文件传输
    string rerunLogLogDir;      // 对应 rerunLog.log_dir  rerunLog 文件的路径
    double rerunLogMaxFileMins;         // 一个 log 文件最长多少分钟, 太长会导致读的时候死机

    int rerunLogImgInterval;    // 对应 rerunLog.img_interval 每多少次消息记录一次 log 一次 img, 提升这一值可以减少 log 大小, 提升 log 传输速度.
    
    string treeFilePath;        // 现在没放在 config.yaml 中了，放在 launch.py 中指定，行为树文件的路径
    // ----------  end config from config.yaml ---------------------------------------------

    // game 参数
    FieldDimensions fieldDimensions; // 球场尺寸
    vector<FieldLine> mapLines;       // 地图上的标线的理论位置, 用于比较和识别看到的标线
    vector<MapMarking> mapMarkings;   // 地图上的标志点的理论位置, 用于比较和识别看到的标志点
    
    int numOfPlayers = 2;            // 2; // 机器人数量, 2及2以上，用于做自动决策切换

    // 避障参数
    double collisionThreshold;        // 碰撞阈值, 当障碍物距离小于此值时, 认为需要避障
    double safeDistance;              // 安全距离, 当障碍物距离小于此值时, 认为需要避障
    double avoidSecs;                 // 避障时, 每次绕行障碍物的时长

    // ** 相机相关参数, 会被 config 中的值 override ** 
    // 相机像素
    double camPixX = 1280;
    double camPixY = 720;

    // 相机视角
    double camAngleX = deg2rad(90);
    double camAngleY = deg2rad(65);

    // 相机内参
    double camfx = 643.898;
    double camfy = 643.216;
    double camcx = 649.038;
    double camcy = 357.21;

    // 外参
    Eigen::Matrix4d camToHead;


    // 头转动软限位
    double headYawLimitLeft = 1.1;
    double headYawLimitRight = -1.1;
    double headPitchLimitUp = 0.2; // 这个角度足以看到全场, 更高更远的信息全是干扰，k1 realsense 0.3 to 0.2

    // 速度上限
    double vxLimit = 1.2;
    double vyLimit = 0.4;
    double vthetaLimit = 1.5;

    // 策略参数
    double safeDist = 2.0;                  // 用于碰撞检测的安全距离, 小于这个距离认为碰撞
    double goalPostMargin = 0.4;
    double goalPostMarginForTouch = 0.1; // 计算球门柱的margin，用于计算球门柱的角度，越大则计算时候球门越小，touch的时候，这个margin不一样，通常会更小
    // double memoryLength = 3.0;           // 连续多少秒看不到球时, 就认为球丢了
    double ballConfidenceThreshold;        // confidence 低于这个阈值, 认为不是球(注意detection模块目前传入目标置信度都是 > 20 的)
    double ballConfidenceDecayRate;        // # 看到 confidence 为 100 的球后, 多少秒没有再看到, 则认为球丢了. 最后看到的球信息越低, 则越快认为球已经丢了.
    bool enableStableKick = false;        // 开启后, 在 kick 时, 如风险较低, 则稳定一下再出脚.
    bool treatPersonAsRobot = false;     // 是否把人当作机器人处理, 用于调试
    double ballOutThreshold = 2.0;        // 球出界的阈值, 用于判断球是否出界
    double tmBallDistThreshold = 2.0;      // 队友发送的球的位置与我当前位置的距离, 如果大于此值, 则认为队友的信息准确. (因为双方定位精度问题, 如果这个距离太小, 则会使我产生大方向上的误判. 例如, 队友给的球的位置在我身前, 而实际球在我身后. 因为队友给我的不是球相对于我的位置, 而是相对于球场的位置)
    bool limitNearBallSpeed = true;        // 是否限制在 chase 时靠近球时的速度
    double nearBallSpeedLimit = 0.6;       // 靠近球时的速度限制
    double nearBallRange = 2.0;            // 靠近球时的范围, 在这个范围内, 限制速度

    // locator 参数
    int pfMinMarkerCnt = 5; // 最少看到多少到 marker, 才可以进行粒子滤波定位
    double pfMaxResidual = 0.3; // 粒子滤波定位时, 最终每个粒子的加权偏差大于此值时, 认为定位不成功

    // sound 参数
    bool soundEnable = false;
    string soundPack = "espeak"; // espeak 或 语音名名称, 假设语音包名称为 <name>, 相应的语音文件需放到: sound_play/sounds/<name>/ 目录下

    // RLVisionKick 参数
    string RLVisionKickVisualKickVersion; // 对应 RLVisionKick.visual_kick_version  kV1 | kV2

    // Recovery 参数
    string recoveryGetUpVersion; // 对应 recovery.get_up_version  kV1 | kV2

    // 计算地面标线与标志点的理论值
    void calcMapLines();
    void calcMapMarkings();

    // BrainNode 填充完参数后，调用 handle() 进行一些参数的处理（校正、计算等）,成功返回 true
    void handle();
    
    // 把配置信息输出到指定的输出流中(debug用)
    void print(ostream &os);
};
